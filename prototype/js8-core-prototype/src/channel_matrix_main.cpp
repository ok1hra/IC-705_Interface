// PROTOTYPE — repeatable channel-impairment matrix above the AWGN decode floor.
// Question: does the 8 kHz uLaw path add failures under noise-seed variation,
// audio offsets, slow fading, adjacent QRM or small within-frame drift?

#include "audio_frontend.hpp"
#include "decoder_c_api.h"
#include "js8_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr std::string_view kRaw = "TrMcT8++++++";
constexpr int kType = 7;
constexpr std::array<int, 5> kOffsetsHz = {700, 1100, 1500, 1900, 2300};

struct Captured { std::vector<Js8ReferenceDecode> frames; };

void capture(Js8ReferenceDecode const *decoded, void *context) {
    static_cast<Captured *>(context)->frames.push_back(*decoded);
}

bool decodeExact(std::span<const std::int16_t> pcm, int mode) {
    Captured captured;
    auto const count = js8_reference_decode(pcm.data(), pcm.size(), mode,
                                            0, 5000, 1500,
                                            capture, &captured);
    if (count < 0) throw std::runtime_error("reference decoder rejected matrix input");
    return std::any_of(captured.frames.begin(), captured.frames.end(),
        [mode](auto const &frame) {
            return frame.submode == mode && frame.type == kType && frame.data == kRaw;
        });
}

class StableNoise {
  public:
    explicit StableNoise(std::uint64_t seed) : state_(seed) {}

    std::vector<double> gaussian(std::size_t count) {
        std::vector<double> result;
        result.reserve(count);
        while (result.size() < count) {
            auto const radius = std::sqrt(-2.0 * std::log(uniform()));
            auto const angle = 2.0 * std::numbers::pi * uniform();
            result.push_back(radius * std::cos(angle));
            if (result.size() < count) result.push_back(radius * std::sin(angle));
        }
        double mean = 0.0;
        for (auto value : result) mean += value;
        mean /= result.size();
        double square = 0.0;
        for (auto &value : result) { value -= mean; square += value * value; }
        auto const rms = std::sqrt(square / result.size());
        for (auto &value : result) value /= rms;
        return result;
    }

  private:
    std::uint64_t next() {
        state_ ^= state_ >> 12; state_ ^= state_ << 25; state_ ^= state_ >> 27;
        return state_ * 2685821657736338717ULL;
    }
    double uniform() {
        return (static_cast<double>(next() >> 11) + 0.5) / 9007199254740992.0;
    }
    std::uint64_t state_;
};

std::vector<double> asDouble(std::span<const std::int16_t> pcm) {
    std::vector<double> result(pcm.size());
    std::transform(pcm.begin(), pcm.end(), result.begin(),
                   [](auto value) { return value / 32768.0; });
    return result;
}

double rms(std::span<const double> values) {
    double square = 0.0;
    for (auto value : values) square += value * value;
    return std::sqrt(square / values.size());
}

std::vector<std::int16_t> mixNoise(std::span<const double> signal,
                                   std::span<const double> noise,
                                   double referenceSignalRms,
                                   int injectedSnrDb) {
    auto const noiseRms = referenceSignalRms /
                          std::pow(10.0, injectedSnrDb / 20.0);
    std::vector<double> mixed(signal.size());
    double peak = 0.0;
    for (std::size_t i = 0; i < signal.size(); ++i) {
        mixed[i] = signal[i] + noise[i] * noiseRms;
        peak = std::max(peak, std::abs(mixed[i]));
    }
    auto const gain = peak > 0.90 ? 0.90 / peak : 1.0;
    std::vector<std::int16_t> result(mixed.size());
    std::transform(mixed.begin(), mixed.end(), result.begin(), [gain](double value) {
        return static_cast<std::int16_t>(std::lround(
            std::clamp(value * gain, -1.0, 0.999969) * 32768.0));
    });
    return result;
}

std::vector<double> faded(std::span<const double> signal, double depthDb) {
    std::vector<double> result(signal.size());
    for (std::size_t i = 0; i < signal.size(); ++i) {
        auto const progress = static_cast<double>(i) /
                              std::max<std::size_t>(1, signal.size() - 1);
        auto const attenuationDb = depthDb * 0.5 *
            (1.0 - std::cos(4.0 * std::numbers::pi * progress));
        result[i] = signal[i] * std::pow(10.0, -attenuationDb / 20.0);
    }
    return result;
}

std::vector<double> withQrm(std::span<const double> signal,
                            double signalRms, double frequencyHz,
                            double relativeDb) {
    auto result = std::vector<double>(signal.begin(), signal.end());
    auto const amplitude = signalRms * std::pow(10.0, relativeDb / 20.0) *
                           std::sqrt(2.0);
    for (std::size_t i = 0; i < result.size(); ++i)
        result[i] += amplitude * std::sin(2.0 * std::numbers::pi *
                                          frequencyHz * i / 12000.0);
    return result;
}

std::vector<std::int16_t> modulateWithDrift(js8proto::EncodedFrame const &frame,
                                            double baseHz, double spanHz) {
    auto const *mode = js8proto::findSubmode(static_cast<int>(frame.submode));
    if (!mode) return {};
    auto const periodSamples = static_cast<std::size_t>(mode->periodMs * 12);
    auto const delaySamples = static_cast<std::size_t>(mode->startDelayMs * 12);
    auto const samplesPerSymbol = mode->samplesPerSymbol12k;
    auto const signalSamples = samplesPerSymbol * frame.tones.size();
    std::vector<std::int16_t> pcm(periodSamples, 0);
    auto const rampSamples = std::size_t{60};
    double phase = 0.0;
    std::size_t signalIndex = 0;
    for (auto tone : frame.tones) {
        for (int i = 0; i < samplesPerSymbol; ++i, ++signalIndex) {
            auto const progress = static_cast<double>(signalIndex) /
                                  std::max<std::size_t>(1, signalSamples - 1);
            auto const frequency = baseHz + tone * mode->toneSpacingHz() +
                                   spanHz * (progress - 0.5);
            phase = std::fmod(phase + 2.0 * std::numbers::pi * frequency / 12000.0,
                              2.0 * std::numbers::pi);
            auto envelope = 1.0;
            if (signalIndex < rampSamples)
                envelope = 0.5 - 0.5 * std::cos(std::numbers::pi *
                                                signalIndex / rampSamples);
            auto const remaining = signalSamples - signalIndex - 1;
            if (remaining < rampSamples)
                envelope *= 0.5 - 0.5 * std::cos(std::numbers::pi *
                                                 remaining / rampSamples);
            pcm[delaySamples + signalIndex] = static_cast<std::int16_t>(
                std::lround(0.65 * envelope * std::sin(phase) * 32768.0));
        }
    }
    return pcm;
}

std::vector<std::int16_t> ulawRoundtrip(std::span<const std::int16_t> input) {
    std::vector<float> pcm12k(input.size());
    std::transform(input.begin(), input.end(), pcm12k.begin(),
                   [](auto sample) { return sample / 32768.0f; });
    js8proto::SincResampler downsampler(12000, 8000);
    auto pcm8k = downsampler.push(pcm12k);
    auto tail8 = downsampler.finish();
    pcm8k.insert(pcm8k.end(), tail8.begin(), tail8.end());
    std::vector<std::uint8_t> ulaw(pcm8k.size());
    std::transform(pcm8k.begin(), pcm8k.end(), ulaw.begin(),
                   js8proto::encodeUlaw);
    js8proto::AudioTimeline timeline;
    std::vector<float> restored;
    std::uint32_t sequence = 0;
    for (std::size_t offset = 0; offset < ulaw.size(); offset += 160) {
        auto const count = std::min<std::size_t>(160, ulaw.size() - offset);
        js8proto::AudioPacket packet{sequence++, offset, offset / 8.0, false,
            std::vector<std::uint8_t>(ulaw.begin() + offset,
                                      ulaw.begin() + offset + count)};
        auto block = timeline.ingest(packet);
        restored.insert(restored.end(), block.pcm12k.begin(), block.pcm12k.end());
    }
    auto tail12 = timeline.finish();
    restored.insert(restored.end(), tail12.begin(), tail12.end());
    if (restored.size() != input.size())
        throw std::runtime_error("matrix roundtrip changed sample count");
    std::vector<std::int16_t> result(restored.size());
    std::transform(restored.begin(), restored.end(), result.begin(), [](float value) {
        return static_cast<std::int16_t>(std::lround(
            std::clamp(value, -1.0f, 0.999969f) * 32768.0f));
    });
    return result;
}

int marginSnrDb(int mode) {
    switch (mode) {
    case 4: case 0: return -21;
    case 1: return -18;
    case 2: case 8: return -12;
    default: throw std::runtime_error("unknown mode");
    }
}

struct CaseResult { std::string name; bool direct{}; bool ulaw{}; };

CaseResult evaluate(std::string name, std::vector<std::int16_t> const &pcm,
                    int mode) {
    auto const direct = decodeExact(pcm, mode);
    auto const ulaw = decodeExact(ulawRoundtrip(pcm), mode);
    std::cout << "CHANNEL mode=" << mode << " case=" << name
              << " direct=" << (direct ? "PASS" : "FAIL")
              << " ulaw=" << (ulaw ? "PASS" : "FAIL") << '\n';
    return {std::move(name), direct, ulaw};
}
} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: js8-channel-matrix OUTPUT_DIR\n";
        return 2;
    }
    try {
        std::filesystem::path const outputDir(argv[1]);
        std::filesystem::create_directories(outputDir);
        std::ofstream manifest(outputDir / "matrix.json");
        if (!manifest) throw std::runtime_error("cannot create channel matrix manifest");
        manifest << "{\n  \"question\": \"does uLaw regress at +3 dB margin and do both paths pass at +6 dB?\",\n"
                 << "  \"vectors\": [\n";
        bool allPassed = true;
        auto const &modes = js8proto::submodes();
        for (std::size_t modeIndex = 0; modeIndex < modes.size(); ++modeIndex) {
            auto const &spec = modes[modeIndex];
            auto encoded = js8proto::encodeFrame(kRaw, kType, spec.id);
            if (!encoded) throw std::runtime_error("encoder rejected matrix frame");
            auto const cleanPcm = js8proto::modulateFrame12k(*encoded, 1500.0);
            auto const clean = asDouble(cleanPcm);
            auto const referenceRms = rms(clean);
            auto const mode = static_cast<int>(spec.id);
            auto const stressSnrDb = marginSnrDb(mode);
            auto const safeSnrDb = stressSnrDb + 3;
            auto runTier = [&](int snrDb, std::string const &tier) {
                std::vector<CaseResult> result;
                for (int seed = 0; seed < 5; ++seed) {
                    StableNoise noise(0x43484e0000000000ULL + mode * 257 + seed);
                    result.push_back(evaluate(tier + "/seed-" +
                        std::to_string(seed + 1), mixNoise(clean,
                        noise.gaussian(clean.size()), referenceRms, snrDb), mode));
                }
                for (auto offsetHz : kOffsetsHz) {
                    auto offsetPcm = js8proto::modulateFrame12k(*encoded, offsetHz);
                    auto offset = asDouble(offsetPcm);
                    StableNoise noise(0x4f46460000000000ULL + mode * 257 + offsetHz);
                    result.push_back(evaluate(tier + "/offset-" +
                        std::to_string(offsetHz), mixNoise(offset,
                        noise.gaussian(offset.size()), referenceRms, snrDb), mode));
                }
                StableNoise fadeNoise(0x4641444500000000ULL + mode);
                auto fade = faded(clean, 6.0);
                result.push_back(evaluate(tier + "/fade-6db", mixNoise(fade,
                    fadeNoise.gaussian(fade.size()), referenceRms, snrDb), mode));

                StableNoise qrmNoise(0x51524d0000000000ULL + mode);
                auto qrm = withQrm(clean, referenceRms,
                    1500.0 + spec.bandwidthHz() + spec.toneSpacingHz(), -6.0);
                result.push_back(evaluate(tier + "/adjacent-qrm-m6db",
                    mixNoise(qrm, qrmNoise.gaussian(qrm.size()),
                             referenceRms, snrDb), mode));

                auto driftPcm = modulateWithDrift(*encoded, 1500.0,
                                                   spec.toneSpacingHz() * 0.25);
                auto drift = asDouble(driftPcm);
                StableNoise driftNoise(0x4452494654000000ULL + mode);
                result.push_back(evaluate(tier + "/drift-quarter-tone",
                    mixNoise(drift, driftNoise.gaussian(drift.size()),
                             referenceRms, snrDb), mode));
                return result;
            };
            auto cases = runTier(stressSnrDb, "stress");
            auto safeCases = runTier(safeSnrDb, "safe");
            auto const codecParityPass = std::all_of(cases.begin(), cases.end(),
                [](auto const &value) { return !value.direct || value.ulaw; });
            auto const stressAllPass = std::all_of(cases.begin(), cases.end(),
                [](auto const &value) { return value.direct && value.ulaw; });
            auto const safePass = std::all_of(safeCases.begin(), safeCases.end(),
                [](auto const &value) { return value.direct && value.ulaw; });
            auto const modePass = codecParityPass && safePass;
            allPassed &= modePass;
            manifest << "    {\"submode\":" << mode
                     << ",\"stressSnrDb\":" << stressSnrDb
                     << ",\"safeSnrDb\":" << safeSnrDb
                     << ",\"toneSpacingHz\":" << spec.toneSpacingHz()
                     << ",\"codecParityPass\":"
                     << (codecParityPass ? "true" : "false")
                     << ",\"stressAllPass\":" << (stressAllPass ? "true" : "false")
                     << ",\"safePass\":" << (safePass ? "true" : "false")
                     << ",\"pass\":" << (modePass ? "true" : "false")
                     << ",\"cases\":[";
            cases.insert(cases.end(), safeCases.begin(), safeCases.end());
            for (std::size_t i = 0; i < cases.size(); ++i) {
                auto const &value = cases[i];
                manifest << "{\"name\":\"" << value.name
                         << "\",\"direct\":" << (value.direct ? "true" : "false")
                         << ",\"ulaw\":" << (value.ulaw ? "true" : "false") << "}"
                         << (i + 1 == cases.size() ? "" : ",");
            }
            manifest << "]}" << (modeIndex + 1 == modes.size() ? "\n" : ",\n");
            std::cout << "CHANNEL MODE mode=" << mode
                      << " stress_codec=" << (codecParityPass ? "PASS" : "FAIL")
                      << " stress_all=" << (stressAllPass ? "PASS" : "LIMIT")
                      << " safe_all=" << (safePass ? "PASS" : "FAIL")
                      << " result=" << (modePass ? "PASS" : "FAIL") << '\n';
        }
        manifest << "  ],\n  \"pass\": " << (allPassed ? "true" : "false") << "\n}\n";
        std::cout << "CHANNEL MATRIX " << (allPassed ? "PASS" : "FAIL") << '\n';
        return allPassed ? 0 : 1;
    } catch (std::exception const &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
