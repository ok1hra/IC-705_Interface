// PROTOTYPE — deterministic AWGN sensitivity comparison for direct 12 kHz and
// the candidate 12 kHz -> 8 kHz uLaw AUD1 -> 12 kHz receive path.

#include "audio_frontend.hpp"
#include "decoder_c_api.h"
#include "js8_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <sstream>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr std::string_view kRaw = "TrMcT8++++++";
constexpr int kType = 7;
constexpr std::array<int, 14> kSnrDb = {6, 3, 0, -3, -6, -9, -12,
                                        -15, -18, -21, -24, -27, -30, -33};

struct DecodeCapture {
    std::vector<Js8ReferenceDecode> frames;
};

void capture(Js8ReferenceDecode const *decoded, void *context) {
    static_cast<DecodeCapture *>(context)->frames.push_back(*decoded);
}

struct DecodeResult {
    bool exact{};
    int reportedSnr{};
};

DecodeResult decode(std::span<const std::int16_t> pcm, int mode) {
    DecodeCapture captured;
    auto const count = js8_reference_decode(pcm.data(), pcm.size(), mode,
                                            0, 5000, 1500,
                                            capture, &captured);
    if (count < 0) throw std::runtime_error("reference decoder rejected input");
    for (auto const &frame : captured.frames) {
        if (frame.submode == mode && frame.type == kType && frame.data == kRaw)
            return {true, frame.snr};
    }
    return {};
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
            if (result.size() < count)
                result.push_back(radius * std::sin(angle));
        }
        double mean = 0.0;
        for (auto value : result) mean += value;
        mean /= static_cast<double>(result.size());
        double square = 0.0;
        for (auto &value : result) {
            value -= mean;
            square += value * value;
        }
        auto const rms = std::sqrt(square / static_cast<double>(result.size()));
        for (auto &value : result) value /= rms;
        return result;
    }

  private:
    std::uint64_t next() {
        state_ ^= state_ >> 12;
        state_ ^= state_ << 25;
        state_ ^= state_ >> 27;
        return state_ * 2685821657736338717ULL;
    }

    double uniform() {
        constexpr double denominator = 9007199254740992.0;
        return (static_cast<double>(next() >> 11) + 0.5) / denominator;
    }

    std::uint64_t state_;
};

std::vector<std::int16_t> addNoise(std::span<const std::int16_t> signal,
                                   std::span<const double> noise,
                                   int snrDb) {
    double signalSquare = 0.0;
    for (auto sample : signal) {
        auto const value = static_cast<double>(sample) / 32768.0;
        signalSquare += value * value;
    }
    auto const signalRms = std::sqrt(signalSquare / signal.size());
    auto const noiseRms = signalRms / std::pow(10.0, snrDb / 20.0);
    std::vector<double> mixed(signal.size());
    double peak = 0.0;
    for (std::size_t i = 0; i < signal.size(); ++i) {
        mixed[i] = static_cast<double>(signal[i]) / 32768.0 + noise[i] * noiseRms;
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

std::vector<std::int16_t> ulawRoundtrip(std::span<const std::int16_t> input) {
    std::vector<float> pcm12k(input.size());
    std::transform(input.begin(), input.end(), pcm12k.begin(),
                   [](auto sample) { return sample / 32768.0f; });
    js8proto::SincResampler downsampler(12000, 8000);
    auto pcm8k = downsampler.push(pcm12k);
    auto downTail = downsampler.finish();
    pcm8k.insert(pcm8k.end(), downTail.begin(), downTail.end());
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
    auto tail = timeline.finish();
    restored.insert(restored.end(), tail.begin(), tail.end());
    if (restored.size() != input.size())
        throw std::runtime_error("uLaw roundtrip changed sample count");
    std::vector<std::int16_t> result(restored.size());
    std::transform(restored.begin(), restored.end(), result.begin(), [](float value) {
        return static_cast<std::int16_t>(std::lround(
            std::clamp(value, -1.0f, 0.999969f) * 32768.0f));
    });
    return result;
}

void writeU16(std::ostream &output, std::uint16_t value) {
    char bytes[2] = {static_cast<char>(value), static_cast<char>(value >> 8)};
    output.write(bytes, sizeof(bytes));
}

void writeU32(std::ostream &output, std::uint32_t value) {
    char bytes[4] = {static_cast<char>(value), static_cast<char>(value >> 8),
                     static_cast<char>(value >> 16), static_cast<char>(value >> 24)};
    output.write(bytes, sizeof(bytes));
}

void writeWave(std::filesystem::path const &path,
               std::span<const std::int16_t> pcm) {
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot create weak-signal WAV");
    auto const bytes = static_cast<std::uint32_t>(pcm.size() * 2);
    output.write("RIFF", 4); writeU32(output, 36 + bytes);
    output.write("WAVEfmt ", 8); writeU32(output, 16); writeU16(output, 1);
    writeU16(output, 1); writeU32(output, 12000); writeU32(output, 24000);
    writeU16(output, 2); writeU16(output, 16);
    output.write("data", 4); writeU32(output, bytes);
    output.write(reinterpret_cast<char const *>(pcm.data()), bytes);
}

char modeLetter(int mode) {
    switch (mode) {
    case 0: return 'A'; case 1: return 'B'; case 2: return 'C';
    case 4: return 'E'; case 8: return 'I'; default: return 'X';
    }
}

std::string levelToken(int value) {
    std::ostringstream stream;
    stream << (value < 0 ? 'm' : 'p') << std::setw(2) << std::setfill('0')
           << std::abs(value);
    return stream.str();
}

struct Point {
    int snrDb{};
    std::string wav;
    DecodeResult direct;
    DecodeResult ulaw;
};

struct ModeResult {
    int mode{};
    char letter{};
    std::size_t samples{};
    std::vector<Point> points;
    std::optional<int> directFloor;
    std::optional<int> ulawFloor;
    int degradationDb{};
    std::string selectedWav;
    bool pass{};
};

template <typename Predicate>
std::optional<int> floorFor(std::vector<Point> const &points, Predicate predicate) {
    std::optional<int> floor;
    for (auto const &point : points) {
        if (!predicate(point)) break;
        floor = point.snrDb;
    }
    return floor;
}

std::string optionalJson(std::optional<int> value) {
    return value ? std::to_string(*value) : "null";
}
} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: js8-weak-signal OUTPUT_DIR\n";
        return 2;
    }
    try {
        std::filesystem::path const outputDir(argv[1]);
        std::filesystem::create_directories(outputDir);
        std::vector<ModeResult> modes;
        bool allPassed = true;

        for (auto const &spec : js8proto::submodes()) {
            auto encoded = js8proto::encodeFrame(kRaw, kType, spec.id);
            if (!encoded) throw std::runtime_error("encoder rejected weak vector");
            auto const signal = js8proto::modulateFrame12k(*encoded);
            StableNoise generator(0x4a53380000000000ULL +
                                  static_cast<unsigned>(spec.id));
            auto const noise = generator.gaussian(signal.size());
            ModeResult mode;
            mode.mode = static_cast<int>(spec.id);
            mode.letter = modeLetter(mode.mode);
            mode.samples = signal.size();
            for (auto snrDb : kSnrDb) {
                auto noisy = addNoise(signal, noise, snrDb);
                auto const filename = std::string(1, mode.letter) + "_snr_" +
                                      levelToken(snrDb) + ".wav";
                writeWave(outputDir / filename, noisy);
                auto const direct = decode(noisy, mode.mode);
                auto const roundtrip = decode(ulawRoundtrip(noisy), mode.mode);
                mode.points.push_back({snrDb, filename, direct, roundtrip});
                std::cout << "WEAK mode=" << mode.mode << " letter=" << mode.letter
                          << " snr_db=" << snrDb
                          << " direct=" << (direct.exact ? "PASS" : "FAIL")
                          << " ulaw=" << (roundtrip.exact ? "PASS" : "FAIL")
                          << " decoder_snr_direct=" << direct.reportedSnr
                          << " decoder_snr_ulaw=" << roundtrip.reportedSnr << '\n';
            }
            mode.directFloor = floorFor(mode.points,
                [](Point const &point) { return point.direct.exact; });
            mode.ulawFloor = floorFor(mode.points,
                [](Point const &point) { return point.ulaw.exact; });
            mode.degradationDb = mode.directFloor && mode.ulawFloor
                ? *mode.ulawFloor - *mode.directFloor : 999;
            mode.pass = mode.directFloor && mode.ulawFloor &&
                        mode.points.front().direct.exact &&
                        mode.points.front().ulaw.exact && mode.degradationDb <= 3;
            if (mode.ulawFloor) {
                auto const found = std::find_if(mode.points.begin(), mode.points.end(),
                    [&](Point const &point) { return point.snrDb == *mode.ulawFloor; });
                mode.selectedWav = std::string(1, mode.letter) + "_weak_selected.wav";
                std::filesystem::copy_file(outputDir / found->wav,
                    outputDir / mode.selectedWav,
                    std::filesystem::copy_options::overwrite_existing);
            }
            allPassed &= mode.pass;
            std::cout << "WEAK FLOOR mode=" << mode.mode
                      << " direct_db=" << optionalJson(mode.directFloor)
                      << " ulaw_db=" << optionalJson(mode.ulawFloor)
                      << " degradation_db=" << mode.degradationDb
                      << " result=" << (mode.pass ? "PASS" : "FAIL") << '\n';
            modes.push_back(std::move(mode));
        }

        std::ofstream manifest(outputDir / "manifest.json");
        if (!manifest) throw std::runtime_error("cannot create weak manifest");
        manifest << "{\n  \"question\": \"does 8k uLaw materially reduce deterministic AWGN sensitivity?\",\n"
                 << "  \"snrDefinition\": \"time-domain signal RMS / full-band AWGN RMS; not decoder SNR\",\n"
                 << "  \"stepDb\": 3,\n  \"maxAllowedDegradationDb\": 3,\n"
                 << "  \"raw\": \"" << kRaw << "\",\n"
                 << "  \"text\": \"KN4CRD: TEST\",\n"
                 << "  \"nativePass\": " << (allPassed ? "true" : "false")
                 << ",\n  \"vectors\": [\n";
        for (std::size_t index = 0; index < modes.size(); ++index) {
            auto const &mode = modes[index];
            manifest << "    {\"letter\":\"" << mode.letter
                     << "\",\"submode\":" << mode.mode
                     << ",\"samples\":" << mode.samples
                     << ",\"directFloorDb\":" << optionalJson(mode.directFloor)
                     << ",\"ulawFloorDb\":" << optionalJson(mode.ulawFloor)
                     << ",\"degradationDb\":" << mode.degradationDb
                     << ",\"selectedWav\":\"" << mode.selectedWav
                     << "\",\"pass\":" << (mode.pass ? "true" : "false")
                     << ",\"points\":[";
            for (std::size_t p = 0; p < mode.points.size(); ++p) {
                auto const &point = mode.points[p];
                manifest << "{\"snrDb\":" << point.snrDb
                         << ",\"wav\":\"" << point.wav
                         << "\",\"direct\":" << (point.direct.exact ? "true" : "false")
                         << ",\"ulaw\":" << (point.ulaw.exact ? "true" : "false")
                         << "}" << (p + 1 == mode.points.size() ? "" : ",");
            }
            manifest << "]}" << (index + 1 == modes.size() ? "\n" : ",\n");
        }
        manifest << "  ]\n}\n";
        std::cout << "WEAK NATIVE " << (allPassed ? "PASS" : "FAIL") << '\n';
        return 0;
    } catch (std::exception const &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
