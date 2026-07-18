// PROTOTYPE — interactive packet/timebase shell and fixture round-trip.

#include "audio_frontend.hpp"
#include "aud1_protocol.hpp"
#include "decoder_c_api.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using js8proto::AudioPacket;
constexpr std::uint32_t kStreamId = 0x4a533831;

std::vector<std::uint8_t> aud1Wire(std::uint32_t sequence,
                                   std::uint64_t firstSample,
                                   std::span<const std::uint8_t> payload) {
    js8proto::Aud1Header header{js8proto::Aud1Kind::RxUlaw,
                               static_cast<std::uint16_t>(
                                   sequence == 0 ? js8proto::Aud1First : 0),
                               kStreamId,
                               sequence,
                               8000,
                               firstSample,
                               0};
    return js8proto::encodeAud1(header, payload);
}

AudioPacket throughAud1(std::uint32_t sequence, std::uint64_t firstSample,
                        double arrivalMs,
                        std::span<const std::uint8_t> payload) {
    auto const wire = aud1Wire(sequence, firstSample, payload);
    auto decoded = js8proto::decodeAud1(wire);
    if (!decoded || decoded->header.streamId != kStreamId ||
        decoded->header.kind != js8proto::Aud1Kind::RxUlaw)
        throw std::runtime_error("AUD1 encode/decode rejected RX packet");
    return {decoded->header.sequence,
            decoded->header.firstSample,
            arrivalMs,
            (decoded->header.flags & js8proto::Aud1Discontinuity) != 0,
            std::move(decoded->payload)};
}

struct InteractiveState {
    js8proto::AudioTimeline timeline;
    js8proto::DecodeScheduler scheduler{0};
    std::uint32_t sequence{};
    std::uint64_t firstSample8k{};
    std::optional<AudioPacket> lastPacket;
    std::array<std::uint64_t, 5> readyByLane{};
    std::vector<js8proto::DecodeWindow> lastReady;
    std::string lastAction = "new in-memory epoch";
};

std::size_t laneIndex(js8proto::Submode mode) {
    auto const &modes = js8proto::submodes();
    auto const found = std::find_if(modes.begin(), modes.end(),
                                    [mode](auto const &value) {
                                        return value.id == mode;
                                    });
    return static_cast<std::size_t>(std::distance(modes.begin(), found));
}

std::vector<std::uint8_t> sinePacket(std::uint64_t firstSample,
                                     std::size_t count = 160) {
    std::vector<std::uint8_t> encoded(count);
    for (std::size_t i = 0; i < count; ++i) {
        auto const phase = 2.0 * std::numbers::pi * 1000.0 *
                           static_cast<double>(firstSample + i) / 8000.0;
        encoded[i] = js8proto::encodeUlaw(
            static_cast<float>(0.25 * std::sin(phase)));
    }
    return encoded;
}

void collectReady(InteractiveState &state,
                  std::vector<js8proto::DecodeWindow> ready) {
    state.lastReady = std::move(ready);
    for (auto const &window : state.lastReady)
        ++state.readyByLane[laneIndex(window.submode)];
}

void pushPacket(InteractiveState &state, double extraArrivalMs = 0.0) {
    auto const payload = sinePacket(state.firstSample8k);
    auto packet = throughAud1(
        state.sequence, state.firstSample8k,
        static_cast<double>(state.firstSample8k) / 8.0 + extraArrivalMs,
        payload);
    auto result = state.timeline.ingest(packet);
    collectReady(state, state.scheduler.appendThrough(
                            state.timeline.state().producedSamples12k));
    state.lastPacket = packet;
    ++state.sequence;
    state.firstSample8k += packet.ulaw.size();
    state.lastAction = "accepted one 20 ms packet; produced " +
                       std::to_string(result.pcm12k.size()) + " samples @12k";
}

void pushPackets(InteractiveState &state, int count) {
    for (int i = 0; i < count; ++i)
        pushPacket(state);
    state.lastAction = "accepted " + std::to_string(count) +
                       " packets (" + std::to_string(count * 20) + " ms)";
}

void dropAndContinue(InteractiveState &state) {
    ++state.sequence;
    state.firstSample8k += 160;
    pushPacket(state);
    state.lastAction = "dropped 20 ms, next packet inserted a 160-sample gap";
}

void duplicateLast(InteractiveState &state) {
    if (!state.lastPacket) {
        state.lastAction = "no packet available to duplicate";
        return;
    }
    auto const result = state.timeline.ingest(*state.lastPacket);
    collectReady(state, state.scheduler.appendThrough(
                            state.timeline.state().producedSamples12k));
    state.lastAction = result.duplicate
        ? "duplicate rejected without advancing media time"
        : "unexpected: duplicate was accepted";
}

void render(InteractiveState const &state) {
    auto const timeline = state.timeline.state();
    std::cout << "\033[2J\033[H\033[1mPROTOTYPE — 8k µ-law packet timeline\033[0m\n"
              << "\033[2mQuestion: do gaps/jitter change media time or slot scheduling?\033[0m\n\n"
              << "\033[1mCurrent state\033[0m\n"
              << "anchored:              " << (timeline.anchored ? "yes" : "no") << '\n'
              << "next packet:           seq=" << state.sequence
              << " firstSample8k=" << state.firstSample8k << '\n'
              << "accepted/duplicates:   " << timeline.acceptedPackets << " / "
              << timeline.duplicatePackets << '\n'
              << "sequence gaps:         " << timeline.sequenceGaps << '\n'
              << "discontinuities:       " << timeline.discontinuities << '\n'
              << "inserted silence:      " << timeline.insertedGapSamples8k
              << " samples @8k\n"
              << "produced timeline:     " << timeline.producedSamples12k
              << " samples @12k ("
              << std::fixed << std::setprecision(3)
              << timeline.producedSamples12k / 12000.0 << " s)\n"
              << "max arrival jitter:    " << timeline.maxArrivalJitterMs << " ms\n"
              << "peak/RMS after DC:     " << timeline.peak << " / "
              << timeline.rms << '\n'
              << "ready windows E/A/B/C/I:";
    for (auto value : state.readyByLane)
        std::cout << ' ' << value;
    std::cout << "\nlast ready windows:    ";
    if (state.lastReady.empty()) {
        std::cout << "none";
    } else {
        for (auto const &window : state.lastReady)
            std::cout << static_cast<int>(window.submode) << '@'
                      << window.slotUtcMs << "ms ";
    }
    std::cout << "\nlast action:           " << state.lastAction << "\n\n"
              << "\033[1mCommands\033[0m\n"
              << "[p] packet  [s] 1 second  [m] 60 seconds  [g] gap+packet\n"
              << "[d] duplicate  [j] packet +250 ms arrival jitter  [r] reset  [q] quit\n> "
              << std::flush;
}

void interactive() {
    InteractiveState state;
    while (true) {
        render(state);
        std::string action;
        if (!std::getline(std::cin, action) || action == "q")
            return;
        if (action == "p")
            pushPacket(state);
        else if (action == "s")
            pushPackets(state, 50);
        else if (action == "m")
            pushPackets(state, 3000);
        else if (action == "g")
            dropAndContinue(state);
        else if (action == "d")
            duplicateLast(state);
        else if (action == "j") {
            pushPacket(state, 250.0);
            state.lastAction = "arrival delayed 250 ms; media sample index unchanged";
        } else if (action == "r") {
            state = InteractiveState{};
        } else {
            state.lastAction = "unknown command";
        }
    }
}

std::uint16_t readU16(std::istream &input) {
    unsigned char b[2]{};
    input.read(reinterpret_cast<char *>(b), 2);
    return static_cast<std::uint16_t>(b[0]) |
           (static_cast<std::uint16_t>(b[1]) << 8);
}

std::uint32_t readU32(std::istream &input) {
    unsigned char b[4]{};
    input.read(reinterpret_cast<char *>(b), 4);
    return static_cast<std::uint32_t>(b[0]) |
           (static_cast<std::uint32_t>(b[1]) << 8) |
           (static_cast<std::uint32_t>(b[2]) << 16) |
           (static_cast<std::uint32_t>(b[3]) << 24);
}

std::vector<std::int16_t> readWave(std::string const &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open WAV");
    char riff[4]{}, wave[4]{};
    input.read(riff, 4);
    (void)readU32(input);
    input.read(wave, 4);
    if (std::memcmp(riff, "RIFF", 4) || std::memcmp(wave, "WAVE", 4))
        throw std::runtime_error("not RIFF/WAVE");

    int format = 0, channels = 0, bits = 0, rate = 0;
    std::vector<std::int16_t> samples;
    while (input) {
        char id[4]{};
        input.read(id, 4);
        if (!input)
            break;
        auto const size = readU32(input);
        auto const payload = input.tellg();
        if (!std::memcmp(id, "fmt ", 4)) {
            format = readU16(input);
            channels = readU16(input);
            rate = static_cast<int>(readU32(input));
            (void)readU32(input);
            (void)readU16(input);
            bits = readU16(input);
        } else if (!std::memcmp(id, "data", 4)) {
            samples.resize(size / 2);
            input.read(reinterpret_cast<char *>(samples.data()), size);
        }
        input.seekg(payload + static_cast<std::streamoff>(size + (size & 1)));
    }
    if (format != 1 || channels != 1 || bits != 16 || rate != 12000)
        throw std::runtime_error("expected mono PCM16/12000");
    return samples;
}

struct DecodeList { std::vector<Js8ReferenceDecode> values; };

void rememberDecode(Js8ReferenceDecode const *value, void *context) {
    static_cast<DecodeList *>(context)->values.push_back(*value);
}

DecodeList decode(std::vector<std::int16_t> const &pcm, int submode) {
    DecodeList result;
    auto const count = js8_reference_decode(pcm.data(), pcm.size(), submode,
                                            0, 5000, 1500,
                                            rememberDecode, &result);
    if (count < 0)
        throw std::runtime_error("reference decoder rejected sample window");
    return result;
}

std::vector<float> resample(std::span<const float> input, int inRate,
                            int outRate) {
    js8proto::SincResampler converter(inRate, outRate);
    auto result = converter.push(input);
    auto tail = converter.finish();
    result.insert(result.end(), tail.begin(), tail.end());
    return result;
}

void fixture(std::string const &path, int submode) {
    auto const original = readWave(path);
    std::vector<float> pcm12k(original.size());
    std::transform(original.begin(), original.end(), pcm12k.begin(),
                   [](auto sample) { return sample / 32768.0f; });
    auto const pcm8k = resample(pcm12k, 12000, 8000);

    std::vector<std::uint8_t> ulaw(pcm8k.size());
    std::transform(pcm8k.begin(), pcm8k.end(), ulaw.begin(),
                   js8proto::encodeUlaw);

    js8proto::AudioTimeline timeline;
    std::vector<float> restored;
    std::uint32_t sequence = 0;
    for (std::size_t offset = 0; offset < ulaw.size(); offset += 160) {
        auto const count = std::min<std::size_t>(160, ulaw.size() - offset);
        auto packet = throughAud1(
            sequence++, offset, offset / 8.0,
            std::span<const std::uint8_t>(ulaw).subspan(offset, count));
        auto result = timeline.ingest(packet);
        restored.insert(restored.end(), result.pcm12k.begin(),
                        result.pcm12k.end());
    }
    auto tail = timeline.finish();
    restored.insert(restored.end(), tail.begin(), tail.end());

    std::vector<std::int16_t> restoredPcm(restored.size());
    std::transform(restored.begin(), restored.end(), restoredPcm.begin(),
                   [](float sample) {
                       return static_cast<std::int16_t>(std::lround(
                           std::clamp(sample, -1.0f, 0.999969f) * 32768.0f));
                   });

    auto const direct = decode(original, submode);
    auto const roundtrip = decode(restoredPcm, submode);
    auto const state = timeline.state();
    std::cout << "PROTOTYPE fixture round-trip\n"
              << "path=" << path << '\n'
              << "samples: 12k-in=" << original.size()
              << " 8k-ulaw=" << ulaw.size()
              << " 12k-out=" << restoredPcm.size() << '\n'
              << "timeline: gaps=" << state.insertedGapSamples8k
              << " duplicates=" << state.duplicatePackets
              << " discontinuities=" << state.discontinuities << '\n'
              << "signal: peak=" << state.peak << " rms=" << state.rms << '\n'
              << "decode: direct=" << direct.values.size()
              << " ulaw-roundtrip=" << roundtrip.values.size() << '\n';
    for (auto const &value : roundtrip.values)
        std::cout << "FRAME mode=" << value.submode << " snr=" << value.snr
                  << " dt=" << value.dt << " freq=" << value.frequency_hz
                  << " data=\"" << value.data << "\"\n";
}

void dumpAud1(std::string const &path, std::string const &outputPath) {
    auto const original = readWave(path);
    std::vector<float> pcm12k(original.size());
    std::transform(original.begin(), original.end(), pcm12k.begin(),
                   [](auto sample) { return sample / 32768.0f; });
    auto const pcm8k = resample(pcm12k, 12000, 8000);
    std::vector<std::uint8_t> ulaw(pcm8k.size());
    std::transform(pcm8k.begin(), pcm8k.end(), ulaw.begin(),
                   js8proto::encodeUlaw);

    std::ofstream output(outputPath, std::ios::binary);
    if (!output)
        throw std::runtime_error("cannot create AUD1 dump");
    std::uint32_t sequence = 0;
    for (std::size_t offset = 0; offset < ulaw.size(); offset += 160) {
        auto const count = std::min<std::size_t>(160, ulaw.size() - offset);
        auto const wire = aud1Wire(
            sequence++, offset,
            std::span<const std::uint8_t>(ulaw).subspan(offset, count));
        auto const length = static_cast<std::uint32_t>(wire.size());
        char prefix[4] = {static_cast<char>(length >> 24),
                          static_cast<char>(length >> 16),
                          static_cast<char>(length >> 8),
                          static_cast<char>(length)};
        output.write(prefix, sizeof(prefix));
        output.write(reinterpret_cast<char const *>(wire.data()), wire.size());
    }
    std::cout << "AUD1 dump packets=" << sequence << " ulaw_samples="
              << ulaw.size() << " path=" << outputPath << '\n';
}
} // namespace

int main(int argc, char **argv) {
    try {
        if (argc == 4 && std::string_view(argv[1]) == "--fixture") {
            fixture(argv[2], std::stoi(argv[3]));
            return 0;
        }
        if (argc == 4 && std::string_view(argv[1]) == "--aud1-dump") {
            dumpAud1(argv[2], argv[3]);
            return 0;
        }
        if (argc != 1) {
            std::cerr << "usage: js8-stream-prototype "
                         "[--fixture WAV SUBMODE | --aud1-dump WAV FILE]\n";
            return 2;
        }
        interactive();
        return 0;
    } catch (std::exception const &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
