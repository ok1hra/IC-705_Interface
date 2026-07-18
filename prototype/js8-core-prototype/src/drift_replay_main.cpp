// PROTOTYPE — generate continuous multi-slot AUD1 fixtures with frame drift.
// Question: does auto-speed Worker decoding preserve every slot and measured
// offset while one station drifts across consecutive frames?

#include "aud1_protocol.hpp"
#include "audio_frontend.hpp"
#include "decoder_c_api.h"
#include "js8_core.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr std::string_view kRaw = "TrMcT8++++++";
constexpr int kType = 7;
constexpr std::array<double, 5> kDriftTones = {-0.5, -0.25, 0.0, 0.25, 0.5};

struct Captured {
    std::vector<Js8ReferenceDecode> frames;
};

void capture(Js8ReferenceDecode const *decoded, void *context) {
    static_cast<Captured *>(context)->frames.push_back(*decoded);
}

std::optional<double> decodeFrequency(std::span<const std::int16_t> pcm,
                                      int mode) {
    Captured captured;
    auto const count = js8_reference_decode(pcm.data(), pcm.size(), mode,
                                            0, 5000, 1500,
                                            capture, &captured);
    if (count < 0)
        throw std::runtime_error("reference decoder rejected drift frame");
    for (auto const &frame : captured.frames) {
        if (frame.submode == mode && frame.type == kType && frame.data == kRaw)
            return frame.frequency_hz;
    }
    return std::nullopt;
}

char modeLetter(int mode) {
    switch (mode) {
    case 0: return 'A';
    case 1: return 'B';
    case 2: return 'C';
    case 4: return 'E';
    case 8: return 'I';
    default: return 'X';
    }
}

std::vector<std::uint8_t>
toUlaw8k(std::span<const std::int16_t> pcm12k) {
    std::vector<float> input(pcm12k.size());
    std::transform(pcm12k.begin(), pcm12k.end(), input.begin(),
                   [](auto sample) { return sample / 32768.0f; });
    js8proto::SincResampler converter(12000, 8000);
    auto pcm8k = converter.push(input);
    auto tail = converter.finish();
    pcm8k.insert(pcm8k.end(), tail.begin(), tail.end());
    std::vector<std::uint8_t> result(pcm8k.size());
    std::transform(pcm8k.begin(), pcm8k.end(), result.begin(),
                   js8proto::encodeUlaw);
    return result;
}

std::size_t writeDump(std::filesystem::path const &path,
                      std::span<const std::uint8_t> ulaw,
                      std::uint32_t streamId) {
    std::ofstream output(path, std::ios::binary);
    if (!output)
        throw std::runtime_error("cannot create drift AUD1 dump");
    std::uint32_t sequence = 0;
    for (std::size_t offset = 0; offset < ulaw.size(); offset += 160) {
        auto const count = std::min<std::size_t>(160, ulaw.size() - offset);
        std::uint16_t flags = 0;
        if (offset == 0) flags |= js8proto::Aud1First;
        if (offset + count == ulaw.size()) flags |= js8proto::Aud1Last;
        js8proto::Aud1Header header{js8proto::Aud1Kind::RxUlaw,
                                    flags,
                                    streamId,
                                    sequence++,
                                    8000,
                                    offset,
                                    0};
        auto const wire = js8proto::encodeAud1(
            header, ulaw.subspan(offset, count));
        if (wire.empty())
            throw std::runtime_error("AUD1 encoder rejected drift packet");
        auto const length = static_cast<std::uint32_t>(wire.size());
        std::array<char, 4> prefix = {
            static_cast<char>(length >> 24),
            static_cast<char>(length >> 16),
            static_cast<char>(length >> 8),
            static_cast<char>(length)};
        output.write(prefix.data(), prefix.size());
        output.write(reinterpret_cast<char const *>(wire.data()), wire.size());
    }
    return sequence;
}
} // namespace

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: js8-drift-replay OUTPUT_DIR\n";
        return 2;
    }
    try {
        std::filesystem::path const outputDir(argv[1]);
        std::filesystem::create_directories(outputDir);
        std::ofstream manifest(outputDir / "manifest.json");
        if (!manifest)
            throw std::runtime_error("cannot create drift manifest");
        manifest << std::setprecision(10)
                 << "{\n  \"question\": \"does auto-speed AUD1 decoding track consecutive frame drift?\",\n"
                 << "  \"raw\": \"" << kRaw << "\",\n"
                 << "  \"type\": " << kType << ",\n"
                 << "  \"text\": \"KN4CRD: TEST\",\n"
                 << "  \"frameCount\": " << kDriftTones.size() << ",\n"
                 << "  \"vectors\": [\n";

        bool nativePass = true;
        auto const &modes = js8proto::submodes();
        for (std::size_t modeIndex = 0; modeIndex < modes.size(); ++modeIndex) {
            auto const &spec = modes[modeIndex];
            auto const mode = static_cast<int>(spec.id);
            auto encoded = js8proto::encodeFrame(kRaw, kType, spec.id);
            if (!encoded)
                throw std::runtime_error("encoder rejected drift frame");

            std::vector<std::int16_t> stream;
            std::array<double, kDriftTones.size()> nativeFrequencies{};
            std::array<double, kDriftTones.size()> baseFrequencies{};
            for (std::size_t frame = 0; frame < kDriftTones.size(); ++frame) {
                auto const base = 1500.0 +
                    kDriftTones[frame] * spec.toneSpacingHz();
                auto pcm = js8proto::modulateFrame12k(*encoded, base);
                auto frequency = decodeFrequency(pcm, mode);
                auto const exact = frequency.has_value();
                nativePass &= exact;
                baseFrequencies[frame] = base;
                nativeFrequencies[frame] = frequency.value_or(0.0);
                stream.insert(stream.end(), pcm.begin(), pcm.end());
                std::cout << "DRIFT NATIVE mode=" << mode
                          << " frame=" << frame
                          << " base=" << base
                          << " decoded=" << (exact ? "PASS" : "FAIL")
                          << " freq=" << nativeFrequencies[frame] << '\n';
            }

            auto const ulaw = toUlaw8k(stream);
            auto const letter = modeLetter(mode);
            auto const dumpName = std::string(1, letter) + "_drift.aud1.bin";
            auto const packets = writeDump(outputDir / dumpName, ulaw,
                0x44524600U + static_cast<std::uint32_t>(mode));
            manifest << "    {\"submode\":" << mode
                     << ",\"letter\":\"" << letter
                     << "\",\"periodMs\":" << spec.periodMs
                     << ",\"toneSpacingHz\":" << spec.toneSpacingHz()
                     << ",\"samples12k\":" << stream.size()
                     << ",\"samples8k\":" << ulaw.size()
                     << ",\"packets\":" << packets
                     << ",\"dump\":\"" << dumpName << "\",\"frames\":[";
            for (std::size_t frame = 0; frame < kDriftTones.size(); ++frame) {
                manifest << "{\"slotUtcMs\":" << frame * spec.periodMs
                         << ",\"driftTones\":" << kDriftTones[frame]
                         << ",\"baseHz\":" << baseFrequencies[frame]
                         << ",\"nativeFrequencyHz\":"
                         << nativeFrequencies[frame] << "}"
                         << (frame + 1 == kDriftTones.size() ? "" : ",");
            }
            manifest << "]}" << (modeIndex + 1 == modes.size() ? "\n" : ",\n");
        }
        manifest << "  ],\n  \"nativePass\": "
                 << (nativePass ? "true" : "false") << "\n}\n";
        std::cout << "DRIFT FIXTURES " << (nativePass ? "PASS" : "FAIL")
                  << '\n';
        return nativePass ? 0 : 1;
    } catch (std::exception const &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
