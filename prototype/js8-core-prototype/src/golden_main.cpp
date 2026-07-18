// PROTOTYPE — deterministic all-speed encoder/modulator/decoder golden probe.

#include "decoder_c_api.h"
#include "js8_core.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr std::string_view kRaw = "TrMcT8++++++";
constexpr int kType = 7;

void writeU16(std::ostream& output, std::uint16_t value) {
    char bytes[2] = {static_cast<char>(value), static_cast<char>(value >> 8)};
    output.write(bytes, sizeof(bytes));
}

void writeU32(std::ostream& output, std::uint32_t value) {
    char bytes[4] = {static_cast<char>(value), static_cast<char>(value >> 8),
                     static_cast<char>(value >> 16), static_cast<char>(value >> 24)};
    output.write(bytes, sizeof(bytes));
}

void writeWave(std::filesystem::path const& path,
               std::vector<std::int16_t> const& pcm) {
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot create golden WAV");
    auto const dataBytes = static_cast<std::uint32_t>(pcm.size() * 2);
    output.write("RIFF", 4); writeU32(output, 36 + dataBytes);
    output.write("WAVEfmt ", 8); writeU32(output, 16); writeU16(output, 1);
    writeU16(output, 1); writeU32(output, 12000); writeU32(output, 24000);
    writeU16(output, 2); writeU16(output, 16);
    output.write("data", 4); writeU32(output, dataBytes);
    output.write(reinterpret_cast<char const*>(pcm.data()), dataBytes);
}

struct Captured { std::vector<Js8ReferenceDecode> frames; };

void capture(Js8ReferenceDecode const* decoded, void* context) {
    static_cast<Captured*>(context)->frames.push_back(*decoded);
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
} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: js8-golden-fixtures OUTPUT_DIR\n";
        return 2;
    }
    try {
        std::filesystem::path const outputDir(argv[1]);
        std::filesystem::create_directories(outputDir);
        std::ofstream manifest(outputDir / "manifest.json");
        if (!manifest) throw std::runtime_error("cannot create golden manifest");
        manifest << "{\n  \"question\": \"does one protocol frame round-trip at every JS8 speed?\",\n"
                 << "  \"raw\": \"" << kRaw << "\",\n  \"type\": " << kType
                 << ",\n  \"text\": \"KN4CRD: TEST\",\n  \"vectors\": [\n";

        bool allPassed = true;
        auto const& modes = js8proto::submodes();
        for (std::size_t index = 0; index < modes.size(); ++index) {
            auto const& mode = modes[index];
            auto encoded = js8proto::encodeFrame(kRaw, kType, mode.id);
            if (!encoded) throw std::runtime_error("encoder rejected golden frame");
            auto pcm = js8proto::modulateFrame12k(*encoded);
            if (pcm.empty()) throw std::runtime_error("modulator rejected golden frame");
            auto const filename = std::string(1, modeLetter(static_cast<int>(mode.id))) +
                                  "_golden_KN4CRD_TEST.wav";
            writeWave(outputDir / filename, pcm);
            Captured captured;
            auto const count = js8_reference_decode(
                pcm.data(), pcm.size(), static_cast<int>(mode.id), 0, 5000, 1500,
                capture, &captured);
            bool matched = false;
            for (auto const& decoded : captured.frames)
                matched |= decoded.submode == static_cast<int>(mode.id) &&
                           decoded.type == kType && decoded.data == kRaw;
            allPassed &= count >= 0 && matched;
            std::cout << "GOLDEN mode=" << static_cast<int>(mode.id)
                      << " letter=" << modeLetter(static_cast<int>(mode.id))
                      << " samples=" << pcm.size() << " decodes=" << count
                      << " exact=" << (matched ? "PASS" : "FAIL") << '\n';
            manifest << "    {\"submode\": " << static_cast<int>(mode.id)
                     << ", \"letter\": \"" << modeLetter(static_cast<int>(mode.id))
                     << "\", \"wav\": \"" << filename << "\", \"samples\": "
                     << pcm.size() << ", \"exact\": " << (matched ? "true" : "false")
                     << "}" << (index + 1 == modes.size() ? "\n" : ",\n");
        }
        manifest << "  ]\n}\n";
        std::cout << "GOLDEN RESULT " << (allPassed ? "PASS" : "FAIL") << '\n';
        return allPassed ? 0 : 1;
    } catch (std::exception const& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
