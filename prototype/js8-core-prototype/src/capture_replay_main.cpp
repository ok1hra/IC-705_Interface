// PROTOTYPE — turn upstream reference WAV captures into replayable AUD1 data.
// Question: does the real uLaw/Worker path retain the pinned native decoder
// yield for each capture and for one continuous multi-slot sequence?

#include "aud1_protocol.hpp"
#include "audio_frontend.hpp"
#include "decoder_c_api.h"
#include "js8_core.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::uint16_t readU16(std::istream &input) {
    unsigned char bytes[2]{};
    input.read(reinterpret_cast<char *>(bytes), 2);
    return static_cast<std::uint16_t>(bytes[0]) |
           static_cast<std::uint16_t>(bytes[1] << 8);
}

std::uint32_t readU32(std::istream &input) {
    unsigned char bytes[4]{};
    input.read(reinterpret_cast<char *>(bytes), 4);
    return static_cast<std::uint32_t>(bytes[0]) |
           (static_cast<std::uint32_t>(bytes[1]) << 8) |
           (static_cast<std::uint32_t>(bytes[2]) << 16) |
           (static_cast<std::uint32_t>(bytes[3]) << 24);
}

std::vector<std::int16_t> readWave(std::filesystem::path const &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open WAV: " + path.string());
    char riff[4]{}, wave[4]{};
    input.read(riff, 4); (void)readU32(input); input.read(wave, 4);
    if (std::memcmp(riff, "RIFF", 4) || std::memcmp(wave, "WAVE", 4))
        throw std::runtime_error("not a RIFF/WAVE file: " + path.string());
    int format = 0, channels = 0, bits = 0, rate = 0;
    std::vector<std::int16_t> samples;
    while (input) {
        char id[4]{};
        input.read(id, 4);
        if (!input) break;
        auto const size = readU32(input);
        auto const payload = input.tellg();
        if (!std::memcmp(id, "fmt ", 4)) {
            format = readU16(input); channels = readU16(input);
            rate = static_cast<int>(readU32(input));
            (void)readU32(input); (void)readU16(input); bits = readU16(input);
        } else if (!std::memcmp(id, "data", 4)) {
            if (size % 2) throw std::runtime_error("odd WAV PCM payload");
            samples.resize(size / 2);
            input.read(reinterpret_cast<char *>(samples.data()), size);
        }
        input.seekg(payload + static_cast<std::streamoff>(size + (size & 1)));
    }
    if (format != 1 || channels != 1 || bits != 16 || rate != 12000 ||
        samples.empty())
        throw std::runtime_error("expected mono PCM16/12000: " + path.string());
    return samples;
}

struct Capture {
    std::filesystem::path path;
    int mode{};
    int upstreamExpected{};
    std::vector<std::int16_t> pcm;
    std::vector<Js8ReferenceDecode> frames;
    std::string dump;
    std::uint64_t startUtcMs{};
};

void remember(Js8ReferenceDecode const *frame, void *context) {
    static_cast<std::vector<Js8ReferenceDecode> *>(context)->push_back(*frame);
}

int upstreamExpected(std::filesystem::path const &path) {
    auto const stem = path.stem().string();
    auto const split = stem.rfind('_');
    if (split == std::string::npos) return -1;
    auto const suffix = stem.substr(split + 1);
    try {
        std::size_t consumed = 0;
        auto const value = std::stoi(suffix, &consumed);
        return consumed == suffix.size() ? value : -1;
    } catch (std::exception const &) {
        return -1;
    }
}

std::string json(std::string_view input) {
    std::string output;
    for (auto const value : input) {
        if (value == '\\' || value == '"') output.push_back('\\');
        output.push_back(value);
    }
    return output;
}

std::vector<std::uint8_t> toUlaw8k(std::span<const std::int16_t> pcm) {
    std::vector<float> input(pcm.size());
    std::transform(pcm.begin(), pcm.end(), input.begin(),
                   [](auto sample) { return sample / 32768.0f; });
    js8proto::SincResampler converter(12000, 8000);
    auto output = converter.push(input);
    auto tail = converter.finish();
    output.insert(output.end(), tail.begin(), tail.end());
    std::vector<std::uint8_t> ulaw(output.size());
    std::transform(output.begin(), output.end(), ulaw.begin(),
                   js8proto::encodeUlaw);
    return ulaw;
}

std::size_t writeDump(std::filesystem::path const &path,
                      std::span<const std::uint8_t> ulaw,
                      std::uint32_t streamId) {
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot create AUD1 capture dump");
    std::uint32_t sequence = 0;
    for (std::size_t offset = 0; offset < ulaw.size(); offset += 160) {
        auto const count = std::min<std::size_t>(160, ulaw.size() - offset);
        std::uint16_t flags = 0;
        if (offset == 0) flags |= js8proto::Aud1First;
        if (offset + count == ulaw.size()) flags |= js8proto::Aud1Last;
        auto const wire = js8proto::encodeAud1(
            {js8proto::Aud1Kind::RxUlaw, flags, streamId, sequence++, 8000,
             offset, 0}, ulaw.subspan(offset, count));
        auto const length = static_cast<std::uint32_t>(wire.size());
        char prefix[4] = {static_cast<char>(length >> 24),
                          static_cast<char>(length >> 16),
                          static_cast<char>(length >> 8),
                          static_cast<char>(length)};
        output.write(prefix, sizeof(prefix));
        output.write(reinterpret_cast<char const *>(wire.data()), wire.size());
    }
    return sequence;
}

void writeFrames(std::ostream &output,
                 std::vector<Js8ReferenceDecode> const &frames) {
    output << '[';
    for (std::size_t index = 0; index < frames.size(); ++index) {
        auto const &frame = frames[index];
        output << "{\"raw\":\"" << json(frame.data)
               << "\",\"frameType\":" << frame.type
               << ",\"snr\":" << frame.snr
               << ",\"dtMs\":" << frame.dt * 1000.0
               << ",\"frequencyHz\":" << frame.frequency_hz << '}';
        if (index + 1 != frames.size()) output << ',';
    }
    output << ']';
}
} // namespace

int main(int argc, char **argv) {
    if (argc < 4 || (argc - 2) % 2 != 0) {
        std::cerr << "usage: js8-capture-replay OUTPUT_DIR WAV MODE [WAV MODE]\n";
        return 2;
    }
    try {
        std::filesystem::path const outputDir(argv[1]);
        std::filesystem::create_directories(outputDir);
        std::vector<Capture> captures;
        bool nativePass = true;
        for (int argument = 2; argument < argc; argument += 2) {
            Capture capture;
            capture.path = argv[argument];
            capture.mode = std::stoi(argv[argument + 1]);
            capture.upstreamExpected = upstreamExpected(capture.path);
            capture.pcm = readWave(capture.path);
            auto const decoded = js8_reference_decode(
                capture.pcm.data(), capture.pcm.size(), capture.mode,
                0, 5000, 1500, remember, &capture.frames);
            nativePass &= decoded >= 0;
            capture.dump = capture.path.stem().string() + ".aud1.bin";
            auto const ulaw = toUlaw8k(capture.pcm);
            writeDump(outputDir / capture.dump, ulaw,
                      0x43500000U + static_cast<std::uint32_t>(captures.size()));
            std::cout << "CAPTURE NATIVE file=" << capture.path.filename().string()
                      << " mode=" << capture.mode
                      << " upstream=" << capture.upstreamExpected
                      << " pinned=" << capture.frames.size()
                      << (capture.upstreamExpected ==
                          static_cast<int>(capture.frames.size())
                              ? " MATCH" : " DIFFER") << '\n';
            captures.push_back(std::move(capture));
        }

        std::vector<std::int16_t> longPcm;
        for (auto &capture : captures) {
            auto const *spec = js8proto::findSubmode(capture.mode);
            if (!spec) throw std::runtime_error("unknown capture submode");
            auto const currentMs = longPcm.size() / 12;
            auto const alignedMs =
                ((currentMs + spec->periodMs - 1) / spec->periodMs) *
                spec->periodMs;
            longPcm.resize(alignedMs * 12, 0);
            capture.startUtcMs = alignedMs;
            longPcm.insert(longPcm.end(), capture.pcm.begin(), capture.pcm.end());
        }
        auto const longUlaw = toUlaw8k(longPcm);
        auto const longDump = std::string("long_reference_sequence.aud1.bin");
        auto const longPackets = writeDump(outputDir / longDump, longUlaw,
                                           0x43504c4fU);

        std::ofstream manifest(outputDir / "manifest.json");
        if (!manifest) throw std::runtime_error("cannot create capture manifest");
        manifest << "{\n  \"question\": \"does uLaw/Worker replay retain pinned native yield for upstream WAV captures?\",\n"
                 << "  \"sourceClass\": \"reference-wav-manifest\",\n"
                 << "  \"missingReferenceSubmodes\": [";
        bool firstMissing = true;
        for (auto const mode : {0, 1, 2, 4, 8}) {
            auto const found = std::any_of(captures.begin(), captures.end(),
                [mode](auto const &capture) { return capture.mode == mode; });
            if (!found) {
                if (!firstMissing) manifest << ',';
                manifest << mode;
                firstMissing = false;
            }
        }
        manifest << "],\n"
                 << "  \"vectors\": [\n";
        int nativeTotal = 0;
        int upstreamTotal = 0;
        for (std::size_t index = 0; index < captures.size(); ++index) {
            auto const &capture = captures[index];
            nativeTotal += capture.frames.size();
            if (capture.upstreamExpected >= 0)
                upstreamTotal += capture.upstreamExpected;
            manifest << "    {\"id\":\"" << json(capture.path.stem().string())
                     << "\",\"source\":\""
                     << json(capture.path.generic_string())
                     << "\",\"submode\":" << capture.mode
                     << ",\"upstreamExpected\":" << capture.upstreamExpected
                     << ",\"nativeCount\":" << capture.frames.size()
                     << ",\"nativeMatchesUpstream\":"
                     << (capture.upstreamExpected ==
                         static_cast<int>(capture.frames.size()) ? "true" : "false")
                     << ",\"samples12k\":" << capture.pcm.size()
                     << ",\"startUtcMs\":" << capture.startUtcMs
                     << ",\"dump\":\"" << capture.dump << "\",\"frames\":";
            writeFrames(manifest, capture.frames);
            manifest << '}' << (index + 1 == captures.size() ? "\n" : ",\n");
        }
        manifest << "  ],\n  \"longSequence\": {\"dump\":\"" << longDump
                 << "\",\"samples12k\":" << longPcm.size()
                 << ",\"packets\":" << longPackets
                 << ",\"audioSeconds\":" << longPcm.size() / 12000.0
                 << "},\n  \"nativeTotal\":" << nativeTotal
                 << ",\n  \"upstreamExpectedTotal\":" << upstreamTotal
                 << ",\n  \"nativePass\":" << (nativePass ? "true" : "false")
                 << "\n}\n";
        std::cout << "CAPTURE FIXTURES " << (nativePass ? "PASS" : "FAIL")
                  << " native_total=" << nativeTotal
                  << " upstream_total=" << upstreamTotal
                  << " long_seconds=" << longPcm.size() / 12000.0 << '\n';
        return nativePass ? 0 : 1;
    } catch (std::exception const &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
