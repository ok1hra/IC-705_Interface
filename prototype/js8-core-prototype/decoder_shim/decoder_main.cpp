#include "decoder_c_api.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::uint16_t read_u16(std::istream &input) {
    unsigned char bytes[2]{};
    input.read(reinterpret_cast<char *>(bytes), 2);
    return static_cast<std::uint16_t>(bytes[0]) |
           static_cast<std::uint16_t>(bytes[1] << 8);
}

std::uint32_t read_u32(std::istream &input) {
    unsigned char bytes[4]{};
    input.read(reinterpret_cast<char *>(bytes), 4);
    return static_cast<std::uint32_t>(bytes[0]) |
           (static_cast<std::uint32_t>(bytes[1]) << 8) |
           (static_cast<std::uint32_t>(bytes[2]) << 16) |
           (static_cast<std::uint32_t>(bytes[3]) << 24);
}

struct Wave {
    int sample_rate{};
    std::vector<std::int16_t> samples;
};

Wave read_wave(std::string const &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open WAV: " + path);

    char riff[4]{};
    input.read(riff, 4);
    (void)read_u32(input);
    char wave[4]{};
    input.read(wave, 4);
    if (std::memcmp(riff, "RIFF", 4) || std::memcmp(wave, "WAVE", 4))
        throw std::runtime_error("not a RIFF/WAVE file");

    std::uint16_t format = 0;
    std::uint16_t channels = 0;
    std::uint16_t bits = 0;
    std::uint32_t rate = 0;
    std::vector<std::int16_t> samples;

    while (input) {
        char id[4]{};
        input.read(id, 4);
        if (!input)
            break;
        auto const size = read_u32(input);
        auto const payload = input.tellg();

        if (!std::memcmp(id, "fmt ", 4)) {
            format = read_u16(input);
            channels = read_u16(input);
            rate = read_u32(input);
            (void)read_u32(input);
            (void)read_u16(input);
            bits = read_u16(input);
        } else if (!std::memcmp(id, "data", 4)) {
            if (size % 2)
                throw std::runtime_error("odd PCM payload length");
            samples.resize(size / 2);
            input.read(reinterpret_cast<char *>(samples.data()), size);
        }

        input.seekg(payload + static_cast<std::streamoff>(size + (size & 1)));
    }

    if (format != 1 || channels != 1 || bits != 16 || rate != 12000 ||
        samples.empty())
        throw std::runtime_error("expected mono PCM16 at 12000 Hz");
    return Wave{static_cast<int>(rate), std::move(samples)};
}

void print_decode(Js8ReferenceDecode const *decoded, void *) {
    std::cout << "DECODE mode=" << decoded->submode << " snr=" << decoded->snr
              << " dt=" << decoded->dt << " freq=" << decoded->frequency_hz
              << " type=" << decoded->type << " quality=" << decoded->quality
              << " data=\"" << decoded->data << "\"\n";
}
} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "usage: js8-reference-decoder WAV [submode]\n"
                     "submode: 0=normal, 1=fast, 2=turbo, 4=slow, 8=ultra\n";
        return 2;
    }

    try {
        auto const wave = read_wave(argv[1]);
        auto const submode = argc == 3 ? std::stoi(argv[2]) : 0;
        std::cout << "WAV rate=" << wave.sample_rate
                  << " samples=" << wave.samples.size()
                  << " seconds="
                  << static_cast<double>(wave.samples.size()) / wave.sample_rate
                  << " submode=" << submode << '\n';
        auto const decoded = js8_reference_decode(
            wave.samples.data(), wave.samples.size(), submode, 0, 5000, 1500,
            print_decode, nullptr);
        if (decoded < 0) {
            std::cerr << "decoder rejected input (error " << decoded << ")\n";
            return 1;
        }
        std::cout << "RESULT unique_decodes=" << decoded << '\n';
        return 0;
    } catch (std::exception const &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
