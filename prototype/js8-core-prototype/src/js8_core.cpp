// PROTOTYPE — isolated JS8 transport encoder.
//
// The Costas arrays, alphabet, CRC parameters, parity matrix and encoder layout
// are derived from JS8Call-improved JS8_Mode/JS8.{h,cpp}, GPLv3.

#include "js8_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace js8proto {
namespace {

constexpr std::string_view kAlphabet =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-+";

constexpr std::array<std::array<std::array<int, 7>, 3>, 2> kCostas = {{
    {{{4, 2, 5, 6, 1, 3, 0},
      {4, 2, 5, 6, 1, 3, 0},
      {4, 2, 5, 6, 1, 3, 0}}},
    {{{0, 6, 2, 3, 5, 4, 1},
      {1, 5, 0, 2, 3, 6, 4},
      {2, 5, 0, 6, 4, 1, 3}}},
}};

constexpr std::array<SubmodeSpec, 5> kSubmodes = {{
    {Submode::Slow, "SLOW", 8, 30000, 3840, 500, 10, Costas::Modified},
    {Submode::Normal, "NORMAL", 1, 15000, 1920, 500, 10, Costas::Original},
    {Submode::Fast, "FAST", 2, 10000, 1200, 200, 16, Costas::Modified},
    {Submode::Js840, "JS8 40", 4, 6000, 600, 100, 32, Costas::Modified},
    {Submode::Js860, "JS8 60", 16, 4000, 384, 100, 50, Costas::Modified},
}};

constexpr std::array<std::string_view, 87> kParityRows = {{
    "23bba830e23b6b6f50982e", "1f8e55da218c5df3309052",
    "ca7b3217cd92bd59a5ae20", "56f78313537d0f4382964e",
    "6be396b5e2e819e373340c", "293548a138858328af4210",
    "cb6c6afcdc28bb3f7c6e86", "3f2a86f5c5bd225c961150",
    "849dd2d63673481860f62c", "56cdaec6e7ae14b43feeee",
    "04ef5cfa3766ba778f45a4", "c525ae4bd4f627320a3974",
    "41fd9520b2e4abeb2f989c", "7fb36c24085a34d8c1dbc4",
    "40fc3e44bb7d2bb2756e44", "d38ab0a1d2e52a8ec3bc76",
    "3d0f929ef3949bd84d4734", "45d3814f504064f80549ae",
    "f14dbf263825d0bd04b05e", "db714f8f64e8ac7af1a76e",
    "8d0274de71e7c1a8055eb0", "51f81573dd4049b082de14",
    "d8f937f31822e57c562370", "b6537f417e61d1a7085336",
    "ecbd7c73b9cd34c3720c8a", "3d188ea477f6fa41317a4e",
    "1ac4672b549cd6dba79bcc", "a377253773ea678367c3f6",
    "0dbd816fba1543f721dc72", "ca4186dd44c3121565cf5c",
    "29c29dba9c545e267762fe", "1616d78018d0b4745ca0f2",
    "fe37802941d66dde02b99c", "a9fa8e50bcb032c85e3304",
    "83f640f1a48a8ebc0443ea", "3776af54ccfbae916afde6",
    "a8fc906976c35669e79ce0", "f08a91fb2e1f78290619a8",
    "cc9da55fe046d0cb3a770c", "d36d662a69ae24b74dcbd8",
    "40907b01280f03c0323946", "d037db825175d851f3af00",
    "1bf1490607c54032660ede", "0af7723161ec223080be86",
    "eca9afa0f6b01d92305edc", "7a8dec79a51e8ac5388022",
    "9059dfa2bb20ef7ef73ad4", "6abb212d9739dfc02580f2",
    "f6ad4824b87c80ebfce466", "d747bfc5fd65ef70fbd9bc",
    "612f63acc025b6ab476f7c", "05209a0abb530b9e7e34b0",
    "45b7ab6242b77474d9f11a", "6c280d2a0523d9c4bc5946",
    "f1627701a2d692fd9449e6", "8d9071b7e7a6a2eed6965e",
    "bf4f56e073271f6ab4bf80", "c0fc3ec4fb7d2bb2756644",
    "57da6d13cb96a7689b2790", "a9fa2eefa6f8796a355772",
    "164cc861bdd803c547f2ac", "cc6de59755420925f90ed2",
    "a0c0033a52ab6299802fd2", "b274db8abd3c6f396ea356",
    "97d4169cb33e7435718d90", "81cfc6f18c35b1e1f17114",
    "481a2a0df8a23583f82d6c", "081c29a10d468ccdbcecb6",
    "2c4142bf42b01e71076acc", "a6573f3dc8b16c9d19f746",
    "c87af9a5d5206abca532a8", "012dee2198eba82b19a1da",
    "b1ca4ea2e3d173bad4379c", "b33ec97be83ce413f9acc8",
    "5b0f7742bca86b8012609a", "37d8e0af9258b9e8c5f9b2",
    "35ad3fb0faeb5f1b0c30dc", "6114e08483043fd3f38a8a",
    "cd921fdf59e882683763f6", "95e45ecd0135aca9d6e6ae",
    "2e547dd7a05f6597aac516", "14cd0f642fc0c5fe3a65ca",
    "3a0a1dfd7eee29c2e827e0", "c8b5dffc335095dcdcaf2a",
    "3dd01a59d86310743ec752", "8abdb889efbe39a510a118",
    "3f231f212055371cf3e2a2",
}};

constexpr int hexValue(char c) {
    return c >= '0' && c <= '9' ? c - '0'
         : c >= 'a' && c <= 'f' ? c - 'a' + 10
         : c >= 'A' && c <= 'F' ? c - 'A' + 10
                                 : -1;
}

bool parityBit(std::size_t row, std::size_t col) {
    const auto encoded = kParityRows[row];
    const auto nibble = hexValue(encoded[col / 4]);
    const auto mask = 1 << (3 - static_cast<int>(col % 4));
    return (nibble & mask) != 0;
}

int alphabetWord(char value) {
    const auto pos = kAlphabet.find(value);
    return pos == std::string_view::npos ? -1 : static_cast<int>(pos);
}

std::int64_t positiveMod(std::int64_t value, std::int64_t modulus) {
    const auto result = value % modulus;
    return result < 0 ? result + modulus : result;
}

} // namespace

double SubmodeSpec::toneSpacingHz() const {
    return static_cast<double>(kRxSampleRate) / samplesPerSymbol12k;
}

double SubmodeSpec::bandwidthHz() const { return 8.0 * toneSpacingHz(); }

double SubmodeSpec::symbolDurationMs() const {
    return 79.0 * samplesPerSymbol12k * 1000.0 / kRxSampleRate;
}

const std::array<SubmodeSpec, 5>& submodes() { return kSubmodes; }

const SubmodeSpec* findSubmode(int id) {
    const auto found = std::find_if(kSubmodes.begin(), kSubmodes.end(),
                                    [id](const auto& mode) {
                                        return static_cast<int>(mode.id) == id;
                                    });
    return found == kSubmodes.end() ? nullptr : &*found;
}

SlotPlan planNextSlot(std::int64_t nowUtcMs, const SubmodeSpec& mode,
                      int clockCorrectionMs, int minimumLeadMs) {
    const auto earliest = nowUtcMs + std::max(0, minimumLeadMs);
    const auto corrected = earliest + clockCorrectionMs;
    const auto remainder = positiveMod(corrected, mode.periodMs);
    const auto correctedSlot = remainder == 0
        ? corrected
        : corrected + mode.periodMs - remainder;
    const auto slot = correctedSlot - clockCorrectionMs;
    return {nowUtcMs, slot, slot - nowUtcMs, clockCorrectionMs};
}

bool validToneOffset(double offsetHz, double rxLowHz, double rxHighHz,
                     const SubmodeSpec& mode) {
    return std::isfinite(offsetHz) && offsetHz >= rxLowHz &&
           offsetHz + mode.bandwidthHz() <= rxHighHz;
}

std::uint16_t crc12(const std::array<std::uint8_t, 11>& bytes) {
    constexpr std::uint16_t widthMask = 0x0fff;
    constexpr std::uint16_t highBit = 0x0800;
    constexpr std::uint16_t polynomial = 0x0c06;
    std::uint16_t remainder = 0;

    for (const auto byte : bytes) {
        for (int bit = 7; bit >= 0; --bit) {
            const bool quotient = (remainder & highBit) != 0;
            remainder = static_cast<std::uint16_t>(
                ((remainder << 1) | ((byte >> bit) & 1U)) & widthMask);
            if (quotient) remainder ^= polynomial;
        }
    }
    return static_cast<std::uint16_t>(remainder ^ 42U);
}

std::optional<EncodedFrame> encodeFrame(std::string_view frame, int type,
                                        Submode mode) {
    if (frame.size() != kFrameChars || type < 0 || type > 7) return std::nullopt;
    const auto* spec = findSubmode(static_cast<int>(mode));
    if (!spec) return std::nullopt;

    std::array<std::uint8_t, 11> bytes{};
    for (int i = 0, j = 0; i < 12; i += 4, j += 3) {
        const int a = alphabetWord(frame[i]);
        const int b = alphabetWord(frame[i + 1]);
        const int c = alphabetWord(frame[i + 2]);
        const int d = alphabetWord(frame[i + 3]);
        if (a < 0 || b < 0 || c < 0 || d < 0) return std::nullopt;
        const std::uint32_t words = static_cast<std::uint32_t>(
            (a << 18) | (b << 12) | (c << 6) | d);
        bytes[j] = static_cast<std::uint8_t>(words >> 16);
        bytes[j + 1] = static_cast<std::uint8_t>(words >> 8);
        bytes[j + 2] = static_cast<std::uint8_t>(words);
    }

    bytes[9] = static_cast<std::uint8_t>((type & 7) << 5);
    const auto checksum = crc12(bytes);
    bytes[9] |= static_cast<std::uint8_t>((checksum >> 7) & 0x1f);
    bytes[10] = static_cast<std::uint8_t>((checksum & 0x7f) << 1);

    EncodedFrame result{};
    result.crc12 = checksum;
    result.type = type;
    result.submode = mode;

    const auto costasIndex = spec->costas == Costas::Original ? 0U : 1U;
    std::size_t costasOffset = 0;
    for (const auto& block : kCostas[costasIndex]) {
        std::copy(block.begin(), block.end(), result.tones.begin() + costasOffset);
        costasOffset += 36;
    }

    std::size_t outputBits = 0;
    std::size_t outputByte = 0;
    std::uint8_t outputMask = 0x80;
    std::uint8_t outputWord = 0;
    std::uint8_t parityWord = 0;
    std::size_t parityOffset = 7;
    std::size_t dataOffset = 43;

    for (std::size_t i = 0; i < 87; ++i) {
        std::size_t parityBits = 0;
        std::size_t parityByte = 0;
        std::uint8_t parityMask = 0x80;
        for (std::size_t j = 0; j < 87; ++j) {
            parityBits += parityBit(i, j) && (bytes[parityByte] & parityMask);
            parityMask = parityMask == 1
                ? (++parityByte, static_cast<std::uint8_t>(0x80))
                : static_cast<std::uint8_t>(parityMask >> 1);
        }

        parityWord = static_cast<std::uint8_t>((parityWord << 1) | (parityBits & 1));
        outputWord = static_cast<std::uint8_t>(
            (outputWord << 1) | ((bytes[outputByte] & outputMask) != 0));
        outputMask = outputMask == 1
            ? (++outputByte, static_cast<std::uint8_t>(0x80))
            : static_cast<std::uint8_t>(outputMask >> 1);

        if (++outputBits == 3) {
            result.tones[parityOffset++] = parityWord;
            result.tones[dataOffset++] = outputWord;
            outputBits = 0;
            parityWord = 0;
            outputWord = 0;
        }
    }

    return result;
}

namespace {
std::vector<std::int16_t> modulateFrameAtRate(const EncodedFrame& frame,
                                              int sampleRate,
                                              bool padToPeriod,
                                              double baseFrequencyHz,
                                              double amplitude) {
    auto const* mode = findSubmode(static_cast<int>(frame.submode));
    if (!mode || !std::isfinite(baseFrequencyHz) ||
        !std::isfinite(amplitude) || amplitude <= 0.0 || amplitude > 1.0)
        return {};

    auto const periodSamples = static_cast<std::size_t>(
        static_cast<std::int64_t>(mode->periodMs) * sampleRate / 1000);
    auto const delaySamples = static_cast<std::size_t>(
        static_cast<std::int64_t>(mode->startDelayMs) * sampleRate / 1000);
    auto const samplesPerSymbol = mode->samplesPerSymbol12k * sampleRate / 12000;
    auto const signalSamples = static_cast<std::size_t>(
        samplesPerSymbol) * frame.tones.size();
    if (delaySamples + signalSamples > periodSamples)
        return {};

    std::vector<std::int16_t> pcm(
        padToPeriod ? periodSamples : delaySamples + signalSamples, 0);
    constexpr double pi = 3.141592653589793238462643383279502884;
    constexpr double tau = 2.0 * pi;
    auto const rampSamples = static_cast<std::size_t>(sampleRate / 200); // 5 ms.
    double phase = 0.0;
    std::size_t signalIndex = 0;
    for (auto const tone : frame.tones) {
        auto const frequency = baseFrequencyHz + tone * mode->toneSpacingHz();
        auto const phaseStep = tau * frequency / sampleRate;
        for (int i = 0; i < samplesPerSymbol; ++i, ++signalIndex) {
            auto envelope = 1.0;
            if (signalIndex < rampSamples)
                envelope = 0.5 - 0.5 * std::cos(
                    pi * signalIndex / rampSamples);
            auto const remaining = signalSamples - signalIndex - 1;
            if (remaining < rampSamples)
                envelope *= 0.5 - 0.5 * std::cos(
                    pi * remaining / rampSamples);
            phase = std::fmod(phase + phaseStep, tau);
            auto const sample = std::clamp(amplitude * envelope * std::sin(phase),
                                           -1.0, 0.999969482421875);
            pcm[delaySamples + signalIndex] = static_cast<std::int16_t>(
                std::lround(sample * 32768.0));
        }
    }
    return pcm;
}
} // namespace

std::vector<std::int16_t> modulateFrame12k(const EncodedFrame& frame,
                                           double baseFrequencyHz,
                                           double amplitude) {
    return modulateFrameAtRate(frame, 12000, true, baseFrequencyHz, amplitude);
}

std::vector<std::int16_t> modulateFrame48k(const EncodedFrame& frame,
                                           double baseFrequencyHz,
                                           double amplitude) {
    return modulateFrameAtRate(frame, 48000, false, baseFrequencyHz, amplitude);
}

} // namespace js8proto

extern "C" int js8_proto_get_submode(int id, Js8ProtoSubmodeSpec* out) {
    if (!out) return 0;
    const auto* mode = js8proto::findSubmode(id);
    if (!mode) return 0;
    *out = {static_cast<int>(mode->id), mode->decoderMask, mode->periodMs,
            mode->samplesPerSymbol12k, mode->startDelayMs,
            mode->rxThresholdHz, mode->toneSpacingHz(), mode->bandwidthHz()};
    return 1;
}

extern "C" int js8_proto_encode_frame(const char* frame12, int type,
                                        int submode, std::uint8_t* tones79,
                                        std::uint16_t* checksum) {
    if (!frame12 || !tones79) return 0;
    const auto encoded = js8proto::encodeFrame(
        std::string_view(frame12, js8proto::kFrameChars), type,
        static_cast<js8proto::Submode>(submode));
    if (!encoded) return 0;
    std::copy(encoded->tones.begin(), encoded->tones.end(), tones79);
    if (checksum) *checksum = encoded->crc12;
    return 1;
}
