// PROTOTYPE — binary audio WebSocket envelope candidate.

#include "aud1_protocol.hpp"

#include <algorithm>
#include <array>
#include <limits>

namespace js8proto {
namespace {
constexpr std::array<std::uint8_t, 4> kMagic = {'A', 'U', 'D', '1'};
constexpr std::uint16_t kKnownFlags =
    Aud1First | Aud1Last | Aud1Discontinuity | Aud1Abort;

void put16(std::vector<std::uint8_t> &out, std::size_t at,
           std::uint16_t value) {
    out[at] = static_cast<std::uint8_t>(value >> 8);
    out[at + 1] = static_cast<std::uint8_t>(value);
}

void put32(std::vector<std::uint8_t> &out, std::size_t at,
           std::uint32_t value) {
    for (int byte = 0; byte < 4; ++byte)
        out[at + byte] = static_cast<std::uint8_t>(value >> (24 - byte * 8));
}

void put64(std::vector<std::uint8_t> &out, std::size_t at,
           std::uint64_t value) {
    for (int byte = 0; byte < 8; ++byte)
        out[at + byte] = static_cast<std::uint8_t>(value >> (56 - byte * 8));
}

std::uint16_t get16(std::span<const std::uint8_t> input, std::size_t at) {
    return static_cast<std::uint16_t>((input[at] << 8) | input[at + 1]);
}

std::uint32_t get32(std::span<const std::uint8_t> input, std::size_t at) {
    std::uint32_t value = 0;
    for (int byte = 0; byte < 4; ++byte)
        value = (value << 8) | input[at + byte];
    return value;
}

std::uint64_t get64(std::span<const std::uint8_t> input, std::size_t at) {
    std::uint64_t value = 0;
    for (int byte = 0; byte < 8; ++byte)
        value = (value << 8) | input[at + byte];
    return value;
}

bool validKind(Aud1Kind kind) {
    return kind == Aud1Kind::RxUlaw || kind == Aud1Kind::RxPcm16 ||
           kind == Aud1Kind::TxPcm16;
}
} // namespace

std::vector<std::uint8_t>
encodeAud1(Aud1Header const &header, std::span<const std::uint8_t> payload) {
    if (!validKind(header.kind) || header.sampleRate == 0 ||
        payload.size() > std::numeric_limits<std::uint32_t>::max())
        return {};

    std::vector<std::uint8_t> wire(kAud1HeaderBytes + payload.size());
    std::copy(kMagic.begin(), kMagic.end(), wire.begin());
    wire[4] = kAud1Version;
    wire[5] = static_cast<std::uint8_t>(header.kind);
    put16(wire, 6, header.flags);
    put16(wire, 8, kAud1HeaderBytes);
    put16(wire, 10, 0);
    put32(wire, 12, header.streamId);
    put32(wire, 16, header.sequence);
    put32(wire, 20, header.sampleRate);
    put64(wire, 24, header.firstSample);
    put32(wire, 32, header.txId);
    put32(wire, 36, static_cast<std::uint32_t>(payload.size()));
    std::copy(payload.begin(), payload.end(), wire.begin() + kAud1HeaderBytes);
    return wire;
}

std::optional<Aud1Message> decodeAud1(std::span<const std::uint8_t> wire) {
    if (wire.size() < kAud1HeaderBytes ||
        !std::equal(kMagic.begin(), kMagic.end(), wire.begin()) ||
        wire[4] != kAud1Version || get16(wire, 8) != kAud1HeaderBytes ||
        get16(wire, 10) != 0)
        return std::nullopt;

    Aud1Header header{static_cast<Aud1Kind>(wire[5]),
                      get16(wire, 6),
                      get32(wire, 12),
                      get32(wire, 16),
                      get32(wire, 20),
                      get64(wire, 24),
                      get32(wire, 32)};
    auto const payloadBytes = get32(wire, 36);
    if (!validKind(header.kind) || header.sampleRate == 0 ||
        (header.flags & ~kKnownFlags) != 0 ||
        wire.size() != kAud1HeaderBytes + payloadBytes ||
        ((header.kind == Aud1Kind::RxPcm16 ||
          header.kind == Aud1Kind::TxPcm16) &&
         payloadBytes % 2 != 0))
        return std::nullopt;

    return Aud1Message{header,
                       std::vector<std::uint8_t>(
                           wire.begin() + kAud1HeaderBytes, wire.end())};
}

} // namespace js8proto
