// PROTOTYPE — binary audio WebSocket envelope candidate.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace js8proto {

inline constexpr std::size_t kAud1HeaderBytes = 40;
inline constexpr std::uint8_t kAud1Version = 1;

enum class Aud1Kind : std::uint8_t {
    RxUlaw = 1,
    RxPcm16 = 2,
    TxPcm16 = 3,
};

enum Aud1Flag : std::uint16_t {
    Aud1First = 1U << 0,
    Aud1Last = 1U << 1,
    Aud1Discontinuity = 1U << 2,
    Aud1Abort = 1U << 3,
};

struct Aud1Header {
    Aud1Kind kind{Aud1Kind::RxUlaw};
    std::uint16_t flags{};
    std::uint32_t streamId{};
    std::uint32_t sequence{};
    std::uint32_t sampleRate{};
    std::uint64_t firstSample{};
    std::uint32_t txId{};
};

struct Aud1Message {
    Aud1Header header;
    std::vector<std::uint8_t> payload;
};

[[nodiscard]] std::vector<std::uint8_t>
encodeAud1(Aud1Header const &header, std::span<const std::uint8_t> payload);
[[nodiscard]] std::optional<Aud1Message>
decodeAud1(std::span<const std::uint8_t> wire);

} // namespace js8proto
