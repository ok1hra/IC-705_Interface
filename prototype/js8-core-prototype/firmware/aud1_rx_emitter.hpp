// PROTOTYPE — reference firmware-side AUD1 RX envelope state, not integrated.
#pragma once

#include "aud1_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace js8proto::firmware {

struct PreparedRxFrame {
    std::vector<std::uint8_t> wire;
    std::uint32_t sequence{};
    std::uint64_t firstSample{};
    std::size_t sampleCount{};
};

class Aud1RxEmitter {
  public:
    static constexpr std::uint32_t kSampleRate = 8000;
    static constexpr std::size_t kMaxPayloadBytes = 2048;

    bool beginSession(std::uint32_t streamId);
    [[nodiscard]] std::string helloJson() const;
    [[nodiscard]] std::optional<PreparedRxFrame>
    prepare(std::span<const std::uint8_t> ulaw);
    bool commit(bool sent);

    [[nodiscard]] std::uint32_t streamId() const { return streamId_; }
    [[nodiscard]] std::uint32_t nextSequence() const { return sequence_; }
    [[nodiscard]] std::uint64_t nextSample() const { return firstSample_; }
    [[nodiscard]] bool discontinuityPending() const { return discontinuity_; }

  private:
    std::uint32_t streamId_{};
    std::uint32_t sequence_{};
    std::uint64_t firstSample_{};
    std::size_t pendingSamples_{};
    bool first_{true};
    bool discontinuity_{};
    bool pending_{};
};

} // namespace js8proto::firmware
