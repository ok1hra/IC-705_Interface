// PROTOTYPE — firmware-owned paced TX/prebuffer/PTT gate, not integrated.
#pragma once

#include "aud1_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace js8proto::firmware {

enum class TxGateState {
    Idle,
    Ready,
    Prebuffering,
    Transmitting,
    Drained,
    Aborted,
    Fault,
};

struct TxPrepare {
    std::uint32_t streamId{};
    std::uint32_t txId{};
    std::uint64_t slotUtcMs{};
    std::uint64_t totalSamples{};
    std::uint32_t totalPackets{};
    std::uint32_t prebufferSamples{48000}; // 1 s at 48 kHz.
};

struct TxGateSnapshot {
    TxGateState state{TxGateState::Idle};
    bool ptt{};
    std::uint64_t receivedSamples{};
    std::uint64_t consumedSamples{};
    std::uint64_t bufferedSamples{};
    std::uint32_t receivedPackets{};
    bool lastSeen{};
    std::string error;
};

class Aud1TxGate {
  public:
    explicit Aud1TxGate(std::size_t maxBufferedSamples = 48000,
                        bool captureMode = false);
    bool prepare(TxPrepare request, std::uint64_t nowUtcMs);
    bool accept(std::span<const std::uint8_t> wire, std::uint64_t nowUtcMs);
    void tick(std::uint64_t nowUtcMs);
    void abort(std::string reason);
    void disconnect();
    [[nodiscard]] TxGateSnapshot snapshot() const;
    [[nodiscard]] std::span<const std::int16_t> capture() const { return capture_; }

  private:
    void fail(std::string reason);
    static constexpr std::uint32_t kSampleRate = 48000;
    std::size_t maxBufferedSamples_;
    bool captureMode_;
    TxPrepare request_{};
    TxGateState state_{TxGateState::Idle};
    bool ptt_{};
    bool firstSeen_{};
    bool lastSeen_{};
    std::uint32_t expectedSequence_{};
    std::uint64_t receivedSamples_{};
    std::uint64_t consumedSamples_{};
    std::string error_;
    std::vector<std::int16_t> capture_;
};

[[nodiscard]] char const *txGateStateName(TxGateState state);

} // namespace js8proto::firmware
