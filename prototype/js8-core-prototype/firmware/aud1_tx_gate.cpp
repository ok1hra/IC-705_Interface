// PROTOTYPE — firmware-owned paced TX/prebuffer/PTT gate, not integrated.

#include "aud1_tx_gate.hpp"

#include <algorithm>
#include <limits>

namespace js8proto::firmware {

Aud1TxGate::Aud1TxGate(std::size_t maxBufferedSamples, bool captureMode)
    : maxBufferedSamples_(maxBufferedSamples), captureMode_(captureMode) {}

bool Aud1TxGate::prepare(TxPrepare request, std::uint64_t nowUtcMs) {
    if (!(state_ == TxGateState::Idle || state_ == TxGateState::Drained ||
          state_ == TxGateState::Aborted || state_ == TxGateState::Fault) ||
        request.streamId == 0 || request.txId == 0 ||
        request.slotUtcMs <= nowUtcMs || request.totalSamples == 0 ||
        request.totalPackets == 0 || request.prebufferSamples == 0 ||
        request.prebufferSamples > request.totalSamples ||
        request.prebufferSamples > maxBufferedSamples_) {
        return false;
    }
    request_ = request;
    state_ = TxGateState::Ready;
    ptt_ = false;
    firstSeen_ = false;
    lastSeen_ = false;
    expectedSequence_ = 0;
    receivedSamples_ = 0;
    consumedSamples_ = 0;
    error_.clear();
    capture_.clear();
    if (captureMode_ && request.totalSamples <=
            static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
        capture_.reserve(static_cast<std::size_t>(request.totalSamples));
    return true; // tx-ready means capacity reserved; PTT remains OFF.
}

bool Aud1TxGate::accept(std::span<const std::uint8_t> wire,
                        std::uint64_t nowUtcMs) {
    if (!(state_ == TxGateState::Ready || state_ == TxGateState::Prebuffering ||
          state_ == TxGateState::Transmitting))
        return false;
    auto decoded = decodeAud1(wire);
    if (!decoded || decoded->header.kind != Aud1Kind::TxPcm16 ||
        decoded->header.sampleRate != kSampleRate ||
        decoded->header.streamId != request_.streamId ||
        decoded->header.txId != request_.txId ||
        decoded->header.sequence != expectedSequence_ ||
        decoded->header.firstSample != receivedSamples_) {
        fail("TX packet identity/continuity failure");
        return false;
    }
    auto const samples = decoded->payload.size() / 2;
    auto const first = (decoded->header.flags & Aud1First) != 0;
    auto const last = (decoded->header.flags & Aud1Last) != 0;
    if ((expectedSequence_ == 0) != first || (expectedSequence_ > 0 && first) ||
        receivedSamples_ + samples > request_.totalSamples ||
        receivedSamples_ + samples - consumedSamples_ > maxBufferedSamples_) {
        fail("TX FIRST/length/buffer failure");
        return false;
    }
    if (last && (receivedSamples_ + samples != request_.totalSamples ||
                 expectedSequence_ + 1 != request_.totalPackets)) {
        fail("premature TX LAST");
        return false;
    }
    if (!last && expectedSequence_ + 1 == request_.totalPackets) {
        fail("TX stream lacks LAST");
        return false;
    }
    if (captureMode_) {
        for (std::size_t at = 0; at < decoded->payload.size(); at += 2) {
            auto const value = static_cast<std::uint16_t>(decoded->payload[at]) |
                (static_cast<std::uint16_t>(decoded->payload[at + 1]) << 8);
            capture_.push_back(static_cast<std::int16_t>(value));
        }
    }
    firstSeen_ = true;
    lastSeen_ = last;
    receivedSamples_ += samples;
    ++expectedSequence_;
    if (state_ == TxGateState::Ready) state_ = TxGateState::Prebuffering;
    // Count a frame already being handled at the boundary before evaluating
    // the slot. This mirrors the browser->firmware WebSocket service order.
    tick(nowUtcMs);
    return true;
}

void Aud1TxGate::tick(std::uint64_t nowUtcMs) {
    if (state_ == TxGateState::Ready || state_ == TxGateState::Prebuffering) {
        if (nowUtcMs < request_.slotUtcMs) return;
        if (!firstSeen_ || receivedSamples_ < request_.prebufferSamples) {
            fail("TX prebuffer missed slot");
            return;
        }
        state_ = TxGateState::Transmitting;
        ptt_ = true;
    }
    if (state_ != TxGateState::Transmitting) return;
    auto const elapsedMs = nowUtcMs > request_.slotUtcMs
        ? nowUtcMs - request_.slotUtcMs : 0;
    auto const wanted = std::min<std::uint64_t>(request_.totalSamples,
                                                elapsedMs * 48);
    if (wanted > receivedSamples_) {
        fail("TX buffer underrun");
        return;
    }
    consumedSamples_ = wanted;
    if (lastSeen_ && consumedSamples_ >= request_.totalSamples) {
        ptt_ = false;
        state_ = TxGateState::Drained;
    }
}

void Aud1TxGate::abort(std::string reason) {
    ptt_ = false;
    error_ = std::move(reason);
    state_ = TxGateState::Aborted;
}

void Aud1TxGate::disconnect() { abort("WebSocket disconnected"); }

void Aud1TxGate::fail(std::string reason) {
    ptt_ = false;
    error_ = std::move(reason);
    state_ = TxGateState::Fault;
}

TxGateSnapshot Aud1TxGate::snapshot() const {
    return {state_, ptt_, receivedSamples_, consumedSamples_,
            receivedSamples_ - consumedSamples_, expectedSequence_,
            lastSeen_, error_};
}

char const *txGateStateName(TxGateState state) {
    switch (state) {
    case TxGateState::Idle: return "idle";
    case TxGateState::Ready: return "ready";
    case TxGateState::Prebuffering: return "prebuffering";
    case TxGateState::Transmitting: return "transmitting";
    case TxGateState::Drained: return "drained";
    case TxGateState::Aborted: return "aborted";
    case TxGateState::Fault: return "fault";
    }
    return "unknown";
}

} // namespace js8proto::firmware
