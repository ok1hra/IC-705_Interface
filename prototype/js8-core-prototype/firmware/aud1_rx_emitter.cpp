// PROTOTYPE — reference firmware-side AUD1 RX envelope state, not integrated.

#include "aud1_rx_emitter.hpp"

#include <sstream>

namespace js8proto::firmware {

bool Aud1RxEmitter::beginSession(std::uint32_t streamId) {
    if (streamId == 0 || pending_) return false;
    streamId_ = streamId;
    sequence_ = 0;
    firstSample_ = 0;
    pendingSamples_ = 0;
    first_ = true;
    discontinuity_ = false;
    pending_ = false;
    return true;
}

std::string Aud1RxEmitter::helloJson() const {
    if (streamId_ == 0) return {};
    std::ostringstream out;
    out << "{\"type\":\"hello\",\"protocol\":\"AUD1\",\"version\":1,"
        << "\"streamId\":" << streamId_
        << ",\"rx\":[{\"kind\":\"RX_ULAW\",\"sampleRate\":8000}],"
        << "\"tx\":[{\"kind\":\"TX_PCM16\",\"sampleRate\":48000}],"
        << "\"maxPayloadBytes\":2048}";
    return out.str();
}

std::optional<PreparedRxFrame>
Aud1RxEmitter::prepare(std::span<const std::uint8_t> ulaw) {
    if (streamId_ == 0 || pending_ || ulaw.empty() ||
        ulaw.size() > kMaxPayloadBytes)
        return std::nullopt;
    std::uint16_t flags = 0;
    if (first_) flags |= Aud1First;
    if (discontinuity_) flags |= Aud1Discontinuity;
    auto wire = encodeAud1({Aud1Kind::RxUlaw, flags, streamId_, sequence_,
                            kSampleRate, firstSample_, 0}, ulaw);
    if (wire.empty()) return std::nullopt;
    pending_ = true;
    pendingSamples_ = ulaw.size();
    return PreparedRxFrame{std::move(wire), sequence_, firstSample_, ulaw.size()};
}

bool Aud1RxEmitter::commit(bool sent) {
    if (!pending_) return false;
    firstSample_ += pendingSamples_;
    if (sent) {
        ++sequence_;
        first_ = false;
        discontinuity_ = false;
    } else {
        discontinuity_ = true;
    }
    pendingSamples_ = 0;
    pending_ = false;
    return true;
}

} // namespace js8proto::firmware
