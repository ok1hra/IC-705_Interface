#pragma once

#include <stddef.h>
#include <stdint.h>

// Incremental parser for one browser -> firmware WebSocket frame. It consumes
// only bytes already read from the socket; waiting and work budgeting remain
// the caller's responsibility. Keeping this Arduino-independent also gives the
// production parser a native regression seam.
class Aud1WsParser {
public:
  enum Result : uint8_t { NeedMore, FrameReady, Error };
  static const size_t MaxPayload = 2048;

  Aud1WsParser() { reset(); }

  void reset() {
    stage_ = HeaderFirst;
    first_ = opcode_ = 0;
    final_ = masked_ = false;
    payloadLength_ = payloadIndex_ = 0;
    extendedRemaining_ = maskIndex_ = 0;
    for(uint8_t &value : mask_) value = 0;
  }

  Result push(uint8_t byte) {
    if(stage_ == Failed) return Error;
    if(stage_ == Complete) return fail();

    switch(stage_) {
      case HeaderFirst:
        first_ = byte;
        final_ = (byte & 0x80U) != 0;
        opcode_ = byte & 0x0FU;
        // Extensions and fragmented messages were never supported by this
        // endpoint. Reject them explicitly instead of desynchronising framing.
        if((byte & 0x70U) != 0 || !final_) return fail();
        stage_ = HeaderSecond;
        return NeedMore;

      case HeaderSecond: {
        masked_ = (byte & 0x80U) != 0;
        uint8_t const shortLength = byte & 0x7FU;
        payloadLength_ = 0;
        if(shortLength < 126U) {
          payloadLength_ = shortLength;
          return lengthReady();
        }
        extendedRemaining_ = shortLength == 126U ? 2U : 8U;
        stage_ = ExtendedLength;
        return NeedMore;
      }

      case ExtendedLength:
        // More than 32 significant length bits can never fit MaxPayload.
        if(payloadLength_ > 0xFFFFFFFFULL >> 8) return fail();
        payloadLength_ = (payloadLength_ << 8) | byte;
        if(--extendedRemaining_ == 0) return lengthReady();
        return NeedMore;

      case Mask:
        mask_[maskIndex_++] = byte;
        if(maskIndex_ < 4) return NeedMore;
        if(payloadLength_ == 0) return complete();
        stage_ = Payload;
        return NeedMore;

      case Payload:
        payload_[payloadIndex_] = masked_
          ? static_cast<uint8_t>(byte ^ mask_[payloadIndex_ % 4]) : byte;
        if(++payloadIndex_ == payloadLength_) return complete();
        return NeedMore;

      case Complete:
      case Failed:
        break;
    }
    return fail();
  }

  bool idle() const { return stage_ == HeaderFirst; }
  bool inProgress() const { return stage_ != HeaderFirst && stage_ != Complete && stage_ != Failed; }
  bool finalFrame() const { return final_; }
  bool masked() const { return masked_; }
  uint8_t opcode() const { return opcode_; }
  uint8_t* payload() { return payload_; }
  const uint8_t* payload() const { return payload_; }
  size_t payloadSize() const { return static_cast<size_t>(payloadLength_); }

private:
  enum Stage : uint8_t { HeaderFirst, HeaderSecond, ExtendedLength, Mask, Payload, Complete, Failed };

  Result lengthReady() {
    if(payloadLength_ > MaxPayload) return fail();
    payloadIndex_ = 0;
    maskIndex_ = 0;
    if(masked_) {
      stage_ = Mask;
      return NeedMore;
    }
    if(payloadLength_ == 0) return complete();
    stage_ = Payload;
    return NeedMore;
  }

  Result complete() {
    stage_ = Complete;
    return FrameReady;
  }

  Result fail() {
    stage_ = Failed;
    return Error;
  }

  Stage stage_;
  uint8_t first_;
  uint8_t opcode_;
  bool final_;
  bool masked_;
  uint64_t payloadLength_;
  size_t payloadIndex_;
  uint8_t extendedRemaining_;
  uint8_t maskIndex_;
  uint8_t mask_[4];
  uint8_t payload_[MaxPayload + 1];
};
