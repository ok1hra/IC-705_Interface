// PROTOTYPE — throwaway shell, portable core candidate.
// Derived in part from JS8Call-improved, GPLv3.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace js8proto {

enum class Submode : int {
    Normal = 0,
    Fast = 1,
    Js840 = 2,
    Slow = 4,
    Js860 = 8,
};

enum class Costas : int { Original, Modified };

struct SubmodeSpec {
    Submode id;
    std::string_view name;
    int decoderMask;
    int periodMs;
    int samplesPerSymbol12k;
    int startDelayMs;
    int rxThresholdHz;
    Costas costas;

    [[nodiscard]] double toneSpacingHz() const;
    [[nodiscard]] double bandwidthHz() const;
    [[nodiscard]] double symbolDurationMs() const;
};

struct SlotPlan {
    std::int64_t nowUtcMs;
    std::int64_t slotUtcMs;
    std::int64_t waitMs;
    int clockCorrectionMs;
};

struct EncodedFrame {
    std::array<std::uint8_t, 79> tones{};
    std::uint16_t crc12{};
    int type{};
    Submode submode{};
};

inline constexpr std::size_t kFrameChars = 12;
inline constexpr std::size_t kToneCount = 79;
inline constexpr int kRxSampleRate = 12000;

[[nodiscard]] const std::array<SubmodeSpec, 5>& submodes();
[[nodiscard]] const SubmodeSpec* findSubmode(int id);
[[nodiscard]] SlotPlan planNextSlot(std::int64_t nowUtcMs,
                                    const SubmodeSpec& mode,
                                    int clockCorrectionMs,
                                    int minimumLeadMs = 800);
[[nodiscard]] bool validToneOffset(double offsetHz, double rxLowHz,
                                   double rxHighHz,
                                   const SubmodeSpec& mode);
[[nodiscard]] std::optional<EncodedFrame> encodeFrame(std::string_view frame,
                                                      int type,
                                                      Submode mode);
[[nodiscard]] std::vector<std::int16_t>
modulateFrame12k(const EncodedFrame& frame, double baseFrequencyHz = 1500.0,
                 double amplitude = 0.65);
[[nodiscard]] std::vector<std::int16_t>
modulateFrame48k(const EncodedFrame& frame, double baseFrequencyHz = 1500.0,
                 double amplitude = 0.65);
[[nodiscard]] std::uint16_t crc12(const std::array<std::uint8_t, 11>& bytes);

} // namespace js8proto

extern "C" {

struct Js8ProtoSubmodeSpec {
    int id;
    int decoder_mask;
    int period_ms;
    int samples_per_symbol_12k;
    int start_delay_ms;
    int rx_threshold_hz;
    double tone_spacing_hz;
    double bandwidth_hz;
};

int js8_proto_get_submode(int id, Js8ProtoSubmodeSpec* out);
int js8_proto_encode_frame(const char* frame12, int type, int submode,
                           std::uint8_t* tones79, std::uint16_t* crc12);

}
