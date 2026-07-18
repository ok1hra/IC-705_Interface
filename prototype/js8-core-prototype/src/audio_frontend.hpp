// PROTOTYPE — packet audio/timebase candidate, intentionally not production.
#pragma once

#include "js8_core.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace js8proto {

[[nodiscard]] float decodeUlaw(std::uint8_t value);
[[nodiscard]] std::uint8_t encodeUlaw(float value);

class SincResampler {
  public:
    SincResampler(int inputRate, int outputRate, int halfLength = 32);

    [[nodiscard]] std::vector<float> push(std::span<const float> input);
    [[nodiscard]] std::vector<float> finish();
    void reset();

    [[nodiscard]] std::uint64_t inputCount() const { return totalInput_; }
    [[nodiscard]] std::uint64_t outputCount() const { return nextOutput_; }

  private:
    [[nodiscard]] std::vector<float> generate(bool flushing);
    [[nodiscard]] float sampleAt(std::int64_t index) const;
    void discardOldInput();

    int inputRate_;
    int outputRate_;
    int halfLength_;
    double cutoff_;
    std::vector<float> input_;
    std::uint64_t bufferStart_{};
    std::uint64_t totalInput_{};
    std::uint64_t nextOutput_{};
    bool finished_{};
};

// Fixed 8 kHz -> 12 kHz rational resampler. It is the production-oriented
// candidate: three precomputed fractional-delay phases replace trigonometry in
// the streaming hot path while retaining the reference filter response.
class PolyphaseResampler8to12 {
  public:
    static constexpr int kInputRate = 8000;
    static constexpr int kOutputRate = 12000;
    static constexpr int kHalfLength = 32;
    static constexpr int kPhaseCount = 3;
    static constexpr int kTapsPerPhase = kHalfLength * 2;

    PolyphaseResampler8to12();

    [[nodiscard]] std::vector<float> push(std::span<const float> input);
    [[nodiscard]] std::vector<float> finish();
    void reset();

    [[nodiscard]] std::uint64_t inputCount() const { return totalInput_; }
    [[nodiscard]] std::uint64_t outputCount() const { return nextOutput_; }
    [[nodiscard]] std::size_t peakBufferedSamples() const {
        return peakBufferedSamples_;
    }
    [[nodiscard]] static constexpr std::size_t coefficientCount() {
        return kPhaseCount * kTapsPerPhase;
    }

  private:
    [[nodiscard]] std::vector<float> generate(bool flushing);
    [[nodiscard]] float sampleAt(std::int64_t index) const;
    void discardOldInput();

    std::array<std::array<double, kTapsPerPhase>, kPhaseCount> phases_{};
    std::vector<float> input_;
    std::uint64_t bufferStart_{};
    std::uint64_t totalInput_{};
    std::uint64_t nextOutput_{};
    std::size_t peakBufferedSamples_{};
    bool finished_{};
};

struct AudioPacket {
    std::uint32_t sequence{};
    std::uint64_t firstSample8k{};
    double arrivalMs{};
    bool discontinuity{};
    std::vector<std::uint8_t> ulaw;
};

struct AudioIngestResult {
    bool accepted{};
    bool duplicate{};
    bool discontinuity{};
    std::uint64_t insertedGap8k{};
    std::uint64_t firstOutput12k{};
    std::vector<float> pcm12k;
};

struct AudioTimelineState {
    bool anchored{};
    std::uint64_t originSample8k{};
    std::uint64_t expectedSample8k{};
    std::uint64_t acceptedPackets{};
    std::uint64_t duplicatePackets{};
    std::uint64_t sequenceGaps{};
    std::uint64_t discontinuities{};
    std::uint64_t insertedGapSamples8k{};
    std::uint64_t producedSamples12k{};
    double maxArrivalJitterMs{};
    float peak{};
    double rms{};
};

class AudioTimeline {
  public:
    AudioTimeline();

    [[nodiscard]] AudioIngestResult ingest(AudioPacket const &packet);
    [[nodiscard]] std::vector<float> finish();
    void reset();
    [[nodiscard]] AudioTimelineState state() const;

  private:
    void appendDecoded(std::span<const std::uint8_t> ulaw,
                       std::vector<float> &decoded);
    void appendSilence(std::size_t count, std::vector<float> &decoded);
    float filterAndMeasure(float input);

    PolyphaseResampler8to12 resampler_;
    bool anchored_{};
    std::uint64_t originSample8k_{};
    std::uint64_t expectedSample8k_{};
    std::optional<std::uint32_t> expectedSequence_;
    std::optional<double> lastArrivalMs_;
    std::optional<std::uint64_t> lastPacketFirst_;
    std::uint64_t acceptedPackets_{};
    std::uint64_t duplicatePackets_{};
    std::uint64_t sequenceGaps_{};
    std::uint64_t discontinuities_{};
    std::uint64_t insertedGapSamples8k_{};
    double maxArrivalJitterMs_{};
    float dcPreviousInput_{};
    float dcPreviousOutput_{};
    double squareSum_{};
    std::uint64_t measuredSamples_{};
    float peak_{};
};

struct DecodeWindow {
    Submode submode{};
    std::int64_t slotUtcMs{};
    std::uint64_t firstSample12k{};
    std::uint64_t sampleCount{};
};

class DecodeScheduler {
  public:
    explicit DecodeScheduler(std::int64_t anchorUtcMs = 0);
    [[nodiscard]] std::vector<DecodeWindow>
    appendThrough(std::uint64_t producedSamples12k);
    void reset(std::int64_t anchorUtcMs = 0);

  private:
    struct Lane {
        Submode mode{};
        int periodMs{};
        std::int64_t nextSlotUtcMs{};
    };

    std::int64_t anchorUtcMs_{};
    std::array<Lane, 5> lanes_{};
};

} // namespace js8proto
