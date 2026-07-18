// PROTOTYPE — packet audio/timebase candidate, intentionally not production.

#include "audio_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace js8proto {
namespace {
constexpr int kUlawBias = 0x84;
constexpr int kUlawClip = 32635;

double sinc(double value) {
    if (std::abs(value) < 1e-12)
        return 1.0;
    auto const angle = std::numbers::pi * value;
    return std::sin(angle) / angle;
}

std::int64_t firstSlotAtOrAfter(std::int64_t utcMs, int periodMs) {
    auto remainder = utcMs % periodMs;
    if (remainder < 0)
        remainder += periodMs;
    return remainder == 0 ? utcMs : utcMs + periodMs - remainder;
}
} // namespace

float decodeUlaw(std::uint8_t value) {
    auto const encoded = static_cast<std::uint8_t>(~value);
    auto const sign = encoded & 0x80;
    auto const exponent = (encoded >> 4) & 0x07;
    auto const mantissa = encoded & 0x0f;
    auto magnitude = ((static_cast<int>(mantissa) << 3) + kUlawBias)
                     << exponent;
    magnitude -= kUlawBias;
    auto const sample = sign ? -magnitude : magnitude;
    return std::clamp(static_cast<float>(sample) / 32768.0f, -1.0f, 1.0f);
}

std::uint8_t encodeUlaw(float value) {
    auto sample = static_cast<int>(std::lround(
        std::clamp(value, -1.0f, 1.0f) * 32767.0f));
    auto const sign = sample < 0 ? 0x80 : 0;
    if (sample < 0)
        sample = -sample;
    sample = std::min(sample, kUlawClip) + kUlawBias;

    int exponent = 7;
    for (int mask = 0x4000; exponent > 0 && !(sample & mask);
         mask >>= 1, --exponent) {
    }
    auto const mantissa = (sample >> (exponent + 3)) & 0x0f;
    return static_cast<std::uint8_t>(~(sign | (exponent << 4) | mantissa));
}

SincResampler::SincResampler(int inputRate, int outputRate, int halfLength)
    : inputRate_(inputRate), outputRate_(outputRate),
      halfLength_(halfLength),
      cutoff_(0.94 * std::min(1.0, static_cast<double>(outputRate) /
                                      static_cast<double>(inputRate))) {}

std::vector<float> SincResampler::push(std::span<const float> input) {
    if (finished_ || input.empty())
        return {};
    input_.insert(input_.end(), input.begin(), input.end());
    totalInput_ += input.size();
    auto output = generate(false);
    discardOldInput();
    return output;
}

std::vector<float> SincResampler::finish() {
    if (finished_)
        return {};
    finished_ = true;
    auto output = generate(true);
    input_.clear();
    bufferStart_ = totalInput_;
    return output;
}

void SincResampler::reset() {
    input_.clear();
    bufferStart_ = 0;
    totalInput_ = 0;
    nextOutput_ = 0;
    finished_ = false;
}

float SincResampler::sampleAt(std::int64_t index) const {
    if (index < 0 || static_cast<std::uint64_t>(index) >= totalInput_)
        return 0.0f;
    auto const absolute = static_cast<std::uint64_t>(index);
    if (absolute < bufferStart_)
        return 0.0f;
    return input_[absolute - bufferStart_];
}

std::vector<float> SincResampler::generate(bool flushing) {
    std::vector<float> output;
    auto const target = static_cast<std::uint64_t>(std::llround(
        static_cast<long double>(totalInput_) * outputRate_ / inputRate_));

    while (nextOutput_ < target) {
        auto const sourcePosition =
            static_cast<double>(nextOutput_) * inputRate_ / outputRate_;
        auto const center = static_cast<std::int64_t>(std::floor(sourcePosition));
        if (!flushing && center + halfLength_ >=
                             static_cast<std::int64_t>(totalInput_))
            break;

        double sum = 0.0;
        double weightSum = 0.0;
        for (auto index = center - halfLength_ + 1;
             index <= center + halfLength_; ++index) {
            auto const distance = sourcePosition - static_cast<double>(index);
            auto const window =
                0.5 + 0.5 * std::cos(std::numbers::pi * distance /
                                     static_cast<double>(halfLength_));
            auto const weight = cutoff_ * sinc(cutoff_ * distance) * window;
            sum += static_cast<double>(sampleAt(index)) * weight;
            weightSum += weight;
        }
        output.push_back(weightSum == 0.0
                             ? 0.0f
                             : static_cast<float>(sum / weightSum));
        ++nextOutput_;
    }
    return output;
}

void SincResampler::discardOldInput() {
    auto const nextSource = static_cast<std::int64_t>(
        std::floor(static_cast<double>(nextOutput_) * inputRate_ / outputRate_));
    auto const keepFrom = std::max<std::int64_t>(0, nextSource - halfLength_ - 2);
    if (keepFrom <= static_cast<std::int64_t>(bufferStart_) + 4096)
        return;
    auto const remove = static_cast<std::size_t>(keepFrom - bufferStart_);
    input_.erase(input_.begin(), input_.begin() +
                                     std::min(remove, input_.size()));
    bufferStart_ = static_cast<std::uint64_t>(keepFrom);
}

PolyphaseResampler8to12::PolyphaseResampler8to12() {
    constexpr double cutoff = 0.94;
    for (int phase = 0; phase < kPhaseCount; ++phase) {
        auto const fraction = static_cast<double>(phase) / kPhaseCount;
        double weightSum = 0.0;
        for (int tap = 0; tap < kTapsPerPhase; ++tap) {
            auto const relativeIndex = -kHalfLength + 1 + tap;
            auto const distance = fraction - relativeIndex;
            auto const window =
                0.5 + 0.5 * std::cos(std::numbers::pi * distance /
                                     static_cast<double>(kHalfLength));
            auto const weight = cutoff * sinc(cutoff * distance) * window;
            phases_[phase][tap] = weight;
            weightSum += weight;
        }
        for (auto &weight : phases_[phase])
            weight /= weightSum;
    }
}

std::vector<float>
PolyphaseResampler8to12::push(std::span<const float> input) {
    if (finished_ || input.empty())
        return {};
    input_.insert(input_.end(), input.begin(), input.end());
    totalInput_ += input.size();
    peakBufferedSamples_ = std::max(peakBufferedSamples_, input_.size());
    auto output = generate(false);
    discardOldInput();
    return output;
}

std::vector<float> PolyphaseResampler8to12::finish() {
    if (finished_)
        return {};
    finished_ = true;
    auto output = generate(true);
    input_.clear();
    bufferStart_ = totalInput_;
    return output;
}

void PolyphaseResampler8to12::reset() {
    input_.clear();
    bufferStart_ = 0;
    totalInput_ = 0;
    nextOutput_ = 0;
    peakBufferedSamples_ = 0;
    finished_ = false;
}

float PolyphaseResampler8to12::sampleAt(std::int64_t index) const {
    if (index < 0 || static_cast<std::uint64_t>(index) >= totalInput_)
        return 0.0f;
    auto const absolute = static_cast<std::uint64_t>(index);
    if (absolute < bufferStart_)
        return 0.0f;
    return input_[absolute - bufferStart_];
}

std::vector<float> PolyphaseResampler8to12::generate(bool flushing) {
    std::vector<float> output;
    auto const target = (totalInput_ * 3 + 1) / 2;
    while (nextOutput_ < target) {
        auto const sourceNumerator = nextOutput_ * 2;
        auto const center = static_cast<std::int64_t>(sourceNumerator / 3);
        auto const phase = static_cast<std::size_t>(sourceNumerator % 3);
        if (!flushing && center + kHalfLength >=
                             static_cast<std::int64_t>(totalInput_))
            break;

        double sum = 0.0;
        auto const first = center - kHalfLength + 1;
        for (int tap = 0; tap < kTapsPerPhase; ++tap)
            sum += static_cast<double>(sampleAt(first + tap)) *
                   phases_[phase][tap];
        output.push_back(static_cast<float>(sum));
        ++nextOutput_;
    }
    return output;
}

void PolyphaseResampler8to12::discardOldInput() {
    auto const nextSource = static_cast<std::int64_t>((nextOutput_ * 2) / 3);
    auto const keepFrom =
        std::max<std::int64_t>(0, nextSource - kHalfLength - 2);
    if (keepFrom <= static_cast<std::int64_t>(bufferStart_) + 4096)
        return;
    auto const remove = static_cast<std::size_t>(keepFrom - bufferStart_);
    input_.erase(input_.begin(), input_.begin() +
                                     std::min(remove, input_.size()));
    bufferStart_ = static_cast<std::uint64_t>(keepFrom);
}

AudioTimeline::AudioTimeline() = default;

AudioIngestResult AudioTimeline::ingest(AudioPacket const &packet) {
    AudioIngestResult result{};
    result.firstOutput12k = resampler_.outputCount();
    if (packet.ulaw.empty())
        return result;

    if (!anchored_) {
        anchored_ = true;
        originSample8k_ = packet.firstSample8k;
        expectedSample8k_ = packet.firstSample8k;
    }

    auto const packetEnd = packet.firstSample8k + packet.ulaw.size();
    if (packetEnd <= expectedSample8k_) {
        ++duplicatePackets_;
        result.duplicate = true;
        return result;
    }

    std::vector<float> decoded;
    if (packet.firstSample8k > expectedSample8k_) {
        auto const gap = packet.firstSample8k - expectedSample8k_;
        appendSilence(gap, decoded);
        insertedGapSamples8k_ += gap;
        ++discontinuities_;
        result.discontinuity = true;
        result.insertedGap8k = gap;
    }

    auto skip = packet.firstSample8k < expectedSample8k_
                    ? expectedSample8k_ - packet.firstSample8k
                    : 0;
    appendDecoded(std::span(packet.ulaw).subspan(skip), decoded);
    expectedSample8k_ = packetEnd;

    if (expectedSequence_ && packet.sequence != *expectedSequence_)
        ++sequenceGaps_;
    expectedSequence_ = packet.sequence + 1;

    if (packet.discontinuity) {
        ++discontinuities_;
        dcPreviousInput_ = 0.0f;
        dcPreviousOutput_ = 0.0f;
        result.discontinuity = true;
    }

    if (lastArrivalMs_ && lastPacketFirst_) {
        auto const arrivalDelta = packet.arrivalMs - *lastArrivalMs_;
        auto const mediaDelta =
            static_cast<double>(packet.firstSample8k - *lastPacketFirst_) /
            8.0;
        maxArrivalJitterMs_ = std::max(maxArrivalJitterMs_,
                                      std::abs(arrivalDelta - mediaDelta));
    }
    lastArrivalMs_ = packet.arrivalMs;
    lastPacketFirst_ = packet.firstSample8k;

    result.pcm12k = resampler_.push(decoded);
    result.accepted = true;
    ++acceptedPackets_;
    return result;
}

std::vector<float> AudioTimeline::finish() { return resampler_.finish(); }

void AudioTimeline::reset() {
    *this = AudioTimeline{};
}

void AudioTimeline::appendDecoded(std::span<const std::uint8_t> ulaw,
                                  std::vector<float> &decoded) {
    decoded.reserve(decoded.size() + ulaw.size());
    for (auto const value : ulaw)
        decoded.push_back(filterAndMeasure(decodeUlaw(value)));
}

void AudioTimeline::appendSilence(std::size_t count,
                                  std::vector<float> &decoded) {
    decoded.reserve(decoded.size() + count);
    for (std::size_t i = 0; i < count; ++i)
        decoded.push_back(filterAndMeasure(0.0f));
}

float AudioTimeline::filterAndMeasure(float input) {
    constexpr float pole = 0.995f;
    auto const output = input - dcPreviousInput_ + pole * dcPreviousOutput_;
    dcPreviousInput_ = input;
    dcPreviousOutput_ = output;
    peak_ = std::max(peak_, std::abs(output));
    squareSum_ += static_cast<double>(output) * output;
    ++measuredSamples_;
    return output;
}

AudioTimelineState AudioTimeline::state() const {
    return {anchored_,
            originSample8k_,
            expectedSample8k_,
            acceptedPackets_,
            duplicatePackets_,
            sequenceGaps_,
            discontinuities_,
            insertedGapSamples8k_,
            resampler_.outputCount(),
            maxArrivalJitterMs_,
            peak_,
            measuredSamples_ == 0
                ? 0.0
                : std::sqrt(squareSum_ / measuredSamples_)};
}

DecodeScheduler::DecodeScheduler(std::int64_t anchorUtcMs) {
    reset(anchorUtcMs);
}

void DecodeScheduler::reset(std::int64_t anchorUtcMs) {
    anchorUtcMs_ = anchorUtcMs;
    auto const &specs = submodes();
    for (std::size_t i = 0; i < specs.size(); ++i) {
        auto const slot = firstSlotAtOrAfter(anchorUtcMs_, specs[i].periodMs);
        lanes_[i] = {specs[i].id, specs[i].periodMs, slot};
    }
}

std::vector<DecodeWindow>
DecodeScheduler::appendThrough(std::uint64_t producedSamples12k) {
    std::vector<DecodeWindow> ready;
    auto const availableUtcMs = anchorUtcMs_ +
        static_cast<std::int64_t>(producedSamples12k / 12);

    for (auto &lane : lanes_) {
        while (lane.nextSlotUtcMs + lane.periodMs <= availableUtcMs) {
            auto const offsetMs = lane.nextSlotUtcMs - anchorUtcMs_;
            if (offsetMs >= 0) {
                ready.push_back({lane.mode,
                                 lane.nextSlotUtcMs,
                                 static_cast<std::uint64_t>(offsetMs) * 12,
                                 static_cast<std::uint64_t>(lane.periodMs) * 12});
            }
            lane.nextSlotUtcMs += lane.periodMs;
        }
    }
    return ready;
}

} // namespace js8proto
