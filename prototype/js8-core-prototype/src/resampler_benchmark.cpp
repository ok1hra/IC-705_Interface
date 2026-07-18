// PROTOTYPE — compare the reference sinc loop with the fixed 8->12k polyphase FIR.
// Question: can precomputed phases preserve streaming output and JS8 fidelity
// while removing trigonometry from the browser audio hot path?

#include "audio_frontend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <span>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

std::vector<float> signal(std::size_t samples) {
    std::vector<float> result(samples);
    for (std::size_t i = 0; i < samples; ++i) {
        auto const t = static_cast<double>(i) / 8000.0;
        result[i] = static_cast<float>(
            0.22 * std::sin(2 * std::numbers::pi * 300 * t) +
            0.19 * std::sin(2 * std::numbers::pi * 1500 * t + 0.2) +
            0.15 * std::sin(2 * std::numbers::pi * 2700 * t + 0.7) +
            0.08 * std::sin(2 * std::numbers::pi * 3500 * t + 1.1));
    }
    return result;
}

template <typename Resampler>
std::pair<std::vector<float>, double>
run(Resampler &resampler, std::span<const float> input, std::size_t chunk) {
    auto const started = Clock::now();
    std::vector<float> output;
    output.reserve((input.size() * 3 + 1) / 2);
    for (std::size_t offset = 0; offset < input.size(); offset += chunk) {
        auto const count = std::min(chunk, input.size() - offset);
        auto block = resampler.push(input.subspan(offset, count));
        output.insert(output.end(), block.begin(), block.end());
    }
    auto tail = resampler.finish();
    output.insert(output.end(), tail.begin(), tail.end());
    auto const elapsed = std::chrono::duration<double, std::milli>(
        Clock::now() - started).count();
    return {std::move(output), elapsed};
}

struct Difference {
    double rms{};
    double referenceRms{};
    double maximum{};
    double snrDb{};
};

Difference difference(std::span<const float> reference,
                      std::span<const float> candidate) {
    Difference result;
    double errorSquares = 0.0;
    double referenceSquares = 0.0;
    auto const count = std::min(reference.size(), candidate.size());
    for (std::size_t i = 0; i < count; ++i) {
        auto const error = static_cast<double>(candidate[i]) - reference[i];
        errorSquares += error * error;
        referenceSquares += static_cast<double>(reference[i]) * reference[i];
        result.maximum = std::max(result.maximum, std::abs(error));
    }
    result.rms = count ? std::sqrt(errorSquares / count) : 0.0;
    result.referenceRms = count ? std::sqrt(referenceSquares / count) : 0.0;
    result.snrDb = result.rms > 0.0
        ? 20.0 * std::log10(result.referenceRms / result.rms)
        : 300.0;
    return result;
}
} // namespace

int main() {
    constexpr std::size_t inputSamples = 60 * 8000;
    auto const input = signal(inputSamples);

    js8proto::SincResampler reference(8000, 12000);
    auto [referenceOutput, referenceMs] = run(reference, input, 160);

    js8proto::PolyphaseResampler8to12 candidate;
    auto [candidateOutput, candidateMs] = run(candidate, input, 160);

    js8proto::PolyphaseResampler8to12 wholeBlock;
    auto [wholeOutput, wholeMs] = run(wholeBlock, input, input.size());
    auto const fidelity = difference(referenceOutput, candidateOutput);
    auto const chunking = difference(wholeOutput, candidateOutput);
    auto const expected = (inputSamples * 3 + 1) / 2;
    auto const valid = referenceOutput.size() == expected &&
                       candidateOutput.size() == expected &&
                       wholeOutput.size() == expected &&
                       fidelity.maximum < 1e-5 && chunking.maximum == 0.0;

    std::cout << std::fixed << std::setprecision(3)
              << "RESAMPLER " << (valid ? "PASS" : "FAIL") << '\n'
              << "samples input8k=" << inputSamples
              << " expected12k=" << expected
              << " reference12k=" << referenceOutput.size()
              << " polyphase12k=" << candidateOutput.size() << '\n'
              << std::setprecision(9)
              << "fidelity rms_error=" << fidelity.rms
              << " max_error=" << fidelity.maximum
              << std::setprecision(2) << " snr_db=" << fidelity.snrDb << '\n'
              << std::setprecision(9)
              << "chunk_invariance max_error=" << chunking.maximum << '\n'
              << std::setprecision(3)
              << "timing reference_ms=" << referenceMs
              << " polyphase_ms=" << candidateMs
              << " whole_block_ms=" << wholeMs
              << " speedup=" << (candidateMs > 0.0 ? referenceMs / candidateMs : 0.0)
              << "x\n"
              << "working_set peak_stream_samples=" << candidate.peakBufferedSamples()
              << " coefficient_count="
              << js8proto::PolyphaseResampler8to12::coefficientCount()
              << " coefficient_bytes="
              << js8proto::PolyphaseResampler8to12::coefficientCount() * sizeof(double)
              << '\n';
    return valid ? 0 : 1;
}
