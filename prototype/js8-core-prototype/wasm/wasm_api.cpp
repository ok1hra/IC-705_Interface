// PROTOTYPE — C ABI intended for the Emscripten boundary.

#include "wasm_api.h"

#include "audio_frontend.hpp"
#include "js8_core.hpp"

#include <algorithm>
#include <cstddef>
#include <deque>
#include <limits>
#include <new>
#include <vector>

static_assert(sizeof(Js8AudioSnapshot) == 80);
static_assert(offsetof(Js8AudioSnapshot, expected_sample_8k) == 0);
static_assert(offsetof(Js8AudioSnapshot, duplicate_packets) == 16);
static_assert(offsetof(Js8AudioSnapshot, produced_samples_12k) == 48);
static_assert(offsetof(Js8AudioSnapshot, max_arrival_jitter_ms) == 56);
static_assert(offsetof(Js8AudioSnapshot, peak) == 64);
static_assert(offsetof(Js8AudioSnapshot, rms) == 72);

struct Js8AudioFrontend {
    explicit Js8AudioFrontend(std::int64_t anchor)
        : scheduler(anchor) {}

    js8proto::AudioTimeline timeline;
    js8proto::DecodeScheduler scheduler;
    std::vector<float> output;
    std::deque<js8proto::DecodeWindow> windows;
};

namespace {
void collectWindows(Js8AudioFrontend &frontend) {
    auto ready = frontend.scheduler.appendThrough(
        frontend.timeline.state().producedSamples12k);
    frontend.windows.insert(frontend.windows.end(), ready.begin(), ready.end());
}
} // namespace

extern "C" int js8_proto_modulate_frame48k(
    char const *frame12, int type, int submode, double base_frequency_hz,
    double amplitude, std::int16_t *output, std::size_t capacity) {
    if (!frame12)
        return 0;
    auto const encoded = js8proto::encodeFrame(
        std::string_view(frame12, js8proto::kFrameChars), type,
        static_cast<js8proto::Submode>(submode));
    if (!encoded)
        return 0;
    auto const pcm = js8proto::modulateFrame48k(
        *encoded, base_frequency_hz, amplitude);
    if (pcm.empty() || pcm.size() >
                           static_cast<std::size_t>(std::numeric_limits<int>::max()))
        return 0;
    if (!output)
        return static_cast<int>(pcm.size());
    if (capacity < pcm.size())
        return -static_cast<int>(pcm.size());
    std::copy(pcm.begin(), pcm.end(), output);
    return static_cast<int>(pcm.size());
}

extern "C" Js8AudioFrontend *js8_audio_create(std::int64_t anchor_utc_ms) {
    return new (std::nothrow) Js8AudioFrontend(anchor_utc_ms);
}

extern "C" void js8_audio_destroy(Js8AudioFrontend *frontend) {
    delete frontend;
}

extern "C" void js8_audio_reset(Js8AudioFrontend *frontend,
                                  std::int64_t anchor_utc_ms) {
    if (!frontend)
        return;
    frontend->timeline.reset();
    frontend->scheduler.reset(anchor_utc_ms);
    frontend->output.clear();
    frontend->windows.clear();
}

extern "C" int js8_audio_push_ulaw(
    Js8AudioFrontend *frontend, std::uint32_t sequence,
    std::uint64_t first_sample_8k, double arrival_monotonic_ms,
    int discontinuity, std::uint8_t const *ulaw, std::size_t count) {
    if (!frontend || !ulaw || count == 0)
        return -1;
    js8proto::AudioPacket packet{sequence,
                                 first_sample_8k,
                                 arrival_monotonic_ms,
                                 discontinuity != 0,
                                 std::vector<std::uint8_t>(ulaw, ulaw + count)};
    auto result = frontend->timeline.ingest(packet);
    frontend->output = std::move(result.pcm12k);
    collectWindows(*frontend);
    return static_cast<int>(frontend->output.size());
}

extern "C" int js8_audio_finish(Js8AudioFrontend *frontend) {
    if (!frontend)
        return -1;
    frontend->output = frontend->timeline.finish();
    collectWindows(*frontend);
    return static_cast<int>(frontend->output.size());
}

extern "C" std::size_t
js8_audio_copy_output(Js8AudioFrontend const *frontend, float *output,
                      std::size_t capacity) {
    if (!frontend || !output || capacity < frontend->output.size())
        return 0;
    std::copy(frontend->output.begin(), frontend->output.end(), output);
    return frontend->output.size();
}

extern "C" int js8_audio_snapshot(Js8AudioFrontend const *frontend,
                                   Js8AudioSnapshot *snapshot) {
    if (!frontend || !snapshot)
        return 0;
    auto const state = frontend->timeline.state();
    *snapshot = {state.expectedSample8k,
                 state.acceptedPackets,
                 state.duplicatePackets,
                 state.sequenceGaps,
                 state.discontinuities,
                 state.insertedGapSamples8k,
                 state.producedSamples12k,
                 state.maxArrivalJitterMs,
                 state.peak,
                 state.rms};
    return 1;
}

extern "C" int js8_audio_next_window(Js8AudioFrontend *frontend,
                                      Js8DecodeWindow *window) {
    if (!frontend || !window || frontend->windows.empty())
        return 0;
    auto const ready = frontend->windows.front();
    frontend->windows.pop_front();
    *window = {static_cast<int>(ready.submode), ready.slotUtcMs,
               ready.firstSample12k, ready.sampleCount};
    return 1;
}
