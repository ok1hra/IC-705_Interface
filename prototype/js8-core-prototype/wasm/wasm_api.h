// PROTOTYPE — C ABI intended for the Emscripten boundary.
#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

struct Js8AudioFrontend;

struct Js8AudioSnapshot {
    std::uint64_t expected_sample_8k;
    std::uint64_t accepted_packets;
    std::uint64_t duplicate_packets;
    std::uint64_t sequence_gaps;
    std::uint64_t discontinuities;
    std::uint64_t inserted_gap_samples_8k;
    std::uint64_t produced_samples_12k;
    double max_arrival_jitter_ms;
    float peak;
    double rms;
};

struct Js8DecodeWindow {
    int submode;
    std::int64_t slot_utc_ms;
    std::uint64_t first_sample_12k;
    std::uint64_t sample_count;
};

int js8_proto_modulate_frame48k(const char* frame12, int type, int submode,
                                double base_frequency_hz, double amplitude,
                                std::int16_t* output, std::size_t capacity);

Js8AudioFrontend *js8_audio_create(std::int64_t anchor_utc_ms);
void js8_audio_destroy(Js8AudioFrontend *frontend);
void js8_audio_reset(Js8AudioFrontend *frontend, std::int64_t anchor_utc_ms);

int js8_audio_push_ulaw(Js8AudioFrontend *frontend, std::uint32_t sequence,
                        std::uint64_t first_sample_8k,
                        double arrival_monotonic_ms, int discontinuity,
                        std::uint8_t const *ulaw, std::size_t count);
int js8_audio_finish(Js8AudioFrontend *frontend);
std::size_t js8_audio_copy_output(Js8AudioFrontend const *frontend,
                                  float *output, std::size_t capacity);
int js8_audio_snapshot(Js8AudioFrontend const *frontend,
                       Js8AudioSnapshot *snapshot);
int js8_audio_next_window(Js8AudioFrontend *frontend,
                          Js8DecodeWindow *window);

}
