#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {
struct Js8ReferenceDecode {
    int utc;
    int snr;
    float dt;
    float frequency_hz;
    char data[13];
    int type;
    float quality;
    int submode;
};

using Js8ReferenceCallback = void (*)(Js8ReferenceDecode const *, void *);

int js8_reference_decode(std::int16_t const *samples, std::size_t sample_count,
                         int submode, int low_hz, int high_hz,
                         int selected_hz, Js8ReferenceCallback callback,
                         void *context);
}
