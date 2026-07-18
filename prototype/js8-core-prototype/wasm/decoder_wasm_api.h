// PROTOTYPE — queued browser-facing API over the reference decoder callback.
#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

struct Js8WasmDecoder;

struct Js8WasmDecodeEvent {
    int utc;
    int snr;
    float dt;
    float frequency_hz;
    char data[13];
    int type;
    float quality;
    int submode;
};

Js8WasmDecoder *js8_wasm_decoder_create();
void js8_wasm_decoder_destroy(Js8WasmDecoder *decoder);
int js8_wasm_decoder_run(Js8WasmDecoder *decoder,
                         std::int16_t const *samples,
                         std::size_t sample_count, int submode,
                         int low_hz, int high_hz, int selected_hz);
int js8_wasm_decoder_next_event(Js8WasmDecoder *decoder,
                                Js8WasmDecodeEvent *event);

}
