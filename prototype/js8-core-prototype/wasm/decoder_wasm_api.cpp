// PROTOTYPE — queued browser-facing API over the reference decoder callback.

#include "decoder_wasm_api.h"

#include "decoder_c_api.h"

#include <cstddef>
#include <deque>
#include <new>

static_assert(offsetof(Js8WasmDecodeEvent, data) == 16);
static_assert(offsetof(Js8WasmDecodeEvent, type) == 32);
static_assert(sizeof(Js8WasmDecodeEvent) == 44);

struct Js8WasmDecoder {
    std::deque<Js8WasmDecodeEvent> events;
};

namespace {
void collectEvent(Js8ReferenceDecode const *decoded, void *context) {
    auto &events = static_cast<Js8WasmDecoder *>(context)->events;
    Js8WasmDecodeEvent event{};
    event.utc = decoded->utc;
    event.snr = decoded->snr;
    event.dt = decoded->dt;
    event.frequency_hz = decoded->frequency_hz;
    for (std::size_t i = 0; i < sizeof(event.data); ++i)
        event.data[i] = decoded->data[i];
    event.type = decoded->type;
    event.quality = decoded->quality;
    event.submode = decoded->submode;
    events.push_back(event);
}
} // namespace

extern "C" Js8WasmDecoder *js8_wasm_decoder_create() {
    return new (std::nothrow) Js8WasmDecoder;
}

extern "C" void js8_wasm_decoder_destroy(Js8WasmDecoder *decoder) {
    delete decoder;
}

extern "C" int js8_wasm_decoder_run(
    Js8WasmDecoder *decoder, std::int16_t const *samples,
    std::size_t sample_count, int submode, int low_hz, int high_hz,
    int selected_hz) {
    if (!decoder)
        return -3;
    decoder->events.clear();
    return js8_reference_decode(samples, sample_count, submode, low_hz,
                                high_hz, selected_hz, collectEvent, decoder);
}

extern "C" int js8_wasm_decoder_next_event(Js8WasmDecoder *decoder,
                                             Js8WasmDecodeEvent *event) {
    if (!decoder || !event || decoder->events.empty())
        return 0;
    *event = decoder->events.front();
    decoder->events.pop_front();
    return 1;
}
