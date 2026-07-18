#include "decoder_c_api.h"

#include <algorithm>
#include <cstring>
#include <memory>

namespace {
template <typename Mode>
int run_reference_decoder(std::int16_t const *samples, std::size_t sample_count,
                          int low_hz, int high_hz, int selected_hz,
                          Js8ReferenceCallback callback, void *context) {
    if (!samples || sample_count == 0 || sample_count > Mode::NMAX)
        return -1;

    auto input = std::make_unique<struct dec_data>();
    std::copy_n(samples, sample_count, input->d2);
    input->params.nutc = 0;
    input->params.nfqso = selected_hz;
    input->params.newdat = true;
    input->params.nfa = low_hz;
    input->params.nfb = high_hz;
    input->params.syncStats = false;

    static DecodeMode<Mode> decoder;
    auto const count = decoder(
        *input, 0, static_cast<int>(sample_count),
        [callback, context](JS8::Event::Variant const &event) {
            auto const decoded = std::get_if<JS8::Event::Decoded>(&event);
            if (!decoded || !callback)
                return;

            Js8ReferenceDecode result{};
            result.utc = decoded->utc;
            result.snr = decoded->snr;
            result.dt = decoded->xdt;
            result.frequency_hz = decoded->frequency;
            auto const length = std::min<std::size_t>(decoded->data.size(), 12);
            std::memcpy(result.data, decoded->data.data(), length);
            result.data[length] = '\0';
            result.type = decoded->type;
            result.quality = decoded->quality;
            result.submode = decoded->mode;
            callback(&result, context);
        });
    return static_cast<int>(count);
}
} // namespace

extern "C" int js8_reference_decode(
    std::int16_t const *samples, std::size_t sample_count, int submode,
    int low_hz, int high_hz, int selected_hz,
    Js8ReferenceCallback callback, void *context) {
    switch (submode) {
    case ModeA::NSUBMODE:
        return run_reference_decoder<ModeA>(samples, sample_count, low_hz,
                                            high_hz, selected_hz, callback,
                                            context);
    case ModeB::NSUBMODE:
        return run_reference_decoder<ModeB>(samples, sample_count, low_hz,
                                            high_hz, selected_hz, callback,
                                            context);
    case ModeC::NSUBMODE:
        return run_reference_decoder<ModeC>(samples, sample_count, low_hz,
                                            high_hz, selected_hz, callback,
                                            context);
    case ModeE::NSUBMODE:
        return run_reference_decoder<ModeE>(samples, sample_count, low_hz,
                                            high_hz, selected_hz, callback,
                                            context);
    case ModeI::NSUBMODE:
        return run_reference_decoder<ModeI>(samples, sample_count, low_hz,
                                            high_hz, selected_hz, callback,
                                            context);
    default:
        return -2;
    }
}
