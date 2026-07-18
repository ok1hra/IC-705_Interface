#include "aud1_tx_state.h"

#include <cstddef>
#include <iostream>

namespace {
std::size_t serviceDisconnected(Aud1TxState initial, bool keyed,
                                std::size_t loopCount) {
    auto state = initial;
    std::size_t aborts = 0;
    for (std::size_t loop = 0; loop < loopCount; ++loop) {
        if (!aud1TxNeedsDisconnectAbort(state, keyed)) continue;
        ++aborts;
        keyed = false;
        state = AUD1_TX_FAULT; // aud1TxAbort(reason) leaves the real gate in FAULT.
    }
    return aborts;
}
} // namespace

int main() {
    bool const pass =
        serviceDisconnected(AUD1_TX_IDLE, false, 100) == 0 &&
        serviceDisconnected(AUD1_TX_DRAINED, false, 100) == 0 &&
        serviceDisconnected(AUD1_TX_FAULT, false, 100) == 0 &&
        serviceDisconnected(AUD1_TX_READY, false, 100) == 1 &&
        serviceDisconnected(AUD1_TX_PREBUFFER, false, 100) == 1 &&
        serviceDisconnected(AUD1_TX_STREAM, false, 100) == 1 &&
        serviceDisconnected(AUD1_TX_IDLE, true, 100) == 1;
    std::cout << "AUD1 DISCONNECT " << (pass ? "PASS" : "FAIL")
              << " active_aborts="
              << serviceDisconnected(AUD1_TX_STREAM, false, 100)
              << " fault_aborts="
              << serviceDisconnected(AUD1_TX_FAULT, false, 100) << '\n';
    return pass ? 0 : 1;
}
