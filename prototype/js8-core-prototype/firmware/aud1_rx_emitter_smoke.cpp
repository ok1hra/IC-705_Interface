// PROTOTYPE — firmware AUD1 prepare/commit/drop/reconnect acceptance story.

#include "aud1_rx_emitter.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <string>

int main() {
    using namespace js8proto;
    using namespace js8proto::firmware;
    Aud1RxEmitter emitter;
    std::array<std::uint8_t, 160> ulaw{};
    ulaw.fill(0xff);

    bool pass = !emitter.prepare(ulaw).has_value() &&
                emitter.beginSession(0x01020304);
    auto const hello = emitter.helloJson();
    pass = pass && hello.find("\"protocol\":\"AUD1\"") != std::string::npos &&
           hello.find("\"streamId\":16909060") != std::string::npos;

    auto first = emitter.prepare(ulaw);
    auto firstDecoded = first ? decodeAud1(first->wire) : std::nullopt;
    pass = pass && firstDecoded && firstDecoded->header.sequence == 0 &&
           firstDecoded->header.firstSample == 0 &&
           firstDecoded->header.flags == Aud1First && emitter.commit(true);

    auto dropped = emitter.prepare(ulaw);
    pass = pass && dropped && emitter.commit(false) &&
           emitter.nextSequence() == 1 && emitter.nextSample() == 320 &&
           emitter.discontinuityPending();

    auto afterDrop = emitter.prepare(ulaw);
    auto dropDecoded = afterDrop ? decodeAud1(afterDrop->wire) : std::nullopt;
    pass = pass && dropDecoded && dropDecoded->header.sequence == 1 &&
           dropDecoded->header.firstSample == 320 &&
           dropDecoded->header.flags == Aud1Discontinuity &&
           emitter.commit(true);

    pass = pass && emitter.beginSession(0xa0b0c0d0);
    auto reconnect = emitter.prepare(ulaw);
    auto reconnectDecoded = reconnect ? decodeAud1(reconnect->wire) : std::nullopt;
    pass = pass && reconnectDecoded && reconnectDecoded->header.sequence == 0 &&
           reconnectDecoded->header.firstSample == 0 &&
           reconnectDecoded->header.flags == Aud1First;

    std::cout << "FIRMWARE AUD1 EMITTER " << (pass ? "PASS" : "FAIL")
              << " stream=" << emitter.streamId()
              << " sequence=" << emitter.nextSequence()
              << " firstSample=" << emitter.nextSample() << '\n';
    return pass ? 0 : 1;
}
