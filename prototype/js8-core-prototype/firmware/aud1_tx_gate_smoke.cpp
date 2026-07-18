// PROTOTYPE — paced capture/prebuffer/PTT/drain and fault acceptance story.

#include "aud1_tx_gate.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {
using namespace js8proto;
using namespace js8proto::firmware;

std::vector<std::uint8_t> packet(std::uint32_t sequence,
                                 std::uint64_t firstSample,
                                 std::uint32_t totalPackets,
                                 std::uint32_t streamId = 55,
                                 std::uint32_t txId = 7) {
    constexpr std::size_t count = 960;
    std::vector<std::uint8_t> pcm(count * 2);
    for (std::size_t index = 0; index < count; ++index) {
        auto const value = static_cast<std::int16_t>((firstSample + index) % 32767);
        pcm[index * 2] = static_cast<std::uint8_t>(value);
        pcm[index * 2 + 1] = static_cast<std::uint8_t>(value >> 8);
    }
    std::uint16_t flags = sequence == 0 ? Aud1First : 0;
    if (sequence + 1 == totalPackets) flags |= Aud1Last;
    return encodeAud1({Aud1Kind::TxPcm16, flags, streamId, sequence, 48000,
                       firstSample, txId}, pcm);
}

TxPrepare request() { return {55, 7, 1000, 48000, 50, 11520}; }
bool pttOff(Aud1TxGate const &gate) { return !gate.snapshot().ptt; }
} // namespace

int main() {
    bool pass = true;
    Aud1TxGate captureGate(24000, true);
    pass = pass && captureGate.prepare(request(), 0) && pttOff(captureGate);
    for (std::uint32_t sequence = 0; sequence < 50; ++sequence) {
        auto const at = 760 + sequence * 20;
        auto wire = packet(sequence, sequence * 960ULL, 50);
        pass = pass && captureGate.accept(wire, at);
        captureGate.tick(at);
        if (at < 1000) pass = pass && pttOff(captureGate);
    }
    captureGate.tick(2000);
    auto const completed = captureGate.snapshot();
    pass = pass && completed.state == TxGateState::Drained && !completed.ptt &&
           completed.receivedSamples == 48000 && completed.consumedSamples == 48000 &&
           captureGate.capture().size() == 48000;

    Aud1TxGate noPrebuffer;
    pass = pass && noPrebuffer.prepare(request(), 0);
    noPrebuffer.tick(1000);
    pass = pass && noPrebuffer.snapshot().state == TxGateState::Fault &&
           pttOff(noPrebuffer);

    // A packet already being handled at the slot boundary must count toward
    // the prebuffer before the deadline is evaluated.  The ESP32 WebSocket
    // loop can wake with this packet queued after a short scheduling delay.
    Aud1TxGate boundaryPacket;
    pass = pass && boundaryPacket.prepare(request(), 0);
    for (std::uint32_t sequence = 0; sequence < 11; ++sequence) {
        auto wire = packet(sequence, sequence * 960ULL, 50);
        pass = pass && boundaryPacket.accept(wire, 760 + sequence * 20);
    }
    auto boundaryWire = packet(11, 11 * 960ULL, 50);
    pass = pass && boundaryPacket.accept(boundaryWire, 1000) &&
           boundaryPacket.snapshot().state == TxGateState::Transmitting &&
           boundaryPacket.snapshot().ptt;

    Aud1TxGate discontinuity;
    pass = pass && discontinuity.prepare(request(), 0);
    auto wrong = packet(1, 960, 50);
    pass = pass && !discontinuity.accept(wrong, 760) &&
           discontinuity.snapshot().state == TxGateState::Fault &&
           pttOff(discontinuity);

    Aud1TxGate underrun;
    pass = pass && underrun.prepare(request(), 0);
    for (std::uint32_t sequence = 0; sequence < 12; ++sequence) {
        auto wire = packet(sequence, sequence * 960ULL, 50);
        pass = pass && underrun.accept(wire, 760 + sequence * 20);
    }
    underrun.tick(1000);
    pass = pass && underrun.snapshot().ptt;
    underrun.tick(1260);
    pass = pass && underrun.snapshot().state == TxGateState::Fault && pttOff(underrun);

    Aud1TxGate disconnect;
    pass = pass && disconnect.prepare(request(), 0);
    for (std::uint32_t sequence = 0; sequence < 13; ++sequence) {
        auto wire = packet(sequence, sequence * 960ULL, 50);
        pass = pass && disconnect.accept(wire, 760 + sequence * 20);
    }
    disconnect.tick(1000);
    pass = pass && disconnect.snapshot().ptt;
    disconnect.disconnect();
    pass = pass && disconnect.snapshot().state == TxGateState::Aborted && pttOff(disconnect);

    Aud1TxGate abort;
    pass = pass && abort.prepare(request(), 0);
    abort.abort("operator");
    pass = pass && abort.snapshot().state == TxGateState::Aborted && pttOff(abort);

    std::cout << "FIRMWARE TX GATE " << (pass ? "PASS" : "FAIL")
              << " captured=" << captureGate.capture().size()
              << " final=" << txGateStateName(completed.state)
              << " ptt=" << completed.ptt
              << " boundary=" << txGateStateName(boundaryPacket.snapshot().state)
              << " faults=prebuffer,continuity,underrun,disconnect,abort\n";
    return pass ? 0 : 1;
}
