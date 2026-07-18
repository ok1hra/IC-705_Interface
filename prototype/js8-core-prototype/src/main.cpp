// PROTOTYPE — interactive shell for the isolated JS8 core candidate.

#include "js8_core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#if JS8_PROTO_HAVE_BOOST_CRC
#include <boost/crc.hpp>
#endif

namespace {

struct State {
    std::size_t modeIndex = 1;
    int type = 0;
    int toneOffsetHz = 1500;
    int clockCorrectionMs = 0;
    std::string frame = "000000000000";
    std::string message;
};

std::int64_t nowUtcMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string utcString(std::int64_t epochMs) {
    const std::time_t seconds = static_cast<std::time_t>(epochMs / 1000);
    std::tm utc{};
    gmtime_r(&seconds, &utc);
    std::ostringstream out;
    out << std::put_time(&utc, "%Y-%m-%d %H:%M:%S") << '.'
        << std::setfill('0') << std::setw(3) << (epochMs % 1000) << " UTC";
    return out.str();
}

std::string toneString(const std::array<std::uint8_t, 79>& tones) {
    std::ostringstream out;
    for (std::size_t i = 0; i < tones.size(); ++i) {
        if (i == 7 || i == 36 || i == 43 || i == 72) out << " |";
        out << ' ' << static_cast<int>(tones[i]);
    }
    return out.str();
}

std::string validateTones(const js8proto::EncodedFrame& encoded,
                          const js8proto::SubmodeSpec& mode) {
    const bool rangeOk = std::all_of(encoded.tones.begin(), encoded.tones.end(),
                                     [](auto tone) { return tone <= 7; });
    const std::array<std::size_t, 3> starts = {0, 36, 72};
    const std::array<std::array<int, 7>, 3> original = {{
        {4, 2, 5, 6, 1, 3, 0}, {4, 2, 5, 6, 1, 3, 0},
        {4, 2, 5, 6, 1, 3, 0}}};
    const std::array<std::array<int, 7>, 3> modified = {{
        {0, 6, 2, 3, 5, 4, 1}, {1, 5, 0, 2, 3, 6, 4},
        {2, 5, 0, 6, 4, 1, 3}}};
    const auto& expected = mode.costas == js8proto::Costas::Original
        ? original : modified;
    bool costasOk = true;
    for (std::size_t block = 0; block < starts.size(); ++block) {
        for (std::size_t i = 0; i < 7; ++i) {
            costasOk &= encoded.tones[starts[block] + i] == expected[block][i];
        }
    }
    return rangeOk && costasOk ? "PASS: tone range and Costas placement"
                               : "FAIL: structural tone invariant";
}

#if JS8_PROTO_HAVE_BOOST_CRC
std::uint16_t boostCrcForFrame(std::string_view frame, int type) {
    constexpr std::string_view alphabet =
        "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-+";
    std::array<std::uint8_t, 11> bytes{};
    for (int i = 0, j = 0; i < 12; i += 4, j += 3) {
        const auto a = alphabet.find(frame[i]);
        const auto b = alphabet.find(frame[i + 1]);
        const auto c = alphabet.find(frame[i + 2]);
        const auto d = alphabet.find(frame[i + 3]);
        const auto words = static_cast<std::uint32_t>(
            (a << 18) | (b << 12) | (c << 6) | d);
        bytes[j] = static_cast<std::uint8_t>(words >> 16);
        bytes[j + 1] = static_cast<std::uint8_t>(words >> 8);
        bytes[j + 2] = static_cast<std::uint8_t>(words);
    }
    bytes[9] = static_cast<std::uint8_t>((type & 7) << 5);
    return static_cast<std::uint16_t>(
        boost::augmented_crc<12, 0xc06>(bytes.data(), bytes.size()) ^ 42);
}
#endif

void render(const State& state, bool clear) {
    if (clear) std::cout << "\033[2J\033[H";
    const auto& mode = js8proto::submodes()[state.modeIndex];
    const auto now = nowUtcMs();
    const auto slot = js8proto::planNextSlot(
        now, mode, state.clockCorrectionMs);
    const auto encoded = js8proto::encodeFrame(state.frame, state.type, mode.id);

    std::cout << "\033[1mPROTOTYPE — isolated JS8 transport core\033[0m\n"
              << "\033[2mQuestion: can the portable core be separated before WASM/browser integration?\033[0m\n\n"
              << "\033[1mCurrent state\033[0m\n"
              << "UTC now:           " << utcString(now) << '\n'
              << "Submode:           " << mode.name << " (id "
              << static_cast<int>(mode.id) << ", decoder mask "
              << mode.decoderMask << ")\n"
              << "Period:            " << mode.periodMs << " ms\n"
              << "Samples/symbol:    " << mode.samplesPerSymbol12k << " @ 12 kHz\n"
              << "Tone spacing/BW:   " << mode.toneSpacingHz() << " / "
              << mode.bandwidthHz() << " Hz\n"
              << "Symbol duration:   " << mode.symbolDurationMs() << " ms\n"
              << "Start delay:       " << mode.startDelayMs << " ms\n"
              << "Clock correction:  " << state.clockCorrectionMs << " ms\n"
              << "Next TX slot:      " << utcString(slot.slotUtcMs) << " (in "
              << slot.waitMs << " ms)\n"
              << "TX tone/span:      " << state.toneOffsetHz << " .. "
              << state.toneOffsetHz + mode.bandwidthHz() << " Hz — "
              << (js8proto::validToneOffset(state.toneOffsetHz, 500, 2700, mode)
                      ? "valid" : "INVALID") << '\n'
              << "Transport frame:   " << state.frame << '\n'
              << "Frame type:        " << state.type << '\n';

    if (encoded) {
        std::cout << "CRC-12:            0x" << std::hex << std::setw(3)
                  << std::setfill('0') << encoded->crc12 << std::dec
                  << std::setfill(' ') << '\n';
#if JS8_PROTO_HAVE_BOOST_CRC
        const auto oracle = boostCrcForFrame(state.frame, state.type);
        std::cout << "CRC oracle:        "
                  << (oracle == encoded->crc12 ? "PASS" : "FAIL")
                  << " (Boost reference 0x" << std::hex << oracle << std::dec
                  << ")\n";
#else
        std::cout << "CRC oracle:        unavailable (boost/crc.hpp missing)\n";
#endif
        std::cout << "Structure:         " << validateTones(*encoded, mode)
                  << "\nTones (7|29|7|29|7):\n" << toneString(encoded->tones)
                  << "\n";
    } else {
        std::cout << "Encoder:           invalid frame/type/submode\n";
    }
    if (!state.message.empty()) std::cout << "\nLast action:        " << state.message << '\n';

    std::cout << "\n\033[1mCommands\033[0m\n"
              << "[m] next mode  [f] set 12-char frame  [t] frame type\n"
              << "[<]/[>] tone -/+10 Hz  [c] clock correction  [n] refresh  [q] quit\n"
              << "> " << std::flush;
}

bool readInteger(const char* prompt, int& value) {
    std::cout << prompt << std::flush;
    std::string input;
    if (!std::getline(std::cin, input)) return false;
    try {
        std::size_t used = 0;
        const int parsed = std::stoi(input, &used);
        if (used != input.size()) return false;
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

void runInteractive() {
    State state;
    while (true) {
        render(state, true);
        std::string input;
        if (!std::getline(std::cin, input)) break;
        if (input == "q") break;
        state.message.clear();
        if (input == "m") {
            state.modeIndex = (state.modeIndex + 1) % js8proto::submodes().size();
        } else if (input == "f") {
            std::cout << "Frame (exactly 12 JS8 alphabet characters): " << std::flush;
            std::string frame;
            if (std::getline(std::cin, frame) && frame.size() == 12 &&
                js8proto::encodeFrame(frame, state.type,
                    js8proto::submodes()[state.modeIndex].id)) {
                state.frame = frame;
            } else {
                state.message = "frame rejected";
            }
        } else if (input == "t") {
            int value = state.type;
            if (readInteger("Frame type (0..7): ", value) && value >= 0 && value <= 7)
                state.type = value;
            else state.message = "type rejected";
        } else if (input == "<") {
            state.toneOffsetHz -= 10;
        } else if (input == ">") {
            state.toneOffsetHz += 10;
        } else if (input == "c") {
            int value = state.clockCorrectionMs;
            if (readInteger("Clock correction in ms: ", value))
                state.clockCorrectionMs = value;
            else state.message = "clock correction rejected";
        } else if (input != "n") {
            state.message = "unknown command";
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string_view(argv[1]) == "--snapshot") {
        render(State{}, false);
        return 0;
    }
    runInteractive();
    return 0;
}
