#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <type_traits>
#include <variant>

namespace JS8 {
namespace Costas {
enum class Type { ORIGINAL, MODIFIED };
using Array = std::array<std::array<int, 7>, 3>;

constexpr auto array = [] {
    constexpr auto costas =
        std::array{std::array{std::array{4, 2, 5, 6, 1, 3, 0},
                              std::array{4, 2, 5, 6, 1, 3, 0},
                              std::array{4, 2, 5, 6, 1, 3, 0}},
                   std::array{std::array{0, 6, 2, 3, 5, 4, 1},
                              std::array{1, 5, 0, 2, 3, 6, 4},
                              std::array{2, 5, 0, 6, 4, 1, 3}}};
    return [costas](Type type) -> Array const & {
        return costas[static_cast<std::underlying_type_t<Type>>(type)];
    };
}();
} // namespace Costas

void encode(int type, Costas::Array const &costas, char const *message,
            int *tones);

namespace Event {
struct DecodeStarted { int submodes; };
struct SyncStart { int position; int size; };
struct SyncState {
    enum class Type { CANDIDATE, DECODED } type;
    int mode;
    float frequency;
    float dt;
    union { int candidate; float decoded; } sync;
};
struct Decoded {
    int utc;
    int snr;
    float xdt;
    float frequency;
    std::string data;
    int type;
    float quality;
    int mode;
};
struct DecodeFinished { std::size_t decoded; };
using Variant =
    std::variant<DecodeStarted, SyncStart, SyncState, Decoded, DecodeFinished>;
using Emitter = std::function<void(Variant const &)>;
} // namespace Event
} // namespace JS8
