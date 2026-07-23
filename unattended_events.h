#pragma once

// Event record for the unattended-operation log, shared by the firmware sketch
// and the native regression tests.
//
// Decision 13 of docs/js8call-neobsluhovany-provoz-plan.md: no silent
// suppression. Every ban, drop, deferral and refusal has to leave a timestamped,
// explained trace, otherwise the station does things the operator cannot
// account for. A restriction that fires at 03:00 must still be findable at 09:00
// -- from a phone, after a page reload -- which is why the log lives in the
// firmware and not in the browser.
//
// The pure parts (classification, line formatting, rotation decision) live here
// so they can be exercised without a filesystem.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum UnattendedEventType : uint8_t {
  UEV_ARM = 0,        // operator armed unattended operation
  UEV_EXTEND,         // arming window restarted
  UEV_REVOKE,         // operator revoked, from anywhere on the network
  UEV_EXPIRE,         // arming lapsed on its own
  UEV_BLOCK,          // a transmission was refused
  UEV_PTT_SAFETY,     // reset/link-up found the radio possibly keyed
  UEV_TX_ABORT,       // transmission ended on a fault path
  UEV_TYPE_COUNT
};

// Keep the file bounded: events are rare (a few per hour), so plain append plus
// halving on overflow costs almost no flash wear and needs no index.
static const uint32_t UNATTENDED_LOG_MAX_BYTES  = 16384;
static const uint32_t UNATTENDED_LOG_KEEP_BYTES = 8192;
static const uint8_t  UNATTENDED_EVENT_LINE_MAX = 120;

inline const char *unattendedEventName(uint8_t type) {
  switch (type) {
    case UEV_ARM:        return "ARM";
    case UEV_EXTEND:     return "EXTEND";
    case UEV_REVOKE:     return "REVOKE";
    case UEV_EXPIRE:     return "EXPIRE";
    case UEV_BLOCK:      return "BLOCK";
    case UEV_PTT_SAFETY: return "PTT_SAFETY";
    case UEV_TX_ABORT:   return "TX_ABORT";
    default:             return "UNKNOWN";
  }
}

// Events the remote panel must highlight: they mean the station refused to do
// something, or recovered from an unsafe state.
inline bool unattendedEventIsAlert(uint8_t type) {
  return type == UEV_BLOCK || type == UEV_PTT_SAFETY || type == UEV_TX_ABORT;
}

// One line per event: uptime in ms, type, free-form detail. Uptime rather than
// wall time because the firmware has no RTC; the browser renders absolute times
// from its own clock and the reported uptime.
// Returns the number of characters written (excluding the terminator), or 0 on
// bad input.
inline size_t unattendedFormatEvent(char *out, size_t outSize, uint32_t upMs,
                                    uint8_t type, const char *detail) {
  if (!out || outSize < 24 || type >= UEV_TYPE_COUNT) return 0;
  char clean[64];
  size_t at = 0;
  if (detail) {
    for (size_t i = 0; detail[i] && at + 1 < sizeof(clean); ++i) {
      char c = detail[i];
      // Newlines would break the one-line-per-event contract; quotes and
      // backslashes would break the JSON the panel is served as.
      if (c == '\n' || c == '\r' || c == '"' || c == '\\') c = ' ';
      clean[at++] = c;
    }
  }
  clean[at] = '\0';
  int written = snprintf(out, outSize, "%lu %s %s\n", (unsigned long)upMs,
                         unattendedEventName(type), clean);
  if (written <= 0) return 0;
  return (size_t)written >= outSize ? outSize - 1 : (size_t)written;
}

// True when appending `addBytes` would push the log past its cap.
inline bool unattendedLogNeedsRotate(uint32_t currentBytes, uint32_t addBytes) {
  return currentBytes + addBytes > UNATTENDED_LOG_MAX_BYTES;
}

// Offset to start copying from when rotating, i.e. drop the oldest part and keep
// the newest UNATTENDED_LOG_KEEP_BYTES. Callers should then advance to the next
// line boundary so the first retained entry is not a fragment.
inline uint32_t unattendedLogRotateFrom(uint32_t currentBytes) {
  if (currentBytes <= UNATTENDED_LOG_KEEP_BYTES) return 0;
  return currentBytes - UNATTENDED_LOG_KEEP_BYTES;
}
