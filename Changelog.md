# Changelog

All notable changes to **IC-705_Interface**, grouped by firmware revision (`#define REV`) and
broken down per commit.

Newest first. Dates are local (CEST). A firmware revision is the `REV` value flashed with that
commit — several commits usually share one revision, and the revision is bumped when a build is
published.

---

## Working tree — not committed

* **Neither IP address has to be typed from memory any more — the radio's is scanned for, and the
  interface's is handed over.** Two addresses stood between a box on the bench and a working
  system, and both were the operator's problem.
  **Finding the radio:** the Icom LAN protocol has no discovery, and neither does wfview — its
  `discoveredRigID()` identifies the rig model over a link that is already open, and there is no
  broadcast probe anywhere in the protocol. So `icom_lan_discovery.h` uses the only unauthenticated
  primitive the handshake offers: `AreYouThere (0x03)` to UDP 50001 on every host of the local /24,
  collecting `IAmHere (0x04)`. **It stops there and never logs in**, so it cannot consume the
  IC-705's single session — a scan will not lock out a wfview that is connected at the time. It
  does have to borrow UDP 50001 from the live client, and the reason it *stops* that client rather
  than opening a second socket is that `WiFiUDP` sets `SO_REUSEADDR`: the duplicate bind would
  succeed and then silently eat the client's control packets instead of failing. 254 datagrams are
  paced 8 per loop pass so the audio-carrying loop is not disturbed, and the scan is refused while
  transmitting. **Test connection** does a real login and separates "radio refused the credentials"
  from "nothing answered" — it declares success at `LAN_STREAM`, before the CI-V channel opens, so
  a radio being probed never writes its frequency into the shared rig state. The result list is
  labelled *answered on UDP 50001*, not *IC-705*: a wfview or RS-BA1 server answers the same probe
  and telling them apart would require the login the scan refuses to do. Wire primitives moved to
  `icom_lan_wire.h` so scanner and client share one definition of the packet layout.
  **Finding the interface:** the AP-mode "tap to open" prompt provably cannot be reproduced in
  station mode — that sheet is the client OS reacting to its connectivity probe, and it only works
  in AP mode because the device *is* the DHCP server and answers DNS for every name; on the home
  network the router owns both, and faking it would mean answering queries addressed to the router.
  What replaces it is a name and a handover. `WiFi.setHostname("ic705")` was missing entirely, so
  the router listed a generic `espressif` entry and could not publish a name; with it, `http://ic705/`
  resolves through ordinary unicast DNS on most consumer routers — the one path that also works from
  **Android**. Three separate faults explain why `ic705.local` "worked sometimes": modem power save
  was left at its default so the AP only delivered the multicast queries at DTIM beacons and dropped
  them (`WiFi.setSleep(false)`, at the cost of steady-state current), `_http._tcp` was registered in
  AP mode only so nothing could browse for the device on the real LAN, and the responder was never
  re-registered after a reconnect — `NetworkIdentityLoop()` now does it on the same edge TrxNet
  already used, kept out of `TrxNetLoop()` because that returns early when TrxNet is disabled. A
  failed `MDNS.begin()` also no longer parks the device in `while(1)` forever.
  **The handover:** saving WiFi in AP mode used to end in a blind restart that took the portal away
  and left no address behind. `/setup/wifi-try` now raises the station beside the still-running
  softAP and the portal shows the address the router handed out, as a link and as a QR code
  (`data/qrcode.min.js` restored from the old filesystem layout, loaded on demand so it costs
  nothing on normal page loads), before the operator presses restart. The address is kept in EEPROM
  132-135 — written only when it changes, because that region is a flash sector — so any later AP
  visit, including the unintended kind when the configured WiFi is out of range at boot, shows
  *"Last address on your network"*. The softAP follows the station onto its channel during the
  handover, so clients briefly drop and re-associate; the page treats a failed poll as "still
  connecting". **Both WiFi profiles are tried, and the target is chosen by scan.** The first cut of
  the handover tried only `WifiProfileConfigured(0) ? 0 : 1`, so an unreachable SSID 1 reported
  failure while SSID 2 was working — caught on the bench. `/setup/wifi-try` now matches a scan
  against both configured profiles through the new `collectVisibleWifiProfiles()` and walks them
  strongest-RSSI first with a targeted channel+BSSID `begin()` — no reason to re-sweep every
  channel for an AP we just heard. The **second** attempt at that was also wrong: an async scan
  started from the request handler right after `WiFi.mode(WIFI_AP_STA)` returns
  `WIFI_SCAN_FAILED` immediately, because the station interface has not come up yet, and the
  handover reported `not_found` without ever trying to connect (`WiFi.status()` was logging 255,
  WL_NO_SHIELD, which was the tell). The scan now runs from the main loop after a 400 ms settle
  and uses the same blocking call the boot path has always used successfully; it stalls the loop
  for a second or two, which during setup costs nothing because there is no radio link and no
  audio. An empty scan is no longer treated as proof of absence either — it may have failed, and
  hidden SSIDs never appear in one — so every configured profile is attempted anyway, blind,
  passing no BSSID rather than the all-zero one a blind candidate carries. Failure is reported
  only after all attempts, as `not_found` (a scan ran, saw nothing of ours, attempts failed too)
  or `no_connect` (the network was there but refused us). The same helper replaced the boot path's
  any-visible check in `ConnectWiFiAlternating()`, which started at profile 0 regardless of what
  the scan had just seen and burned a 20 s timeout before alternating. Six checks added to `tools/data-browser-smoke.js` (scan lists a hit, the row
  click fills the field, the credential verdict renders, the last-known hint appears in AP mode and
  stays hidden otherwise, and the handover screen reaches an address), harness timeout raised
  45→55 s to fit them. Firmware 989 833 → 993 365 B, filesystem image 1 535 320 B. Documented in
  `docs/find-device-ip.md`. **Not yet verified on a radio or a real network** — in particular
  whether the radio answers a probe from an ephemeral port, which would let the scan run without
  dropping the link at all.
* **Brand mark at the head of every menu, and an About window behind it.** The RemoteQTH icon sits
  as the first item of `nav.tabs` on all six pages that carry the bar (DATA, WSPR, QRPLog, SETUP,
  BD, LOGSYNC), 26 px tall with `padding:0` so it stays inside the row height the text tabs already
  set — the bar does not grow, not even at the 33 px mobile tab size. It is **inline SVG**, not a
  file: a separate `/logo.svg` would have cost a firmware MIME entry, an `isStatic` flag, two build
  scripts and one more HTTP request per page, and an external SVG in `<img>` cannot inherit
  `currentColor`. Carrying `class="tab"` and `fill="currentColor"`, the mark takes each page's own
  muted colour instead of a hard-coded grey, so it cannot disappear if a page's palette changes.
  The path was reduced from 3307 to 1648 characters by converting to absolute coordinates, rounding
  to two decimals — which bounds the error instead of accumulating it along the path — and baking
  the Inkscape layer transform into the coordinates; the result differs from the original by 0.67 %
  RMSE, invisible at 15× the size it is drawn. Total cost **+6334 B** of the LittleFS image
  (819 kB still free). Clicking it opens `data/about.html` in a browser pop-up the same way the DXC
  tab does, showing **WIFILT** over *Web interface for Icom LAN transceivers*, the whole block a
  link to the GitHub repository in a new tab. The anchor also carries `target="_blank"`, which the
  DXC tab does not: without JavaScript the fallback navigation would otherwise leave the page and
  drop the radio session. Checks added to `tools/data-browser-smoke.js` (DATA and SETUP) and
  `tools/wspr-browser-smoke.js`; they assert the link target, the 26 px height, that the mark is no
  taller than a text tab, and that `currentColor` resolves to the same colour the tabs use.
* **Design notes in `docs/`** (22 files untracked), including `wspr-page-redesign.md`,
  `wspr-timetable-redesign.md`, `wspr-majak-implementace.md`, `wspr-band-rotation-plan.md`,
  `aprsis-cmd.md`, `aprsis-implementace.md`, `js8-tx-resend-plan.md`,
  `data-menu-wspr-subnav-plan.md`, `wake-lock.md`, `js8lan-hearing-links.md`,
  `setup-interfaces-architecture.md`, `icom-lan-implementace.md`, the `js8call-*` guides and
  `docs/agents/`.
* **`mercury/`** — Rhizomatica Mercury v2 evaluated as a second file-transfer modem beside JS8 and
  WSPR; the WASM build exists and passes a loopback test (~230 kB Brotli). Airtime, not flash, is the
  limiting factor. Notes in `docs/mercury-implementace.md`.
* Most of the `prototype/js8-core-prototype/` smoke harness and the newer `tools/*-smoke.*` scripts,
  plus `backups/` and `AGENTS.md`.

---

## REV 20260731 — 2026-08-01

### `023f241` @APRSIS, PWR preset, Resend

* **RESEND on failed transmissions, plus one armed automatic retry.** A row in the feed that did
  not make it to the air carries a `RESEND` button, and `RESEND` transmits — it does not merely put
  the text back in the field. Row ids are carried through the state snapshot so a rebuild of the
  feed cannot detach the button from what it resends. An **operator abort earns no RESEND**: it is
  the one failure the operator caused on purpose. Beyond the manual button, a failed slot arms
  exactly one automatic retry; a retry that runs out of time says so in the row rather than
  disappearing quietly. Found while building it: `CQ` reset the full interval on every call,
  `drainTxQueue` skipped its own `txBlockReasons`, and the outgoing log leaked across band changes.
  Plan in `docs/js8-tx-resend-plan.md`.
* **The display keeper became a dot in the shared topbar**, next to the firmware version, with the
  whole explanation in its tip — it is the only place those words live, so the tip doubles as the
  title and as the accessible name. The pop-up no longer mentions HTTPS: nobody reading it can put
  TLS in front of the firmware, so the advice was noise. A tap-opened tip can be dismissed by
  tapping away from it.
* **JS8CALL-ICOM sets the TRX power too, and heartbeats moved to 60 minutes.** The same machinery
  as WSPR, with two deliberate differences. The unit is **percent**, not the WSPR dBm grid: JS8
  announces no power in the protocol, so nothing pins it to that grid, and on a 100 W radio the
  grid would offer 10, 20, 50 and 100 W with nothing between 20 and 50. Percent is the radio's own
  display unit and its real resolution, so every value that can be typed is one the radio can be
  set to — and unlike the WSPR menu it needs no model table, so it still works on a radio the table
  does not recognise. Watts are shown beside the field when the model is known. The value is JS8's
  own (`Js8Settings.modems.js8call.rfPercent`), not shared with WSPR: that page caps at 10 W
  because it is a beacon, while 50 W on JS8 is ordinary, and one shared number would either export
  the cap or leave the beacon refusing to start.

  It is written on page open and after the radio's link returns, with the same three guards (knob
  wins, never mid-transmission, a failed `/state` fetch is not a reconnect). Two things differ,
  both because **this write can raise power** where WSPR's automatic value is always the minimum:
  only a level the operator set themselves is ever applied — a QSO mode has no safe value to invent
  — and the automatic write requires **Enable radio TX**, the one place they said the antenna is
  fine. Pressing SET needs no pledge, same as WSPR. The header power bar carries the "radio is not
  where the setting says" state, because the SETTINGS panel opens collapsed.

  Heartbeat interval default moved from 15 to 60 minutes in all four places that held it, plus a
  schema **v8→v9 migration that rewrites stored profiles**: a saved 15 cannot be told apart from
  v8's own default, so a default-only change would never reach anyone who had already opened the
  page. One selection in the menu undoes it.

  Found by the tests: the power field was rewritten from the target on every render whenever it was
  not focused, so a number typed and then left on the way to the SET button beside it was thrown
  away. Now held in an edit draft, the same shape `txGain` two settings below already uses.
  `powerCommand()` gave up `percentToLevel()` and `civLevelCommand()` so both pages share one CI-V
  encoder. Details in `docs/wspr-majak-implementace.md` ch. 24; eleven new checks in the DATA
  browser smoke (fixture gained `/commands`, `/setRfPower`, `/setConnected` and a real 14 0A
  decode) and seven in the JS8 settings smoke.

* **WSPR power menu follows the radio, and the page now applies it.** The dropdown offered every
  legal WSPR level under 10 W regardless of the transmitter, so an IC-705 was offered 17 dBm
  (50 mW = **0,5 %** of its scale) — a level the radio cannot be set to, because its smallest step
  is one percent. The list now starts at that step: seven entries on a 10 W radio, four on a 100 W
  one (1 W…10 W), three on a 200 W one. Each line names its percent (`30 dBm · 1 W · 1 %`), which
  is both the unit the radio's own display uses and the explanation for the shorter list. The floor
  is decided on the CI-V level rather than on watts: 33 dBm is 1,995 W, five thousandths *below*
  one percent of a 200 W radio, so an honest watts comparison would discard exactly the level that
  radio's smallest step produces. An unknown model empties and locks the menu instead of offering
  levels it cannot convert.

  The page also stopped waiting for SET: it writes the target on load and after the radio's link
  returns, so an unattended beacon keys at the level left in the menu rather than at whatever the
  radio remembered after a power cycle. This deliberately reverses an invariant the file defended
  in three places, so three rules bound it — the knob wins (the page remembers the percent it wrote
  and confirmed, and a different reading while the link is up stands the automation down until the
  next SET), a transmission is never interrupted (LAN drops here happen under audio load, i.e.
  mid-carrier, so the write waits for the PTT), and a failed `/state` fetch is not a reconnect
  (that is a WiFi flutter the radio knew nothing about). A stored choice the connected radio cannot
  produce is not honoured and not erased either, so swapping radios back restores it.

  Found on the way: the `±2` tolerance both agreement checks used is 0,78 % of a scale whose step
  is one percent, so it called 1 % and 2 % the same reading — 3 dB, and *the* most likely operating
  point after this change. Both now compare whole percent exactly, as does the knob detector, which
  would otherwise have fired on the radio's own rounding and switched the automation off by itself.
  Details in `docs/wspr-majak-implementace.md` ch. 23; sixteen new checks in the WSPR browser smoke
  (with `/setConnected` and `/setDialFrequency` fixture endpoints), and the schedule rotation in
  that harness is now re-armed before START instead of assuming the intervening checks fit inside
  one two-minute frame.

* **A dial off the presets is marked on both pages, and refused on WSPR.** The frequency button in
  the radio bar turns red whenever the TRX is not on one of the frequencies the pop-up offers — the
  other half of the answer the menu already gives by highlighting the matching preset, for when the
  menu is closed. On WSPR that also disables START, with the reason printed under the buttons and
  repeated in the menu footer: arming on 14.200 would look fine for ten minutes and then fail its
  first slot ten seconds before it keyed. TUNE stays available, since setting the drive level on
  whatever the radio is on is a legitimate thing to be doing at that moment, and a beacon already
  running is exempt — there the schedule tunes the radio itself before every slot, so a hand-turned
  VFO is something it corrects rather than something it stops for. WSPR keeps its ±500 Hz dial
  tolerance; JS8CALL tests the presets exactly, the same comparison that draws the highlight, so
  the button and the menu can never disagree. Seven new checks in the WSPR browser smoke (with a
  `/setDialFrequency` fixture endpoint for turning the VFO behind the page's back) and one in the
  DATA browser smoke.

* **@APRSIS command builder in the JS8CALL composer.** The `▾` menu beside the message field
  gained an `@APRSIS` entry; picking it turns the menu into the APRS-IS catalogue — `GRID` and
  `CMD`, and under `CMD` the eight destinations from `docs/aprsis-cmd.md` (SMSGTE, EMAIL-2,
  WLNK-1, APRS2SOTA, APRS2POTA, WHO-IS, WXBOT, plus a free direct callsign that remembers the
  last five). A breadcrumb walks back out. Parameters are never typed into the field as
  `{placeholders}`: each destination opens a small form with the callsign, locator and dial
  frequency already filled in from the station settings and the radio, a live preview of the exact
  radio payload, and the cost in characters, frames and seconds. The nine-character APRS addressee
  padding is computed, never typed, and recomputed before transmission — a hand-edited
  `:OK1ABC:` becomes `:OK1ABC   :` and an over-long destination is truncated at nine, matching
  `APRSISClient.cpp`. Over 67 characters of APRS text the send is refused (the gateway would
  truncate it anyway, after up to two minutes of airtime); over six frames it is only warned
  about. A half-built command cannot be transmitted at all. The group call lives in the Message
  field, never in Recipient, so an APRS spot mid-QSO leaves the selected station, its chat thread
  and its LOG QSO button untouched — the command goes to the recent-traffic feed like CQ and HB.
  Sending needs no recipient at all. Replies come back from an IGate addressed to the group rather
  than to us, so they are now recognised, kept under the MYCALL filter and marked `APRS` in the
  feed. New `data/js8-aprs.js` (catalogue, parser, padding, validation; no DOM) and
  `tools/js8-aprs-smoke.js`; sixteen new checks in the DATA browser smoke.

* **Two traps fixed while building it.** Rebuilding the preset menu through `innerHTML` detaches
  the button that was just clicked, so the global close-the-menu handler walked an orphaned
  subtree, failed to find `.message-field` and closed the menu mid-use; it now reads
  `event.composedPath()`, which is captured at dispatch. And `tools/data-browser-smoke.js` served
  its test page as `text/html` with no `charset`, so Chrome decoded the inline checks as
  windows-1252 and any literal compared against non-ASCII page text (the `·` separators the UI is
  full of) silently never matched. Escape inside a modal no longer aborts a transmission in
  progress.

---

## REV 20260730 — 2026-07-30 … 2026-07-31

### `61d6749` fix prebuffered missed slot, now 3s before

* **The TX guard window grew from 1.3 s to 3 s**, ahead of the browser's own 1.35 s stream lead.
  Besides keeping the cooperative loop off blocking work (port-80 handlers, DXC connect) around the
  key instant, the wider window gives the firmware somewhere to push scheduling opportunities to a
  **backgrounded page whose JavaScript timers have been throttled** — a hidden Chrome tab can have
  chained timers serviced as rarely as once a minute, which is what made a prebuffered WSPR slot
  miss its frame.
* While a key is imminent the firmware now emits `tx-level` status about five times a second over
  the audio WebSocket, so the WSPR pump is driven by inbound traffic instead of by the browser's
  clock. JS8 shares the socket and ignores the message.

### `81534ad` redesign WSPR time table

* **WSPR timetable simplified to ordered sequence changes.** The band × 48-half-hour matrix and
  variable/randomised period were replaced by a short 24-hour UTC list such as `08:30 20→15→10`,
  `20:00 160→80→40`. Each sequence runs until the next half-hour change, preserves the operator's
  order and wraps through midnight. The scheduler has a fixed six-minute minimum per band; one or
  two bands automatically leave unused frame positions. Existing v1–v3 schedules migrate to the
  new v4 shape. Back-to-back retuning now starts immediately on `tx-drained`, polls CAT readback at
  100 ms and overlaps the 300 ms band-relay settle interval with frequency confirmation.

### `dfc4d5b` redesign all network stream to one RealtimeAudioPump

* **One realtime audio pump for every network stream.** The TX-audio path was pulled out of
  `icomLanClient.h` into a shared `icom_lan_audio_tx.h` used by both WSPR and JS8, allocated only
  for the single LAN slot that owns audio, and driven on ESP32 by a dedicated task that owns the
  whole channel (a wedged socket task can no longer be reused by a reconnect).
* **CI-V commands got a priority queue.** Control and safety traffic — above all PTT — can evict
  the oldest strictly lower-priority entry instead of queueing behind a stale meter or frequency
  request that would hold PTT ON/OFF for half a second. PTT is treated as level state rather than
  an event stream, so an older queued PTT body is dropped instead of replayed, and a failed
  submission assumes the radio may still be keyed.
* During browser TX only `/state` and safety metering are polled; frequency and the rest stand down,
  which keeps lightweight session heartbeats alive through a long WSPR carrier. The audio-channel
  epoch (both sequence spaces) is reset as a unit, and a legacy fallback covers radios that do not
  answer the newer query.
* Firmware health smoke extended (`icom_lan_client_health_smoke.cpp`, +86).

### `02ca83b` wspr time table, js8 pwr bar

* **RF power in the JS8Call header.** A ten-segment vertical bar sits after the mode, the height of
  the TIMETABLE button, one segment per 10 % of the radio's own 0–255 CI-V power scale — so the
  count of lit segments reads back as the percentage. Beside it, that percentage against the
  transmitter's full scale in watts, resolved through the same cascade the WSPR page uses (manual
  model override first, then the model the radio reports), so the two pages can never quote
  different watts for one radio. An unrecognised model leaves the bar lit and the watts at `--`:
  percent belongs to the level alone. On phones the number gives way and the bar stays.
* **`/state` says whether the RF power level was ever read.** `rfPower` starts life as a
  fabrication — 205 on TRX1, 0 in the LAN snapshot — and until the radio answered `14 0A` there was
  no way to tell that from a reading. The new `rfPowerSeen` flag makes the difference visible, and
  the WSPR page now refuses to derive watts, a dBm level or a power mismatch from the default: it
  had been reporting a confident and entirely invented `8.00 W` for a radio that had not yet spoken.

* **WSPR time table rotates bands.** A half hour may now hold several bands, which take turns frame
  by frame: with three or more the radio keys continuously, never twice running on the same band,
  and every band still rests its `periodFrames`. The schedule became one matrix, band × half hour,
  edited by rows — chips pick the band, the 24×2 grid paints its hours, a cell counts the bands
  sharing it. `periodFrames` therefore changed meaning, from "how often the station transmits" to
  "the shortest gap on one band", and the footer derives the old figure from the busiest half hour.
  Settings migrate v2 → v3 (`slots` → `rotation` rows) with no loss.
  * **This exposed a fault in the shipping schedule.** The phase was drawn per half hour, so a band
    held across two of them could key two minutes after itself: measured over 30 days, **581 of
    4 320 transmissions (13 %)** on a single-band day at every 5th frame, worst case one frame
    apart — plus one violation per midnight whenever 720 frames is not a multiple of the period.
    The phase is now drawn per *run of half hours with the same set of bands*, and a look-back lock
    catches what is left. Over the same window that removes 2 399 violations for 73 lost frames.
  * **Band changes now fit the gap between frames.** A WSPR signal is 110,592 s of a 120 s frame, so
    a rotation has 9 s to retune: the beacon confirms the polled PTT flag instead of trusting it,
    hands `WsprTx` a 4 s lead instead of 10 s for those frames, allows 300 ms for the band-decoder
    relays, and **refuses to key on a dial it has not confirmed** by T−2 s. Three late changes in a
    row and the schedule leaves a frame free after each band change until one fits again — declared
    in the schedule itself, so the preview shows the reduced pace rather than promising frames the
    beacon has already given up on. The measured band-change time is shown in TX SESSION.
  * Activity gained band chips: with several bands in a cell the worst status wins its colour, so
    one broken 10 m frame would paint an hour in which 40 m and 30 m were perfect. A band with no
    TUNE reference at the current power is marked amber in the rotation — it still transmits, but
    its forward power is never checked.
  * Fixed alongside: changing the gap or `randomise` did not redraw the predicted tail in Activity,
    so it kept showing the previous setting until something unrelated rebuilt the grid.
* **EMAIL removed from `TX SESSION → Mode`.** Only the `<option>` is gone — the gateway composer,
  both dialogs and `js8-email.js` still ship and stay covered by `tools/data-browser-smoke.js`
  (`emailReady`), which now opens the form through a test hook. Putting the option back re-enables
  the mode.
* **JS8 AUTO countdown starts by itself again.** The arming window lives only in ESP RAM, while the
  AUTO switch is a browser setting that outlives both the tab and the radio, so after a restart the
  pill read `AUTO` with no countdown until the operator toggled the switch off and on. `data.js` now
  re-arms on a page load and on a firmware restart (seen as `upMs` dropping in `GET /unattended`);
  a window that lapsed on its own or was revoked from another device is left alone, so a forgotten
  tab still switches itself off and a remote revoke still sticks. The `armed, N min left` readout in
  SETTINGS is derived from the polled deadline instead of the last POST, so it also shows a window
  armed before this page loaded. Covered by `tools/data-browser-smoke.js`
  (`unattendedRearmAfterRestart`).

---

## REV 20260729 — 2026-07-30

### `eaf6336` some fix

WSPR beacon polish — the page's second pass after the first on-air use.

* **Time table redesigned.** The 24-hour schedule became a compact 4×6 grid of half-hour slots with
  a per-slot popover, apply-to-range filling and a single-step **UNDO** for every write wider than
  one slot (Clear included); closing the panel drops the undo snapshot with it.
* **Planned-frame preview** — the panel shows which frames actually key inside the selected window
  (frames, not half hours), and keeps showing a plan even when the beacon cannot currently carry it
  out.
* `every frame` and `every 2 frames` removed; schedules now start at *every 3rd frame*, so a
  frequency cannot be overloaded by accident.
* **`randomise` explained in place** — deterministic but different every half hour, described in the
  panel footer instead of yet another popup.
* **Bug fixed: signed XOR in `frameOffset`** silently dropped roughly 40 % of scheduled half hours.
* **Power model.** The beacon now opens at 1 % of the transmitter instead of a fixed dBm, keeps a
  per-band TUNE reference and compares it against the radio on the raw 0–255 CI-V level; a mismatch
  raises a non-blocking amber warning (`targetDbm`, `powerMismatch`, `referenceFor`,
  `storeReference`, `fullPowerWatts`).
* **Setup help offered automatically** when the radio is *both* outside a data mode *and* off any
  WSPR dial frequency — either one alone is legitimate, so both conditions must hold.
* TRX button carries the slot number and the frequency is grouped in threes, matching the JS8Call
  page.
* **SWR is shown only while transmitting** (nothing is measured on RX); both meters are polled only
  while the radio is keyed.
* Running clock next to `START/STOP`; during `TUNE` it counts down the watchdog that will end the
  tune.
* Collapsed-section summaries updated so a folded header cannot show stale state; the `TX buffer
  underrun` notice now clears on the next good transmission and the slot thermometer empties on the
  RX transition.
* Activity panel: future slots drawn dark grey instead of bright outlines, and tooltips explain the
  power meter, TUNE reference and ring values.
* REV bumped to 20260729 and the build republished to the USB-C web flasher.

### `e2f1f6e` changelog

* Documentation only.

---

## REV 20260727 — 2026-07-28

### `9d65d8f` WSPR, ICOM-LAN for all TRX1-3, WakeLock on Android os, Add RX lines to map

34 files, ~4 000 insertions. Design notes: `docs/wspr-majak-implementace.md`,
`docs/data-menu-wspr-subnav-plan.md`, `docs/wake-lock.md`, `docs/js8lan-hearing-links.md`.

* **New WSPR beacon page** (`wspr.html`, `wspr.css`, `wspr.js`) with a browser-side encoder
  (`wspr-core.js`: callsign/locator/power packing, convolutional encoder, interleaving), a transmit
  path (`wspr-tx.js`) that emits AUD1 v1 kind-3 frames byte-for-byte as `aud1AcceptTxPacket` expects,
  and its own activity log in a separate IndexedDB database (`wspr-log.js`) so WSPR never mixes into
  the QSO log.
* TX flow control is credit-based off the firmware's TX audio ring (`AUD1_TX_RING_SIZE` = 12 288 B of
  8 kHz µ-law ≈ 1.536 s) instead of browser timers, with a keepalive ping as insurance against an
  idle session. The locator field accepts a locator or a coordinate pair; six characters are kept but
  only four are transmitted. The GPS idea was dropped — the beacon has to work offline.
* **DATA menu with a sub-navigation.** JS8LAN and WSPR became sub-pages of `DATA` (later shown in the
  menu as `JS8Call` and `WSPR-Beacon`), sharing `data/lan-gate.js` — which owns the "radio is not in
  LAN mode" gate card — and `data/spectrum.js`, the waterfall lifted verbatim out of `data.js` so
  both pages render identically.
* **ICOM-LAN can now be assigned to any of TRX1/2/3.** LAN is no longer wired to slot 0: the sketch
  tracks which slot owns the LAN radio and routes through `lanCivFrameRoute()`,
  `lanRadioCivSnapshot()` and `lanRadioAudioService()`. JS8/WSPR audio, PTT and `/state` follow that
  slot, a manual reconnect targets the LAN radio wherever it sits, and a LAN radio outside slot 0
  polls the same rich CI-V schedule as TRX1. `USB` and `USB-D` are now distinguishable by JS8.
* SETUP option renamed `LAN` → **`ICOM-LAN`** (it is an Icom-only protocol); the frequency selector
  header shows the actual TRX number. TrxNet slots deliberately expose only telemetry, frequency and
  mode — `docs/trx-http-api.md` documents the HTTP contract for TRX2/TRX3 adapters.
* **Wake Lock** (`data/wake-lock.js`) keeps the display alive on both data pages: Screen Wake Lock
  API first, a muted looping inline video as fallback (12 440 B of base64, 5 235 B gzipped), and an
  honest failure message when neither works. iOS stays unfixable without TLS.
* **RX lines on the station map** — green "who hears whom" arrows derived from decoded HEARING
  traffic, behind a `LINKS` toggle. Only paths with both ends on the map are drawn, stations that
  were merely heard *about* get a hollow ring and no signal figures, and blocked entities stay hidden
  everywhere.
* Waterfall gained UTC slot boundary lines and a TX marker, and resets the analyser during TX because
  the decoder is deaf while transmitting.
* `tools/upload-firmware-spiffs.sh` reports program and SPIFFS usage prominently after each upload;
  `data/THIRD-PARTY-NOTICES.txt` added; `tools/data-browser-smoke.js` extended for the new pages.

---

## REV 20260725 — 2026-07-25

### `a19c0ef` TRX setup and small changes

* **SETUP / Radio redesign after the Bluetooth removal.** All three radios (TRX1/2/3) now use the
  same shape: label + a `Connection` dropdown with `LAN / TrxNet / CI-V`, and only the fields
  relevant to the selected interface are shown. TRX1 keeps its fixed primary role.
  Design note: `docs/setup-interfaces-architecture.md`.
* **LAN is exclusive to a single TRX.** Choosing LAN on one radio disables the option on the other
  two in the UI, with a validation backstop on save. Firmware still carries multi-LAN plumbing —
  deferred follow-up.
* **New radio transport abstraction** (`radio_transport.h`, ~978 changed lines in the sketch) plus
  contract smoke tests: `tools/setup-radio-contract-smoke.js`, `tools/radio-transport-smoke.cpp`,
  `tools/js8-session-browser-smoke.js`.
* **JS8LAN settings header.** The `AUTO` icon shows the remaining time until deactivation as
  `hh:mm`; the heartbeat icon shows time to the next HB and reflects the adaptive interval
  extension driven by band activity.
* **`Enable radio TX` is now a real master switch** — turning it off disables every service that
  needs TX and greys out the affected icons in the settings header.
* **JS8LAN top navigation** behaves like the other pages: `QRPLog`, `SETUP`, `LOGSYNC` open in a new
  tab and `DXC` in a new window, so leaving JS8LAN no longer drops the session.
* Icom LAN client and the AUD1 WebSocket session updated to match the new transport layer
  (`icomLanClient.h`, `data/js8-aud1.js`, `prototype/.../aud1_websocket_session.js`), with the
  browser and audio-source smoke tests extended accordingly.
* Firmware rebuilt and republished to the USB-C web flasher (`build/gh-pages/`, firmware.bin
  970 272 → 978 864 B).
* **This changelog added** (`Changelog.md`).
* **Known state:** the wider browser smoke suite still reports 5 red checks that were already red at
  the previous HEAD and are unrelated to the SETUP work (own-call highlight is green by design, BD
  nav is hidden rather than removed, `txSlotPauseVisual` unimplemented).

### `81c3797` update changelog

* Documentation only — first version of this file.

---

## REV 20260724 — 2026-07-24

### `62e7e94` Auto LOG QSO, TimeTable

* **Automatic QSO logging for JS8.** A dedicated, permanent JS8CALL log is created and used
  independently of `activeLogId`; a QSO is logged automatically once SNR has been exchanged in both
  directions, with per-band de-duplication. `VIEW LOG` opens `/log` in a new window.
* **FREQ TIMETABLE** on the JS8LAN page: a sparse 24-hour UTC schedule of 48 half-hour slots that
  retunes the band automatically. Browser-side, stored in the settings; a band change is deferred
  while transmitting so it can never cut a TX slot.
* `Recent traffic` gained a **TX** filter next to `5 min / ALL / MYCALL`, showing only own (red)
  transmissions.
* EMAIL session fields aligned left under each other with labels on the left, matching the
  `Gateway callsign` field.
* `HB` and `TUNE` buttons stay side by side on narrow screens instead of stacking full width.
* **`partitions.csv` added to the repo** — custom layout (app0 1.375 MB / SPIFFS 2.56 MB, no
  coredump) so builds no longer depend on the Arduino IDE partition menu.

### `f212010` remove BT and CAT, fix network TX

* **Bluetooth SPP transport removed.** Supported interfaces are now LAN, CI-V and TrxNet only;
  firmware shrank by roughly 739 kB, which is what made room for the JS8 stack. CI-V constants had
  to be moved out of the `#if defined(BLUETOOTH)` block (they were trapped inside it, 102 compile
  errors) — only the CAT layer and its includes stay guarded.
* **CAT and Band Decoder web pages dropped** from the firmware/menu to free SPIFFS; the sources are
  archived in `backups/20260724-cat-bd-pages.tar.gz`.
* **TX slot stability rework** (`docs/js8-tx-slot-stability-plan.md`): TX prebuffering starts
  earlier by the measured lead time, and from the moment a TX request is accepted competing network
  requests are held off until the slot completes. This addressed the repeated
  `TX abort: TX prebuffer missed slot` / `TXfault` entries.
* **TrxNet no longer fails to publish TRX1 frequency** after the transport refactor.
* **JS8LAN blocking + TX visibility:** the existing *Blocked DXCC* list now also fully hides blocked
  calls in JS8LAN and hard-refuses TX to them; own transmissions in `Recent traffic` are red when
  actually emitted and grey when not; a station that reacted to own TX is marked with a red dot in
  the map and table (derived from decoded *messages*, not frames, so it survives `CLEAR`).

---

## REV 20260723 — 2026-07-24 01:36

### `0e8efc4` automatic function

The unattended-operation layer. 71 files, ~4 000 insertions.

* **New JS8 automation modules:** `js8-autoreply.js`, `js8-heartbeat.js`, `js8-inbox.js`,
  `js8-relay.js`, `js8-restrictions.js`, `js8-scheduler.js`, `js8-txqueue.js`, backed by firmware
  side `js8_session.h`, `unattended_guard.h`, `unattended_events.h`.
* **Guard model changed from a hard cap to a watchdog.** A fixed transmit ceiling made continuous
  operation (e.g. long BIN transfers) impossible, so the firmware instead watches for its own stall
  and expects a liveness heartbeat from the front end to accept requests as legitimate.
* **HB ACK corrected against the JS8Call reference** — replies now use the
  `<CALL>: <CALL> HEARTBEAT SNR -NN` form seen on the air instead of a generic ACK; other automatic
  functions were re-verified against the reference implementation at the same time.
* **Settings header status shortcuts** — compact indicators on the right edge of the settings bar
  showing which automatic functions are enabled, styled after the `Recent traffic` buttons but
  signal-only.
* **DXCC country column** in the `Stations` table, reusing the prefix lookup from the LOG page.
* **Single-tab lock** — opening JS8LAN in a second browser/tab/PC shows a notice instead of
  fighting over the radio.
* **UI design pass** to make the page look less like a generated tool: restrained palette,
  emphasis only where it carries meaning, static previews prepared before applying.
* SPIFFS/asset audit: unused files removed, JSC dictionary and the Brotli decoder path explained and
  kept, and the CAT/BD page removal was costed here before being executed the next day.
* Wording pass: "unattended operation" section in SETUP renamed to *remote management of unattended
  JS8 operation*; `Modem settings` renamed to `SETTINGS`; all web GUI text in English.
* New smoke tests: `tools/js8-email-smoke.js`, `tools/js8-file-transfer-smoke.js`,
  `tools/js8-settings-smoke.js`, `tools/icom-lan-login-test.py`.

---

## REV 20260720 — 2026-07-19 … 2026-07-20

### `4f94a9e` bugfix

* CW IP announcement sequence corrected: before keying, RF gain goes to minimum and BK-IN is turned
  off, so the announcement is sidetone-only and never reaches the air; settings are restored
  afterwards.
* SETUP gained the switch that enables/disables the CW IP info.
* Documented the AP topology question (phone as AP with client isolation) that had made the
  data path unclear.

### `7d78416` IP send by CW on start up

* `announceIpViaCw()` now also runs when the radio is connected over **LAN**, not just BT. Mode,
  BK-IN, AF and RF gain are snapshotted and restored around the announcement.
* Fixed JS8LAN input fields (starting with `Recipient`) that dropped characters as they were typed.

### `44169b8` QSO Log button — not tested on radio

* `LOG QSO` button on the JS8LAN page; the TRX label always resolves to TRX1, since JS8LAN works
  only with the primary radio.

### `2c92055` Refresh js8lan data after leaving and returning — not tested on radio

* JS8LAN state is preserved in `sessionStorage` and restored when the page is re-entered, instead of
  showing stale or empty tables.
* `docs/js8call-komunikacni-funkce.md` written from the JS8Call documentation as the feature list
  for normal on-air communication (HB replies, relaying, …).

### `db4d69e` finish network audit

* Closing items of `docs/icom-lan-network-audit.md`.
* `Recent traffic` wraps long messages instead of truncating them with an ellipsis.

### `d240873` publish to web upload tool · `3de89a6` new map and net fix · `8b230b0` #2 #3 #4 #5 net fix

* **Audit item #4:** the overloaded `connected()` state was split into `sessionConnected`,
  `catHealthy` and `audioReady`, so a CAT hiccup no longer yanks power state and audio.
* **Audit item #5:** the audio sub-channel got its own no-data recovery; the console flood of
  `LAN | audio no data, reopening sub-stream` was traced and stopped.
* New polar **map preview** for decoded stations: linear rings, dots instead of callsigns at the
  ends, current radius printed in the corner (preview only — selection stays in the tables).
* Build artefacts republished to the web flasher.

### `d7d3dd7` gui update

* Responsive top menu bar — on narrow phone screens the page no longer becomes wider than the
  display.
* Waterfall vertical marker made visible again.
* SETUP / WiFi section gained a step-by-step guide for joining the IC-705 to the same AP.

### `91994de` gui update and fix

* `docs/icom-lan-network-audit.md` — full audit of the LAN sequence, plus a firmware health smoke
  test (`icom_lan_client_health_smoke.cpp` with `WiFi.h`/`WiFiUdp.h` stubs).
* Own callsign highlighted red in `Stations` and `Recent traffic` (the decoder does hear your own
  signal — the LAN stream is duplex).
* `Recipient` can no longer be your own callsign.
* Table action buttons moved inside their block, as the list header's first row.

### `2e81dd2` bugfix

* **EMAIL mode** (`data/js8-email.js`) per `docs/js8call-email-gateway-implementace.md`, with
  `Server callsign` renamed to `Gateway callsign`.
* **BIN mode** — binary file transfer (`data/js8-file-transfer.js`) per
  `docs/js8call_file_transfer_implementation_guide.md`, including size limits per selected speed and
  rejection of oversized files on import.
* `aud1_ws_parser.h` and `icom_lan_tx_history.h` extracted from the sketch; storage moved to
  **LittleFS** (`LFS | mounted used=… total=1966080`).
* Default `TX audio gain` set to 0.25 and the help text extended with
  `MENU/SET/Connectors/MOD Input/WLAN MOD Level = 25%`.
* HB no longer displays text it does not transmit (`@HB HEARTBEAT JO70` vs the transmitted
  `@HB JO70`).
* `Direction — 1000 km` column shortened to `kkm`, `Last heard` to `Last`.
* Full audit of the network sequence after repeated "we fix one thing and break another" cycles.

### `0c121ad` bugfix

* Own callsign appearing in `Stations`/`Recent traffic` turns red and auto-expands a collapsed
  block.
* With a `Recipient` selected, messages directed at someone else are hidden; undirected messages
  stay visible.
* **Per-character TX progress colouring** — the message background fills as characters are actually
  transmitted, pauses at a slot boundary and continues in the next one; waiting/failed and sent
  messages are visually distinct.
* Callsigns (own and the called station) are now included in composed messages such as
  *reply with received SNR*, and in HB — checked against the JS8Call specification.
* `My callsign` / `My grid` fields in modem settings became editable (characters no longer vanished).
* `TX abort: TX prebuffer missed slot` diagnosed and handled instead of silently dropping the
  transmission.

### `7ca05ee` fix installer

* Web installer at `https://ok1hra.github.io/IC-705_Interface/` failed with
  *Failed to initialize*; ESP Web Tools was pinned back to **10.2.1** and the published assets
  regenerated. (The board has no BOOT button; unplug/replug of the USB cable was the workaround
  during diagnosis.)

---

## REV 20260718 — 2026-07-18

### `319b074` JS8LAN – Web Client for JS8Call

The largest change in the project's history: 546 files, ~434 k insertions (most of it the vendored
JS8Call source used to build the decoder).

* **JS8Call decoder/encoder integrated** as WebAssembly built from `JS8Call-improved-master`
  (Eigen and the JS8 mode sources vendored, licences included), driven from the browser over the
  binary audio WebSocket.
* Waterfall reworked: slower, vertically compressed, recommended JS8 audio passband marked with thin
  green lines and the area outside it dimmed; decode LED bar and slot "thermometer" added so the
  background rhythm is visible.
* `Stations` table with speed letter *and* its real speed, distance column in thousands of km with
  the DXC bearing algorithm (falling back to the DXC estimate when no grid was decoded).
* `Recent traffic` in messenger style — received on the left, sent on the right; single-line rows
  except on very narrow screens.
* **TX SESSION** selector with `CHAT / EMAIL / BIN` modes, preset message dropdown (CQ, SNR
  reply, …), `TUNE` button next to `HB`, and `Enter` sending instead of a large `Send` button.
* Frequency/mode of the radio shown in the page header with a band preset popup for retuning; tables
  are cleared when the dial moves more than 2 kHz and restored on return.
* `?` help popup with the complete IC-705 preset for JS8 over WLAN
  (`MENU/SET/Connectors/MOD Input/DATA MOD = WLAN`, …).
* Menu item renamed `DATA` → `JS8CALL` → finally **`JS8LAN`** with a *Web Client for JS8Call*
  tooltip; `DEBUG` removed from the menu, `BD` shown only on supported hardware, `CAT` moved to the
  end and de-emphasised; `beforeunload` warning when leaving with a live session.
* Page shows an explanatory warning instead of the UI when TRX1 is not configured for LAN.
* **Asset pipeline**: `tools/minify-spiffs-js.sh`, gzip/Brotli generation and
  `tools/upload-firmware-spiffs.sh` (export compiled binary → build SPIFFS → upload) replacing the
  Arduino IDE data-upload menu, which could not fit the tree any more
  (`SPIFFS_write error(-10001): File system is full`).
* Web flasher page now states the required hardware (MCU type and flash size).

---

## REV 20260717 — 2026-07-17 … 2026-07-18

### `7ddfc48` new data page without any modem

* **New DATA page**, active only when the primary TRX1 runs in LAN mode: audio waterfall fed by a
  real RX stream, plus placeholders for the modem's input/output.
* **Audio over the Icom LAN protocol**: uLaw 8 kHz RX and TX sub-stream, exposed to the browser over
  a binary WebSocket on **port 83**; PTT is keyed over CI-V. The last missing piece for the first
  successful transmission was the radio's own `MOD Input = WLAN` setting.
* `docs/modem-implementation.md` written as the integration contract, and the standalone
  `prototype/js8-core-prototype/` toolchain (Emscripten, real-time factor and vector smoke tests)
  set up so modem work stayed strictly outside the firmware until integration.
* UI direction chosen: compact layout, blocks collapsible under headers, ordered by importance, all
  texts in English.

### `53a9ad4` new LAN TRX connect · `ddeab71` bugfix

* **`icomLanClient.h` — native Icom LAN client**: control/CI-V/audio UDP channels, login and
  capabilities exchange, `I am here` handshake, retransmission handling. Verified against the
  IC-705 and tried against an IC-7610. Six documented deviations from wfview were needed to make the
  IC-705 accept the session (ready on every channel, no `resetcap`, the 0x05 auth gate, fixed ports,
  0x05 open, controller `0xE1`).
* **SETUP per-radio connection type** — dropdown `Icom-BT / Icom-LAN / Icom-CIV / TrxNet` for each
  TRX with enable switches, fields shown per selection, LAN as the default choice, and the
  configuration correctly saved/restored across reboots.
* CAT page: MODE display and `FIL1-3` filter setting fixed; a frequency entered directly in the
  `FREQ` field now updates the page immediately.
* LOG band map no longer covers the last log rows and no longer toggles on/off.
* The long-running "DXC keeps dropping WS and Telnet" hunt ended as a false alarm — five DXC windows
  were open at once. CI-V/DXC handling was hardened along the way.

---

## REV 20260712 — 2026-07-12

### `5a27f6d` new Log dupe in DXC, some bugfix

* DXC spots are matched against the log and duplicates highlighted (`refreshLogDupeSet`,
  `collectVisibleRows`).
* Adaptive CAT polling cadence (`CAT_POLL_MS` / `CAT_POLL_FAST_MS` / `CAT_FAST_HOLD_MS`,
  `AUX_POLL_MS`): the fast cadence is held only while the CAT page polls `/state?fast=1`, which
  keeps the BT SPP link out of sniff mode without loading the rest of the system.
* WiFi supervision thresholds (`WIFI_RESTART_AFTER_MS`, `WIFI_HARD_RESET_AFTER_MS`) and
  `DXC_CONNECT_TIMEOUT_MS` introduced.

---

## REV 20260707 — 2026-07-10

### `775bd76` TrxNet new setPriorityPrefixes + many bugfix

* TrxNet priority prefixes configurable and persisted (EEPROM 288 flag + 289–359 string, 8 tokens ×
  8 characters), loaded into the live token buffers at boot.
* Call search: `Enter` jumps to the first match and dismisses the popup; `Enter` in any text field
  saves (except in `<select>`).
* On-screen-keyboard viewport handling for phones and tablets.

---

## 2026-06-09

### `ba18e2b` snapshot

* **CI-V serial transport for TRX2/TRX3** — a second way to reach the auxiliary radios besides
  TrxNet. Includes a CI-V framer with address filtering (frames addressed to the controller or
  broadcast), a non-blocking polling state machine and a single UART0 RX pump shared with the CLI.
* Per-TRX connection type (`CONN_TYPE`: TrxNet or CI-V) with the unused input disabled in the UI;
  CI-V addresses stored in EEPROM bytes 48/49.

---

## REV 20260523 — 2026-05-22 … 2026-05-23

### `27f70d1` tune · `20fa51e` tuning

* Analysis and mitigation of `LOOP| slow: webServer` stalls of up to 10 s observed while TrxNet and
  DXC were active.

### `fee2ee4` Band map now tunes the currently selected TRX · `d1754dd` Fix band map not showing when TRX2/3 is active · `7550835` bugfix

* Clicking the band map tunes the currently selected radio instead of always TRX1.
* The band map is displayed for TRX2/TRX3 under the same conditions as for TRX1 — it is driven by
  that radio's frequency and online state, not reserved for the primary.

---

## REV 20260522 — 2026-05-22

### `90477f0` tuning · `a2da7e7` fix DXC

* **LOGSYNC pairing regression fixed** — after the label rework the two devices stayed stuck at
  *Waiting for the other device…* because their auto-generated device labels and device IDs no
  longer matched the pairing key.
* DXC page fix.

---

## REV 20260520 — 2026-05-20

### `21e0354` tuning

* Device label in LOGSYNC generated automatically from browser name, IP and a unique suffix, and
  displayed on the page again.
* **WiFi signal strength in the top menu of every page**, grey normally and red below −70 dBm.
* SPIFFS upload failures from the Arduino IDE worked around in the asset scripts.

---

## REV 20260519 — 2026-05-19 … 2026-05-20

### `825ce67` fix TrxNet, tune html

* TrxNet `publishTo(peerName)` made mandatory, removing the broadcast publish path.
* LOG now formats CW/messages for TrxNet when TRX2/TRX3 is selected instead of sending them in the
  Bluetooth form.
* `Alt+Enter` saves a QSO without sending the macro; the shortcut list under `?` on the LOG page
  updated.
* Frequency/mode polling slowed to ~0.5–1 s outside the CAT page, relieving the BT link and the web
  server.
* The `LOOP| slow: webServer 9–19 s` stall that made SETUP unreachable (and cost the device its IP)
  was traced and fixed.

### `fcee7c0` tune html

* **Pre-compressed assets**: `tools/gzip-assets.sh` added and every page shipped as `.gz`, which is
  what made LOG and SETUP load quickly. Editing a source `.js` has no effect until the generator
  scripts are re-run — the firmware serves the compressed copies.

---

## REV 20260517 — 2026-05-17 … 2026-05-19

### `3b4e58b` custom BT name, Band Decoder for hw rev 04

* Configurable Bluetooth device name.
* **Band Decoder for hardware revision 04**: `bd.html` / `bd.js`, shift-register pins
  (`BD_CLOCK_PIN`, `BD_DATA_PIN`, `BD_LATCH_PIN`), configurable row/column ranges with red/green
  priority highlighting, and updates driven by the TRX1 frequency.

### `2453067` bugfix · `f94c271` tuning

* Crash after restoring a saved SETUP configuration analysed and fixed.

### `175e59c` remove MQTT | add TrxNet · `f30af41` fix TrxNet

* **MQTT removed and replaced by TrxNet** (`docs/trxnet.md`): `/hz` and `/mode` from any peer feed
  the TRX2/TRX3 slots, `/s-hz` sets the IC-705 VFO through CI-V. Callbacks stay short and the work
  is processed in the main loop. EEPROM 44–46 freed; `NET_ID 0x00` acts as the "TrxNet disabled"
  sentinel.

---

## REV 20260516 — 2026-05-16

### `9e7f0e2` bugfix, redesign log exch

* **Exchange redesign**: instead of the opaque `NR / JO70FD`, the setting offers `TU / NR / LOC`
  with a `?` help bubble, and `LOC` uses the locator from `My locator`.
* Documented how CW/RTTY memories are selected per operating mode, and how VHF QSOs get the locator
  format.
* Bluetooth stack crash (`ASSERT_PARAM(1024 0), in rwbt.c at line 381`) analysed and mitigated.

---

## REV 20260515 — 2026-05-15

### `a5d0e46` bugfix

* **SETUP persistence audit and rebuild.** TRX2/TRX3 label, backend IP and OI3 flag, and the
  CW/frequency memories, survived only until the next page load and were missing from the exported
  config; `Save & Restart` could fail with a network error. Storage, load and export/import were
  reworked rather than patched.
* LOG page mode mapping corrected (`1 = CW`, `2 = SSB`, `4 = RTTY`).
* Band map shown for TRX2/TRX3 as well.

### `3d0b3e4`, `6db5110`, `f558091`, `2947193`, `daa3974`…`51823e4`, `1c3f960`

* Screenshots (`docs/CAT.png`, `DXC.png`, `LOG*.png`, `SETUP.png`, `sw-block.png`), README rewrite,
  user manual update and gh-pages republish.

---

## REV 20260513 — 2026-05-13 … 2026-05-14

### `4dbfb63` bugfixing

* WiFi now scans before connecting, which removes the infinite loop that dropped the device into AP
  mode when configured SSIDs were absent.
* Clarified in the UI what the three network input ports do; the SETUP heading became
  *Network input ports*.

### `2ac2ee3` backup log database

* Log database backup with a `beforeunload` guard and a timer, so a browser-side log cannot be lost
  silently.

### `ad2ec91` DXC

* **New DXC page** — Telnet DX cluster client with band/type/direction/DX filters, zoom, column
  layout and bearing/distance computation.

### `82ade75` LOG bandmap

* SVG band map on the LOG page with live spots published from DXC
  (`publishVisibleDxccSpots`, scale and band rendering).

### `1dc02d6` fix continue

* Storage-persistence check with an explanatory popup (Firefox auto-grants when bookmarked, other
  browsers need the permission dialog).

---

## REV 20260509 — 2026-05-09 … 2026-05-11

### `6b2e5e4` redesign DATASYNC

* Pairing moved to firmware endpoints (`/pairing/offer|answer|reject` with CORS handling) and the
  sync vector rebuilt from the QSOs actually stored, so deletions no longer left a stale
  `sync_state` and partial transfers are re-requested correctly.
* Global dupe/partial search across all logs.

### `02db511` new Log import

* **Log import** with format auto-detection: ADIF, Cabrillo and EDI parsers plus normalisers and an
  import dialog.

### `e53cc4e` GitHub pages USB-C web flasher · `b506dcb` README

* `tools/gh-pages.sh` and a published `build/gh-pages/` manifest — the device can be flashed from
  the browser over USB-C, no toolchain required.

### `be12627` partial save · `42e79bb` clear code + user-manual · `7ea02a2` bugfix

* `data/fw-version.js` (firmware version shown in the UI, fetched locally or remotely),
  `docs/user-manual.md` written, TRX2/TRX3 buttons hidden when no IP is configured with a fallback
  to TRX1, serial RX buffer flushed after boot noise.

---

## REV 20260508 — 2026-05-08 … 2026-05-09

### `c72eff3` redesign structure · `94d1f87` final CAT

* CAT internals reorganised: separate frequency/RIT read pauses while the user is tuning, queued
  command posting, wheel-step handling, CW and frequency memory rendering.
* RIT set/clear (CI-V `0x21`), LSB-first BCD encoding helper, S-meter/SWR/supply sub-bar mapping and
  GPIO FSK keying.

### `d889a4a` new LOG

* **New LOG page** (3 619 insertions): QSO entry with a form state machine, DXCC lookup, exact and
  partial dupe checking, QRB/azimuth from the locator, macros, ADIF field mapping, QSO edit dialog,
  multiple logs (create/delete/activate), CW numbers, and keyboard shortcuts
  (`Alt+U` RUN/S&P, `Alt+W` clear, `Esc` close dialog).

### `1295305` new DATASYNC

* **Log synchronisation between devices** — offer/answer pairing with a QR code
  (`qrcode.min.js`), IndexedDB import/export, compressed range transfers and a device identity.

---

## REV 20260505 — 2026-05-05 … 2026-05-08

### `f6c0122` MQTT bugfix | MQTT RX freq

* Fixed the MQTT publisher that stopped after two frequency messages while the rest of the firmware
  kept running.
* The CI-V `SET` sequence is now sent only on the first connection after a reboot, not on every
  radio reconnect.
* **New MQTT RX topic**: a received frequency is written to the IC-705 VFO over CAT.
* Additional WiFi SSID slots in the web form.

### `c488f18` start WebSocket CAT proxy

* **WebSocket ↔ CI-V proxy** (design later written up in `docs/websocket-civ-proxy.md`): live rig
  state on `/ws` plus a raw CI-V console, with the web assets moved to SPIFFS to make room.
  Includes a diagnostic mode with an event log, filtering of page-generated traffic and clipboard
  copy.

### `24e2920` CAT page

* **New CAT page** (black theme, `CAT | WS-CAT | SETUP` navigation added to every page): large
  frequency display in `MHz.kHz.Hz` with a smaller Hz group, red while transmitting, fixed digit
  positions from 1 to 3 MHz digits, mouse-wheel tuning by the digit under the cursor and
  left/right click for ±1, S-meter, sliders with values, `FREQ` direct entry, radio and WiFi status
  in the bottom bar.

### `8f1a99e` CAT finish

* Key speed shown in WPM and RF power in %, SWR/supply rendered as bars matching the S-meter with a
  thin separator, sliders greyed/zeroed when the radio is off or disconnected.
* Four CW memories (30 characters each) configurable in SETUP and sendable from the CAT page.
* CI-V address and Icom model selection (`IC-705-BT`, `IC-7610-CI-V`) added to SETUP.
* Fixed the recurring `Guru Meditation Error: Core 1 panic'ed (LoadProhibited)` and
  `ERROR: Too many messages queued` crashes seen on band changes, and a `CORRUPT HEAP` after long
  idle periods.

---

## Earlier history (before the current development cycle)

| Commit | Date | Summary |
| --- | --- | --- |
| `a5be7ef` | 2026-03-22 | rev 20260322, many bugfixes |
| `38ae41d` | 2025-02-07 | DV mode added (not working) |
| `1a015c0` | 2024-12-21 | New debug setting in the CLI |
| `b2e9bae` | 2024-08-11 | Hardware revision 03 |
| `1596f7c` | 2024-02-03 | First release, 20240203 |
| `58a22c1` | 2024-01-28 | WiFi AP mode and the SETUP web form on port 80 |
| `6ab8f77` | 2024-01-16 | IP announcement in CW, CLI info |
| `a14f5c7` | 2024-01-11 | Bluetooth name moved to the config |
| `5f00f6e` | 2024-01-06 | RTTY operation fixed |
| `834341f` | 2023-12-29 | MQTT postponed |
| `c593307` | 2023-12-28 | PWR output fixed |
| `bb41c7d` | 2023-12-27 | UDP to CAT, TRX selection |
| `dafb140` | 2023-12-25 | CI-V output and mute |
| `31350c8` | 2023-12-21 | Antenna switch, mDNS, watchdog, PWR OUT |
| `0544d1d` | 2023-12-18 | 3D print model |
| `84d85df` | 2023-12-17 | FSK fixed, hardware ID and status LED |
| `b3a860e` | 2023-10-31 | FSK support (untested) |
| `f1bed1e` | 2023-10-29 | udp2cw fixed |
