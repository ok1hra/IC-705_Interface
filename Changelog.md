# Changelog

All notable changes to **IC-705_Interface**, grouped by firmware revision (`#define REV`) and
broken down per commit.

Sources used to reconstruct the entries: git history and diffs, Claude Code session transcripts
(`~/.claude/projects/-home-dan-Arduino-hra-ok1hra-esp32-IC-705-Interface/`), Codex session logs
(`~/.codex/sessions/`, filtered by this project's `cwd`), and the design notes in `docs/`.

Newest first. Dates are local (CEST). A firmware revision is the `REV` value flashed with that
commit — several commits usually share one revision, and the revision is bumped when a build is
published.

---

## Unreleased — REV 20260725 (working tree)

Work in progress, not yet committed.

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
* **Documentation added** (still untracked): `setup-interfaces-architecture.md`,
  `icom-lan-implementace.md`, `js8call-komunikacni-funkce.md`, `js8call-log-qso-auto.md`,
  `js8call-neobsluhovany-provoz-plan.md`, `js8-tx-slot-stability-plan.md`, `how-to-bugfix.md`,
  `how-to-regenerate-manual.md`, `band-decoder-implementation.md`, `websocket-civ-proxy.md`,
  plus `docs/agents/`.
* **Known state:** the wider browser smoke suite still reports 5 red checks that were already red at
  HEAD and are unrelated to the SETUP work (own-call highlight is green by design, BD nav is hidden
  rather than removed, `txSlotPauseVisual` unimplemented).

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
