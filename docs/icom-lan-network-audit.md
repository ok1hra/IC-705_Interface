# Audit síťové sekvence ICOM LAN

Datum auditu: 2026-07-19

## Rozsah a zdroje

Audit pokrývá control, CI-V a audio UDP kanál, jejich spolehlivost, stavový
automat a reconnect v `IC-705_Interface.ino`. Porovnání bylo provedeno proti:

- lokálnímu klientu `tools/icom-lan-login-test.py`, dříve ověřenému na IC-705;
- wfview, commit `cd18ea55fe479eb4526d1732b443cbfc3969c540`;
- kappanhang, commit `66f2369efcddd4c00caf4d2ddf0e54116582cfaf`;
- živým logům IC-705 z tohoto ladění.

`docs/icom-lan-implementace.md` je návrhový a historický dokument. Pro aktuální
chování mají přednost tento audit, testy a implementace.

## Mapa vlastnictví

| Vrstva | Vlastník | Odpovědnost |
|---|---|---|
| Transport session | `IcomLanClient` | login, token, stream request, UDP kanály |
| UDP spolehlivost | `IcomLanClient` + `IcomLanTxHistory` | tracked sequence a odpovědi na retransmit request |
| CAT health | `IcomLanClient` | CI-V Ready/Open, adresovaný probe, watchdog a polling |
| CAT stav rádia | `lanCivFrameHandler()` | frekvence, mód, metry, PTT a další globální stav |
| Integrace aplikace | `lanClientLoop()` | PWR, audio lifecycle, TrxNet, reconnect/backoff |
| Audio consumer | `lanAudioHandler()` | AUD1 WebSocket a JS8 audio |

Kritická vazba: `lanClientLoop()` používá `connected()` zároveň jako CAT health,
PWR stav i povolení audio kanálu. Proto změna významu prvního CI-V paketu zasáhne
víc částí aplikace než jen CAT parser.

## Očekávaná sekvence

### Control 50001

1. AreYouThere (`0x03`) -> IAmHere (`0x04`).
2. AreYouReady (`0x06`) -> Ready (`0x06`).
3. Login -> login response a token.
4. Token confirm `0x02`, token auth `0x05`.
5. Až po auth odpovědi a capabilities poslat stream request.
6. Status vrátí vzdálené CI-V/audio porty.
7. Za běhu ping 500 ms, tracked idle 100 ms a token renewal 60 s.

### CI-V 50002

1. AreYouThere -> IAmHere.
2. Opakovat AreYouReady, dokud nepřijde Ready. Před Ready rádio může posílat
   broadcasty a pingy, ale příkazy klient->rádio zahazuje.
3. Poslat OpenClose s magic `0x05` a samostatným big-endian data sequence.
4. Poslat jednoduchý `0x03` read-frequency probe z adresy `0xE1`.
5. Teprve adresovaná odpověď `to=E1, from=radio` prokazuje funkční CAT cestu.
   Broadcast `to=00` aktualizuje UI, ale není odpovědí na probe.
6. Po ověření vypnout unsolicited scope a spustit CAT polling.
   Vždy smí být nevyřízený nejvýše jeden CAT požadavek; další se odešle po
   adresované odpovědi nebo po 500ms timeoutu.
7. Při dvousekundovém tichu nejprve read-frequency probe na stávajícím streamu.
   Teprve jeho timeout spustí CI-V Open recovery; po 6 s neúspěšné recovery se
   obnoví celá přihlášená session.

### Audio 50003

1. Kanál se otevírá jen při připojeném Audio WebSocket klientu.
2. AreYouThere -> IAmHere -> AreYouReady -> Ready.
3. RX audio lze přijímat před dokončením Ready; TX audio však musí čekat na
   dokončený handshake.
4. Audio tracked sequence je nezávislá na CI-V a control sekvenci.

## Nálezy

### P0 — opraveno v tomto auditu

1. **Retransmit range byl dekódován jako seznam jednotlivých sekvencí.** Paket
   typu `0x01` s délkou větší než 16 B obsahuje dvojice `first,last`, oba konce
   včetně. Log `BE,C5 -> BF,C4 -> C0,C3` přesně ukázal, že rádio žádalo celý
   rozsah `BE..C5`, zatímco firmware obnovoval jen krajní pakety. Výsledkem bylo
   trvalé blokování následujících CI-V příkazů.

2. **CI-V Open byl posílán před Ready.** Fallback `no ready, opening anyway`
   vytvářel poloviční spojení: rádio odpovídalo na pingy a posílalo broadcasty,
   ale zahazovalo CAT příkazy. Nyní se opakuje Ready handshake a Open se odešle
   až po potvrzení.

3. **Broadcast byl zaměněn za CAT request/reply health.** Ruční ladění generuje
   `to=00 from=A4 cmd=00`; tento rámec se dál parsuje pro UI, ale nepřepne stav
   na `CONNECTED` a neukončí CAT recovery. První health probe je adresovaný
   read-frequency.

4. **CAT watchdog rušil celou přihlášenou session.** Recovery nyní znovu otevírá
   pouze CI-V podkanál. PWR a audio se při krátkém CAT výpadku nerozpojí.

5. **CAT poller přepisoval nevyřízené požadavky.** Příkaz se nyní neposune dál,
   dokud nepřijde adresovaná odpověď nebo neuplyne 500 ms. Mode se nejprve čte
   přes `26 00` (včetně DATA příznaku); po timeoutu následuje kompatibilní `04`.
   Jakmile rádio jednou odpoví na `26 00`, legacy fallback se vypne, aby
   nepřepisoval `USB-D` na `USB`.

6. **Dvě sekundy bez odpovědi okamžitě znovu otevíraly zdravý CI-V stream.**
   Několik nepodporovaných telemetrických dotazů za sebou snadno vytvoří delší
   mezeru, i když frekvence a mode fungují. Watchdog nyní nejprve pošle na
   stávajícím streamu známý read-frequency probe `03`. CI-V Open použije až po
   další sekundě bez odpovědi na tento probe.

7. **Libovolný UDP paket maskoval mrtvou control session.** CI-V retransmit nebo
   audio paket obnovoval společný `lastRxMs`, takže `/state` zůstával
   `connected:true`, i když rádio už ukončilo WLAN session. Whole-session
   watchdog nyní sleduje pouze control kanál.

8. **CI-V recovery neměla konečný stav.** Po neúspěšném probe posílala Open
   neomezeně dlouho. Recovery je nyní omezená na 6 s; potom přejde do
   `LAN_FAILED`, uvolní token a naváže novou session.

9. **Explicitní odpojení a odmítnutý token se ignorovaly.** Control status
   `disc=1` je nyní autoritativní konec session i při nenulových stream portech.
   Nenulový chybový kód odpovědi na token auth/renewal také vyvolá reconnect.
   Perioda renewal byla sjednocena s referencí na 60 s.

10. **JS8LAN zaměňoval navázanou session za živý RX.** `RX LIVE` nyní vyžaduje
    audio vzorek mladší než 1,5 s. Navázaná LAN session bez audia zobrazuje
    `RX WAIT`; odpojená session `OFFLINE`.

11. **Reconnect log hlásil jinou prodlevu, než právě naplánoval.** Log se nyní
    vypíše před navýšením backoffu, takže první retry skutečně hlásí 3 s.

12. **Obecná chyba loginu mohla pokračovat do token auth.** Každý nenulový
    login nebo stream-status error nyní ukončí pokus; shoda `tokRequest` už
    nemůže přepsat chybový výsledek.

### P1 — auditováno, zatím neimplementováno

1. **Chybí spolehlivost rádio->ESP.** Firmware umí odpovědět na retransmit
   request rádia, ale nedetekuje mezery v příchozích tracked sekvencích a sám
   nežádá jejich opakování. Ztracená login odpověď proto skončí obecným timeoutem.

2. **Audio nemá úplnou replay vrstvu.** TX audio používá tracked sequence, ale
   paket se neukládá do historie a `handleAudio()` neobsluhuje retransmit request.
   RX audio se také nereorderuje podle tracked sequence a TX je dnes povolen už
   po IAmHere, nikoli až po Ready.

3. **Control Ready se po ztrátě neopakuje.** CI-V handshake je po auditu odolný,
   control handshake zatím spoléhá na jediný AreYouReady paket.

4. **Jeden stav `CONNECTED` má příliš mnoho významů.** ✅ VYŘEŠENO (viz doplněk
   níže). PWR/TX už korektně sledovaly `state==CONNECTED` a CI-V recovery `state`
   neměnila; doplněny accessory `catHealthy()` / `audioReady()` a vystaveny v
   `/state`, takže UI hlásí CAT/audio zvlášť místo přetíženého `connected`.

5. **Audio podkanál nemá vlastní no-data recovery.** ❌ ZAMÍTNUTO po živém testu.
   Automatický reopen na payload-ticho je chybný: (a) rádio na tichém/zašuměném
   kanálu audio legitimně neposílá, takže „no data" NENÍ zaseknutí; (b) reopen je
   destruktivní — po odpojení/znovuotevření podkanálu rádio audio neobnoví →
   nekonečná smyčka reopenů. UI už správně ukazuje `RX WAIT`. Ponechán jen
   `audioLastDataMs` + `audioReady()` jako RX-live indikátor (freshness, ne reopen).

6. **Control health neověřuje identitu odesílatele.** ✅ VYŘEŠENO. `fromRadio()`
   guard v `pumpControl()`/`pumpCiv()`/`pumpAudio()` zahodí datagram, jehož zdroj
   není nakonfigurovaná `radioIP` — cizí/spoofnutý provoz už neobnoví session
   health ani neinjektuje CAT/audio.

7. **Příčina fyzického zmizení WLAN ikony není z dodaného logu prokázaná.**
   Nové rozlišené konce (`radio disconnected session`, `token auth rejected`,
   `no control packets`, `CAT recovery failed`) umožní v následujícím živém
   testu určit, zda rádio session explicitně zrušilo, vypršel token, nebo zmizel
   celý control kanál.

## Doplněk 2026-07-19: kompenzace stallu smyčky (health timery)

Živý test (5 min bez / 5 min s JS8LAN, firmware REV 20260719 s TX audiem) rádio
neodpojil — žádný z rozlišených konců (`no control packets 6s`,
`CAT recovery failed`, `radio disconnected session`, `token auth rejected`)
nevystřelil, session zůstala `CONNECTED` a každý `CAT probe timeout` skončil
`CAT replies restored`. Log ale odhalil mechanismus: při page-loadu (stažení
wasm) a audio streamu se jednotlivé volání smyčky protáhne na 1–4 s
(`LOOP| slow: webServer 4129ms`, `LanClient 1802ms`, `audioWs 1489ms`), což
vyvolá falešný CAT reopen a retransmit storm, a jednou i `TX prebuffer missed
slot`.

**Nález (audit všech wall-clock timerů v `IcomLanClient::loop()`):** health
rozhodnutí měřila RX-ticho absolutním `millis()`, takže čas ztracený ve stallu
smyčky se počítal jako ticho linky. To se týkalo pěti checků — nejen
`civSilent` (falešný reopen, řádek ~170), ale i **`now - lastCtrlRxMs > 6000`
(řádek ~258), který zahodí celou session**. Stall > 6 s (velký soubor + audio)
tak mohl session falešně shodit; je to kandidát na původní mizení WLAN ikony.

**Oprava:** jednotná kompenzace na začátku `loop()`. `lastServiceMs` drží čas
předchozího průběhu; když mezera přesáhne `LAN_STALL_FORGIVE_MS` (300 ms),
`forgiveClock()` posune `lastCtrlRxMs`, `lastCivDataMs`, `stateSince`,
`civHealthProbeSentMs` a `civRecoveryStartedMs` o délku stallu (s clampem na
`now`, bez unsigned wrapu). Send-kadence (pingy, idle, poll, reauth) se
nekompenzuje — smí po stallu vystřelit hned a obnovit tok. Skutečné RX přijaté
během mezery i tak osvěží timery v `pumpControl()/pumpCiv()`, které běží až po
kompenzaci, takže živá linka není dotčena; jen opravdu tichá linka drží
(stall-forgiven) odpočet. Substantní stall (≥ 800 ms) se loguje jako
`LAN | loop stall <ms>ms, health timers forgiven`.

Dopad na otevřené body: zmírňuje #1 (falešná CAT recovery pod zátěží) a
odstraňuje falešné shození session v bodě, na který míří #7. Body #4 (rozdělit
`sessionConnected`/`catHealthy`/`audioReady`), #5 (audio no-data recovery) a #6
(identita control paketů) zůstávají otevřené.

## Doplněk 2026-07-19: retransmit rate limit (reconnect freeze)

Druhý živý test (~2h, jeden reconnect) potvrdil kompenzaci stallu (desítky
`loop stall … forgiven`, session přežila) a odhalil skutečný drop:
`no control packets 6s, link lost` = genuinní control-ticho ~26 s pod zahlcením
WiFi audiem, následovaný **úspěšným 3s-backoff reconnectem**. To je hledaná
příčina mizející WLAN ikony — nyní honest + self-heal.

Reconnect ale zamrznul celý firmware: `LOOP| slow: LanClient 10032ms` a `5855ms`.
Příčina: každá odpověď na retransmit request je **blokující** UDP send
(`endPacket()` čeká na plné lwIP TX buffery), a jejich počet za jednu iteraci byl
neohraničený — zvlášť stale full-history request, který rádio pošle hned po
reconnectu (seq `0xFFDF..0x0`, vše miss → fill, ještě duplikovaně).

**Oprava:** rate limit `LAN_RETRANSMIT_BUDGET` (16) na `loop()` iteraci.
`respondRetransmit()` odbaví max budget sekvencí za tick, zbytek odloží (rádio je
znovu vyžádá příští tick). Per-sekvenční Serial logy nahrazeny souhrnem
`LAN | retransmit resent=.. filled=.. deferred=..`. Tím se storm rozloží do více
iterací a smyčka zůstane responzivní (web/audio/TrxNet se stíhají obsloužit).

Zbývá: cleanup sendy v `stop()` (~4 blokující sendy na mrtvé lince) zůstávají
neohraničené — sémanticky důležité pro uvolnění session rádia; sledovat v příštím
logu, jestli po tomto fixu ještě zbývá vícesekundový `stop()` blok.

## Doplněk 2026-07-19: identita, audio recovery, split CONNECTED (#6/#5/#4)

Druhý ~2h test potvrdil kompenzaci i rate limit a self-heal reconnect. Následně
uzavřeny tři audit body:

- **#6 identita odesílatele.** `fromRadio(u)` = `u.remoteIP() == radioIP`. Přidán
  do všech tří receive smyček; cizí datagram na lokálním portu se odčerpá a
  ignoruje (neobnoví `lastCtrlRxMs`, nedostane se do handlerů).
- **#5 audio no-data recovery — ZAMÍTNUTO (živý test).** Prototyp reopenu na
  payload-ticho způsobil nekonečnou smyčku `audio no data, reopening` (rádio na
  tichém kanálu nestreamuje a po reopenu audio neobnoví). Odstraněno. Ponechán jen
  `audioLastDataMs` (stampovaný na open/IAmHere/payloadu, stall-forgiven) a
  `audioReady()` = linked + čerstvý payload do `LAN_AUDIO_FRESH_MS` (5 s), jako
  RX-live indikátor. Skutečné audio zaseknutí (vs. tichý kanál) z firmwaru
  nerozlišíme, takže žádný auto-reopen.
- **#4 split CONNECTED.** Chování bylo správné (PWR/TX = session), přidány jen
  accessory `catHealthy()` (`CONNECTED && civGotData && !civRecovering`) a
  `audioReady()` (linked + čerstvý payload) a vystaveny v `/state` jako
  `catHealthy`/`audioReady`. `/state` buffer zvětšen na 720 B.

**#5 missed TX slot (page-load) — vědomě neopraveno.** `aud1TxTargetMs` je
wall-clock deadline JS8 slotu; prodloužení grace (dnes 100 ms) by klíčovalo pozdě
a poškodilo časování JS8. Prázdný prebuffer na page-loadu je symptom hladovění
smyčky (browser načítá wasm, WS RX se nestíhá vyprázdnit) a sám se opraví na dalším
cyklu. Patří do overload větve (audio na jádro 0 / neblokující WS send), ne do
riskantní úpravy TX. Abort+retry je zde korektní chování.

## Doplněk 2026-07-20: neblokující audio WS + oddělení od LAN UDP (overload)

Kořen chronických 1–2 s stallů (`audioWs`/`LanClient`): RX audio se do prohlížeče
posílalo přes blokující `AudioWsClient.write()`, a to i **uvnitř** `pumpAudio()`
(přes `lanAudioHandler → audioFlush`), takže blokace se počítala jednou jako
`audioWs`, podruhé jako `LanClient` a přímo krmila kaskádu CAT-timeout → retransmit
→ control-loss. `select()` pre-check blokaci jen zmírňoval (částečný zápis do
neplného bufferu stejně dočkal).

**Řešení A+B (jeden výstupní WS byte-ring):**
- Veškerý výstup do prohlížeče (binární audio i textové `AUD1` řízení) se
  **enqueuje** do ringu `wsOut` (16 kB, heap — do statického DRAM se nevešel).
  `wsEnqueueFrame` přidá rámec celý, nebo ho celý zahodí → stream je vždy čistá
  sekvence WS rámců. Drop → `audioRxDiscontinuity`.
- `pumpAudio`/`lanAudioHandler` už **nedělají socket send**, jen enqueue → LAN UDP
  cesta je štíhlá (to je oddělení dle auditu: UDP reliability vs audio consumer).
- `audioDrainWs()` vyprazdňuje ring **neblokujícím** `::send(MSG_DONTWAIT)`
  (vzor jako web-server file stream); částečný send nechá zbytek na příští tick
  (TCP drží pořadí). Voláno z `AudioHandleWsClient` (každou iteraci + v
  cooperative-yield při servírování souborů).

Tím mizí obě blokace → rozbití feedback loopu. Ring ~2 s absorbuje přechodné
browser/WiFi stally než začne dropovat (frame nese sample-timestampy, časování
JS8 dekódu se buffrováním nekazí). Zbylá blokující cesta: `sendAudioPacket` (TX
audio LAN UDP) — nižší priorita (UDP blokuje vzácně). Ověřit na dalším živém logu.
Vedlejší efekt: rychlejší smyčka by měla odstranit i `TX prebuffer missed slot`
na page-loadu (#5).

**Ověřeno na zařízení (5 h provozu):** žádné `LOOP| slow`, žádné `loop stall`,
retransmit jen `resent=1/4` ojediněle, žádný drop, TX čistě prošel (#5 potvrzeno).
Jediný zbytek = periodický `CAT probe timeout → reopening → restored` cyklus:
CI-V telemetrie periodicky ~2–3 s nedorazí pod audio zátěží (kontence audio+CI-V
na jedné WiFi lince, převážně radio-side), recovery to pokaždé srovná, session
stabilní. Není to funkční problém — proto ten CAT reopen/restored log **přesunut
za `Debug`** (CLI toggle `D`), aby default konzole zůstala čistá. `Debug` je v
`icomLanClient.h` přes `#ifdef ARDUINO extern bool Debug` (host testy = quiet).

## Regresní pokrytí

`prototype/js8-core-prototype/firmware/icom_lan_client_health_smoke.cpp` ověřuje:

- úplné rozbalení retransmit rozsahu včetně duplicitní kopie requestu;
- zákaz CI-V Open před Ready a Open po Ready;
- broadcast není CAT health;
- adresovaná odpověď naváže CAT;
- CI-V recovery neruší LAN session;
- CAT ticho nejprve ověří read-frequency probe bez Open a Open použije až po
  timeoutu tohoto probe;
- replay history a potlačení redundantního idle paketu;
- polling drží právě jeden outstanding CAT požadavek;
- frequency odpověď uvolní selected-mode dotaz;
- timeout selected-mode dotazu spustí legacy mode fallback.
- CI-V retransmit-only provoz neudrží mrtvou control session jako `CONNECTED`;
- neúspěšná CAT recovery eskaluje na reconnect celé session;
- explicitní `disc=1` a odmítnutý token ukončí session;
- nenulový login error nepokračuje do token auth;
- browser bez čerstvých audio dat nezobrazuje `RX LIVE`;
- CAT timeouty (civSilent probe, probe timeout, recovery escalace) fungují, když
  je ticho linky nasbíráno zdravou smyčkou (mnoho krátkých ticků, `idleHealthy`);
- jeden vícesekundový stall smyčky (jeden `loop()` po velkém skoku hodin) se
  nezhodnotí jako ticho linky — netriggeruje CAT reopen ani drop session;
- retransmit range unroll odbaví plný duplikovaný rozsah (16), když je budget k dispozici;
- retransmit rate limit odbaví max `LAN_RETRANSMIT_BUDGET` sekvencí za iteraci a
  zbytek odloží (deferred), takže storm nezmrazí smyčku;
- cizí control paket (jiná zdrojová IP než `radioIP`) neobnoví session health,
  paket od rádia ano;
- `audioReady()` je true po čerstvém payloadu a false po delším tichu, aniž se
  reopenne podkanál nebo shodí session.
