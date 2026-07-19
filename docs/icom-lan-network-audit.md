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

4. **Jeden stav `CONNECTED` má příliš mnoho významů.** Další refaktor má oddělit
   `sessionConnected`, `catHealthy` a `audioReady`; jinak změna CAT health dál
   ovlivňuje PWR a audio lifecycle.

5. **Audio podkanál nemá vlastní no-data recovery.** Pokud zůstane UDP audio
   socket otevřený, ale rádio neposílá payload, UI už správně ukazuje `RX WAIT`,
   avšak firmware audio kanál sám nezavře a znovu neotevře.

6. **Control health neověřuje identitu odesílatele.** Oddělení control/CAT/audio
   odstranilo aktuální falešnou živost, ale `pumpControl()` zatím považuje každý
   dostatečně dlouhý paket na lokálním control portu za aktivitu session.

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
  zbytek odloží (deferred), takže storm nezmrazí smyčku.
