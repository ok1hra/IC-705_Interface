# AUD1 — audio WebSocket protokol

Stav: návrh ověřovaný pouze v `prototype/js8-core-prototype`; produkční
firmware ani stránka DATA jej zatím nepoužívají.

Izolovaná referenční implementace nyní obsahuje:

- firmware-side `Aud1RxEmitter` s dvoufázovým `prepare/commit`, aby neúspěšný
  WebSocket zápis vytvořil mezeru a `DISCONTINUITY`, nikoli falešnou sekvenci;
- strict browser `WsAudioSource`, který vyžaduje `hello`, shodu `streamId`,
  zachovává původní wire a prvních pět paketů drží do uzamčení UTC epochy;
- předání `epoch(streamId, anchorUtcMs)` Workeru před prvním `audio` paketem;
- Node i nativní Chrome WebSocket test pro duplikát, mezeru a reconnect.

Tyto části nejsou importovány do `IC-705_Interface.ino` ani `data/data.js`.

## Navázání spojení

Po otevření `/audiows` server nejprve pošle textovou JSON zprávu:

```json
{
  "type": "hello",
  "protocol": "AUD1",
  "version": 1,
  "streamId": 16909060,
  "rx": [{"kind": "RX_ULAW", "sampleRate": 8000}],
  "tx": [{"kind": "TX_PCM16", "sampleRate": 48000}],
  "maxPayloadBytes": 2048
}
```

`streamId` je náhodná nenulová 32bitová identita epochy. Změní se při restartu
audio session, resetu čítače vzorků nebo novém přihlášení k rádiu. Klient při
změně zahodí rozpracované sloty; nesmí rozdíl interpretovat jako paketovou
ztrátu v předchozí epoše.

## Binární zpráva

Každá WebSocket binární zpráva obsahuje jednu 40bajtovou hlavičku a právě jeden
payload. Všechna vícebajtová pole hlavičky jsou unsigned big-endian.

| Offset | Délka | Pole | Význam |
|---:|---:|---|---|
| 0 | 4 | magic | ASCII `AUD1` |
| 4 | 1 | version | `1` |
| 5 | 1 | kind | `1=RX_ULAW`, `2=RX_PCM16`, `3=TX_PCM16` |
| 6 | 2 | flags | bitové příznaky níže |
| 8 | 2 | headerBytes | `40` |
| 10 | 2 | reserved | musí být nula |
| 12 | 4 | streamId | identita audio epochy |
| 16 | 4 | sequence | pořadí zprávy v daném směru, modulo 2^32 |
| 20 | 4 | sampleRate | vzorků za sekundu |
| 24 | 8 | firstSample | index prvního audio vzorku od začátku epochy |
| 32 | 4 | txId | nula pro RX, identita TX operace pro TX |
| 36 | 4 | payloadBytes | přesná délka zbytku zprávy |

Příznaky: `0x0001 FIRST`, `0x0002 LAST`, `0x0004 DISCONTINUITY`, `0x0008
ABORT`. Neznámé bity jsou ve verzi 1 chyba protokolu.

`RX_ULAW` má jeden G.711 µ-law byte na vzorek. PCM payload je signed PCM16
little-endian; jeho délka musí být sudá. Endianita PCM je záměrně odlišná od
síťové hlavičky a musí být explicitně implementovaná přes `DataView`, ne přes
neověřený pohled hostitelského typu.

`firstSample` je autorita pro media time. Příchozí čas WebSocket zprávy slouží
jen k měření jitteru. Mezera v `firstSample` se doplní nulami a vyvolá
discontinuity; překrytý payload se ořízne, úplný duplikát se zahodí.

## Byte-level vektor

Hlavička `RX_ULAW`, flags `FIRST|DISCONTINUITY`, `streamId=0x01020304`,
`sequence=0x05060708`, 8 kHz, `firstSample=0x0000000100000002`,
`txId=0x0a0b0c0d` a payload `ff 7f 00`:

```text
41 55 44 31 01 01 00 05 00 28 00 00 01 02 03 04
05 06 07 08 00 00 1f 40 00 00 00 01 00 00 00 02
0a 0b 0c 0d 00 00 00 03 ff 7f 00
```

Příjemce musí odmítnout špatné magic/version, jinou délku hlavičky, nenulové
reserved, neznámý kind/flag, nulový sample rate, nesoulad `payloadBytes` a
lichou délku PCM.

## Bezpečné TX minimum

Samotný binární stream ani `tx-ready` nesmí zapnout PTT. `tx-ready(txId)` pouze
potvrzuje, že firmware ověřil požadavek a rezervoval omezený ring buffer; rádio
je stále PTT OFF. Klient začne posílat audio tempem 20 ms před cílovým slotem.
Firmware smí zaklíčovat teprve v cílovém slotu a pouze s kompletním minimálním
prebufferem. `LAST` ukončuje audio, `ABORT` zahodí buffer a všechny timeouty
nebo odpojení musí ve firmware skončit PTT OFF.

## TX řídicí zprávy prototypu

Každý modemový frame má vlastní nenulové `txId` a vlastní PTT cyklus. Klient
nejprve odešle textovou zprávu `tx.prepare` s poli `txId`, `sampleRate`,
`samples`, `packets`, `mode`, `toneHz`, `slotUtcMs`, `prebufferSamples` a
`packetMs=20`. Binární `TX_PCM16` smí začít odesílat až po odpovědi serveru
`tx-ready` se stejným `txId`, ale `tx-ready` není oprávnění k PTT.

Klient začne v `slotUtcMs - prebufferSamples/sampleRate`, posílá jeden 20ms
paket každých 20 ms a omezuje catch-up burst. Firmware mapuje browserový cílový
slot na svůj monotónní čas; přesnost této mapy musí být změřena při dummy-load
testu. Pokud v cílovém okamžiku chybí prebuffer, sekvence/`firstSample` nesedí,
ring přeteče nebo po PTT nastane underrun, celý `txId` přejde do fault a PTT OFF.

Produkční JS8LAN používá `prebufferSamples=48000` (1 s) a dovolí dohnat nejvýše
25 paketů (500 ms). Firmware po převodu na 8kHz uLaw drží 12288B ring (1,536 s),
takže krátký zásek mobilního prohlížeče nevede k vynechání slotu a současně
zůstává catch-up omezený. PTT se během plnění ring bufferu nezapíná.

První a poslední audio blok nesou příznaky `FIRST` a `LAST`. Firmware průběžně
hlásí `tx-state`; po fyzickém vyprázdnění audio bufferu odešle `tx-drained`.
Chyba se hlásí jako `tx-error`. Klient může přenos ukončit zprávou `tx.abort`;
firmware pak zahodí čekající audio a potvrdí stav s PTT OFF. Timeout, ztráta
WebSocketu, chybná sekvence, underrun i reconnect musí bez výjimky skončit PTT
OFF.

```text
QUEUED -> PREPARING -> WAITING_SLOT -> PREBUFFERING -> TRANSMITTING
             |              |              |                |
             +--------------+--------------+----------------+-> ABORT/FAULT
                                                                  -> PTT OFF
TRANSMITTING -> DRAINING -> COMPLETED -> PTT OFF
```

Vícerámcová zpráva zopakuje tento handshake pro každý frame a jeho následující
časový slot. Potvrzení jednoho `txId` nikdy neopravňuje odeslat další frame.

Izolovaný `Aud1TxGate` ověřuje capture, prebuffer, kontinuitu, omezenou velikost
bufferu, drain, abort, disconnect a underrun. Produkční převod PCM16/48 kHz na
rádiový 8kHz µ-law stream a měření slotového času na zařízení zůstávají součástí
dummy-load brány.
