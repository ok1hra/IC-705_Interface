# IC-705 Interface — TRX HTTP API

Popis komunikačního protokolu mezi webovým logem a TRX zařízením.
Dokument slouží jako referenční specifikace pro implementaci API adaptéru do TRX2 a TRX3.

---

## 1. Přehled architektury

```
  Browser (log.html)
       │
       │ HTTP přes WiFi (port 80)
       ▼
  ESP32 firmware  ←── webServer na port 80 ──→  IC-705 (Bluetooth CI-V)

  Pro TRX2 / TRX3:
  Browser (log.html)
       │
       │ HTTP přes WiFi (libovolný port)
       ▼
  Vlastní zařízení / adapter  ←──→  druhý/třetí TRX (jakýkoliv transport)
```

Web log komunikuje výhradně přes **HTTP na port 80** vůči IP adrese ESP32.
Pro TRX2 a TRX3 musí každé zařízení implementovat **identické HTTP API** na své IP adrese.
Přepínání probíhá přímo v browseru — podle aktivního TRX se mění base URL požadavků.

---

## 2. Endpointy API

### 2.1 `GET /state` — stav TRX (polling)

Klient volá každých **250 ms** při spojení, **1000 ms** při chybě.

**Požadavek:**
```
GET /state HTTP/1.1
Host: <ip-zarizeni>
```

**Odpověď:** `200 OK`, `Content-Type: application/json`

```json
{
  "connected":       true,
  "btStatus":        "BT linked",
  "wifiStatus":      "WiFi STA",
  "wifiRssi":        -67,
  "fwRev":           "42",
  "power":           true,
  "frequency":       14074000,
  "mode":            "USB",
  "filter":          1,
  "radioAddress":    "0xA4",
  "transceiverType": "IC-705-BT",
  "tx":              false,
  "ritRaw":          0,
  "smeterRaw":       0,
  "powerMeterRaw":   0,
  "afGain":          0,
  "keySpeed":        138,
  "rfPower":         205,
  "supplyVolts":     13.80,
  "swr":             1.0,
  "preamp":          0,
  "vox":             0
}
```

#### Klíčová pole pro log

| Pole | Typ | Popis |
|------|-----|-------|
| `connected` | bool | TRX je dostupný a odpovídá na CI-V/CAT |
| `frequency` | uint32 | Aktuální frekvence v Hz |
| `mode` | string | Aktuální mód (viz seznam níže) |
| `tx` | bool | TRX právě vysílá |
| `fwRev` | string | Verze firmware — zobrazuje se v topbaru |
| `power` | bool | TRX je zapnutý |

#### Platné hodnoty `mode`

`LSB`, `USB`, `AM`, `CW`, `CW-R`, `RTTY`, `RTTY-R`, `FM`, `DV`

#### Chování klienta

- Pokud požadavek selže (síťová chyba nebo non-2xx), log přejde do stavu *disconnected*
  a zachová poslední ručně zadané hodnoty frekvence a módu.
- Pole `connected: false` při dostupném serveru = TRX je připojen k adaptéru, ale radio nereaguje.

---

### 2.2 `POST /cmd` — příkazy do TRX

**Požadavek:**
```
POST /cmd HTTP/1.1
Host: <ip-zarizeni>
Content-Type: application/json

{ "type": "<typ>", ... }
```

**Úspěšná odpověď:** `200 OK`
```json
{ "ok": true }
```

**Úspěšná odpověď pro nepodporovaný příkaz:** `200 OK`
```json
{ "ok": true }
```

Adapter vrátí `{"ok": true}` i pro příkazy, které nepodporuje nebo tiše ignoruje — viz sekce [2.5 Nepodporované příkazy](#25-nepodporované-příkazy).

**Chybové odpovědi:**

| HTTP status | JSON | Situace |
|------------|------|---------|
| 503 | `{"error":"radio_disconnected"}` | TRX není připojen nebo nereaguje |
| 400 | `{"error":"invalid_frequency"}` | Frekvence mimo rozsah nebo nulová |
| 400 | `{"error":"invalid_mode"}` | Neznámý název módu |
| 400 | `{"error":"invalid_hex"}` | Špatný hex payload v `civ.raw` |
| 400 | `{"error":"missing_text"}` | Chybí text u `sendCw` |
| 400 | `{"error":"empty body"}` | Prázdné tělo požadavku |
| 500 | `{"error":"tx_failed"}` | Příkaz se nepodařilo odeslat do TRX |

---

#### `abortCw` — okamžitě přerušit probíhající CW nebo RTTY/FSK vysílání

```json
{ "type": "abortCw" }
```

- **Tento příkaz nevyžaduje aktivní BT spojení** — zpracuje se i při `radio_disconnected` stavu, protože přerušení RTTY závisí na přímém GPIO, nikoli na CI-V.
- Mód `CW` / `CW-R` → CI-V příkaz `0x17 0xFF` (datový bajt `FF` zastaví CW keyer IC-705 dle dokumentace: *"FF stops sending CW messages"*); frame: `FE FE A4 E0 17 FF FD`
- Mód `RTTY` / `RTTY-R` → nastaví volatile příznak `abortFskTransmission`; FSK smyčka ho detekuje na konci aktuálního znaku (max. 165 ms), PTT okamžitě vypne, tail delay se přeskočí
- Jiné módy → příkaz se přijme a tiše ignoruje (`{"ok": true}`)
- Adapter pro TRX2/3 by měl příkaz podpořit; pokud ho nepodporuje, vrátí `{"ok": true}` a ignoruje ho
- Volá se z logu při stisku `Esc` bez otevřeného dialogu

---

#### `sendCw` — odeslat CW nebo RTTY/FSK text

```json
{ "type": "sendCw", "text": "CQ TEST DE OK1HRA" }
```

- Firmware routuje automaticky podle aktuálního módu:
  - mód `CW` / `CW-R` → CI-V příkaz `0x17` (CW keyer)
  - mód `RTTY` / `RTTY-R` → FSK keying přes GPIO + PTT
- Text je ASCII, max. délka závisí na bufferu zařízení (ESP32 implementace: `sizeof(CwMsg) - 1`)
- Prázdný text → `400 missing_text`

Příklady volání z logu:
```
CQ TEST DE OK1HRA OK1HRA TEST    ← makro CQ
OK1ABC 5nn TT1                   ← makro TXEXCH (číslo QSO)
tu OK1HRA                        ← makro TU
```

---

#### `setFrequency` — nastavit frekvenci VFO

```json
{ "type": "setFrequency", "frequency": 14074000 }
```

- `frequency` — celé číslo v Hz, kladné nenulové
- ESP32 implementace: CI-V příkaz `0x05` s BCD kódováním 5 bajtů

---

#### `setMode` — nastavit mód a šířku filtru

```json
{ "type": "setMode", "mode": "USB", "filter": "FIL1" }
```

| Parametr | Hodnoty | Popis |
|----------|---------|-------|
| `mode` | `LSB`, `USB`, `AM`, `CW`, `CW-R`, `RTTY`, `RTTY-R`, `FM`, `DV` | Požadovaný mód |
| `filter` | `FIL1` (výchozí), `FIL2`, `FIL3` | Šířka filtru |

- ESP32 implementace: CI-V příkaz `0x06`; `FIL1` = wide, `FIL2` = medium, `FIL3` = narrow

---

#### `setRitClear` — vynulovat RIT

```json
{ "type": "setRitClear" }
```

Voláno automaticky po každém zalogovaném QSO (viz [app.js:648](../data/app.js#L648)).

- ESP32 implementace: CI-V příkaz `0x21 00 00 00 00 00` (RIT clear)

---

#### `civ.raw` — raw CI-V průchod (volitelné)

```json
{
  "type":        "civ.raw",
  "data":        "03",
  "framed":      false,
  "expectReply": true,
  "expectAck":   false
}
```

| Pole | Typ | Popis |
|------|-----|-------|
| `data` | hex string | CI-V payload bez rámce; ESP32 doplní `FE FE <addr> E0 ... FD` |
| `framed` | bool (opt.) | `true` = `data` obsahuje kompletní CI-V rámec včetně `FE FE ... FD` |
| `expectReply` | bool (opt.) | Čekat na datovou odpověď od TRX |
| `expectAck` | bool (opt.) | Čekat na ACK/NACK |

Toto je rozšířené rozhraní pro přímé CI-V operace. Pro základní log stačí `sendCw`, `setFrequency`, `setMode` a `setRitClear`.

---

### 2.5 Nepodporované příkazy — chování adapteru a klienta

Různá zařízení mají různé schopnosti. Pravidlo je jednoduché:

**Adapter vrátí `HTTP 200` + `{"ok": true}` pro každý příkaz, který tiše ignoruje.**

Klient nerozlišuje "provedeno" od "ignorováno" — v obou případech pokračuje normálně.
Adapter **nesmí** vracet chybový status pro nepodporované příkazy, protože by to narušilo chování klienta.

#### Chování klienta pro jednotlivé příkazy

| Příkaz | Co klient udělá při `r.ok === false` | Doporučení pro adapter |
|--------|--------------------------------------|------------------------|
| `sendCw` | zobrazí hint **"Send failed"** | vrátit `{"ok": true}` a text zahodit |
| `abortCw` | tiše ignoruje (`.catch(() => {})`) | vrátit `{"ok": true}`; ideálně zastavit aktivní TX |
| `setRitClear` | tiše ignoruje (`.catch(() => {})`) | vrátit `{"ok": true}` |
| `setFrequency` | tiše ignoruje | vrátit `{"ok": true}` |
| `setMode` | tiše ignoruje | vrátit `{"ok": true}` |
| `civ.raw` | tiše ignoruje | vrátit `{"ok": true}` |

#### Příklad minimálního handleru v adapteru

```python
# Python / Flask příklad
@app.post("/cmd")
def cmd():
    body = request.get_json(silent=True) or {}
    t = body.get("type", "")

    if t == "sendCw":
        text = body.get("text", "")
        if not text:
            return {"error": "missing_text"}, 400
        my_keyer_send(text)          # vlastní implementace
        return {"ok": True}

    if t == "setRitClear":
        my_rit_clear()               # nebo nic, pokud nepodporováno
        return {"ok": True}

    # Všechny ostatní typy — tiše přijmout, neudělat nic
    return {"ok": True}
```

#### Kdy vrátit chybu

Chybový status (`4xx`, `5xx`) má smysl pouze tehdy, když:
- tělo požadavku je prázdné nebo nevalidní JSON → `400 empty body`
- TRX je fyzicky nedostupný a příkaz nelze splnit ani částečně → `503 radio_disconnected`
- interní chyba adapteru → `500 tx_failed`

**Nikdy nevracet `400 unsupported_type`** pro příkazy, které adapter nezná — klient by zobrazil chybový hint.

---

### 2.3 `GET /log-config` — konfigurace logu

```
GET /log-config HTTP/1.1
```

**Odpověď:** `200 OK`, `Content-Type: application/json`
Obsah: libovolný JSON objekt uložený klientem (nebo `{}` pokud prázdný).

Relevantní pole, která log zapisuje a čte:
```json
{
  "trx1Label": "IC-705",
  "trx2Label": "FT-991",
  "trx3Label": "SDR"
}
```

---

### 2.4 `POST /log-config` — uložení konfigurace logu

```
POST /log-config HTTP/1.1
Content-Type: application/json

{ "trx1Label": "IC-705", "trx2Label": "FT-991", "trx3Label": "SDR" }
```

**Odpověď:** `{"ok": true}`

- Zařízení uloží JSON as-is (ESP32 do SPIFFS `/log-config.json`)
- Validace: neprázdný JSON objekt, max. 2048 B

---

## 3. Legacy rozhraní (zachovat pro kompatibilitu)

### 3.1 HTTP CAT port (výchozí 81)

Jednoduchý HTTP server pro externí logery (N1MM+, Win-Test):

```
GET http://<ip>:81/ HTTP/1.1
```

**Odpověď:** plain text
```
14074000|USB|
```
nebo při vypnutém TRX:
```
0|OFF|
```

Formát: `<frekvence_Hz>|<mód>|`

---

### 3.2 UDP CW/FSK port (výchozí 89)

Přijímá ASCII text, odesílá jako CW nebo FSK.

```bash
echo -n "cq de ok1hra;" | nc -u -w1 192.168.1.x 89
```

- Každý paket = jeden CW/FSK text k odeslání
- `;` na konci není vyžadováno, ale bývá zvykem (konvence N1MM+)
- Routování CW vs. FSK podle aktuálního módu TRX (stejné jako `sendCw`)

---

### 3.3 UDP CAT port (výchozí 90)

Přijímá raw CI-V bajty (binárně), z bezpečnostních důvodů je implementován pouze příkaz clear RIT:

```
Byte[0] = 0x21  →  odeslat clear RIT do TRX
```

---

## 4. Minimální implementace pro TRX2 / TRX3

Aby webový log plně fungoval s druhým nebo třetím zařízením, musí adapter implementovat tyto endpointy:

| Endpoint | Metoda | Priorita |
|----------|--------|----------|
| `/state` | GET | **povinné** — bez tohoto log neví nic o stavu TRX |
| `/cmd` s `sendCw` | POST | **povinné** — makra CQ, TXEXCH, TU |
| `/cmd` s `setRitClear` | POST | doporučené — volá se po každém QSO |
| `/cmd` s `setFrequency` | POST | volitelné — jen pokud zařízení umí nastavit VFO |
| `/cmd` s `setMode` | POST | volitelné |
| `/log-config` | GET + POST | volitelné — lze vrátit `{}` |

---

## 5. Rozšíření klienta pro více TRX

V současné implementaci (`app.js`) je `activeTrx` jen UI label — veškerý HTTP provoz jde vždy na stejnou IP (ESP32). Pro skutečné přepínání TRX je potřeba:

1. Přidat do `/log-config` mapování TRX → IP adresa:
   ```json
   {
     "trx1Label": "IC-705",  "trx1Url": "",
     "trx2Label": "FT-991",  "trx2Url": "http://192.168.1.50",
     "trx3Label": "SDR",     "trx3Url": "http://192.168.1.51"
   }
   ```
   Prázdné `trxUrl` = použij lokální ESP32 (výchozí chování).

2. `pollState()` a `handlePostCmd()` v `app.js` cílí na `trxBaseUrl()` podle `app.activeTrx`.

3. Přepnutí TRX → okamžitý nový poll `/state` na novou adresu.

---

## 6. Souhrn CI-V příkazů implementovaných v ESP32

| CI-V cmd | Hex | Funkce |
|----------|-----|--------|
| Read frequency | `0x03` | Čtení aktuální frekvence VFO |
| Read mode | `0x04` | Čtení aktuálního módu a filtru |
| Set frequency | `0x05` | Nastavení frekvence VFO (5 BCD bajtů) |
| Set mode | `0x06` | Nastavení módu a filtru |
| Send CW | `0x17` | Odeslání CW textu keyer bufferem |
| Stop CW | `0x17 0xFF` | Okamžité přerušení aktuálně odesílané CW zprávy (`FF` = stop) |
| RIT clear | `0x21 00 00 00 00 00` | Nulování RIT offsetu |
| CI-V transceive | `1A 05 01 31 01` | Zapnutí push notifikací ze staničky |
| Quick split | `1A 05 00 45` | Čtení split stavu |

CI-V rámec má strukturu: `FE FE <radio_addr> E0 <cmd> [<payload>] FD`

- `FE FE` = start bytes
- `<radio_addr>` = adresa IC-705, typicky `0xA4` (konfigurováno v SETUP)
- `E0` = adresa controlleru (ESP32)
- `FD` = stop byte

---

---

## 7. Změny SETUP API (2026-05-20)

### `/setup-data.json` — nová pole

| Pole | Typ | Popis |
|------|-----|-------|
| `ipLastOctet` | uint8 | Poslední oktet IP adresy zařízení (0 v AP módu). Slouží k návrhu hodnoty `Own NET_ID` v TrxNet sekci. |
| `trxnetidIsDefault` | bool | `true` pokud EEPROM byte 41 nebyl nikdy uložen (factory default) — frontend pak automaticky vyplní `trxnetid` z `ipLastOctet`. |
| `trx2conntype` | uint8 | Typ připojení TRX2: `0` = TrxNet, `1` = CI-V. Uloženo v EEPROM byte 44. |
| `trx3conntype` | uint8 | Typ připojení TRX3: `0` = TrxNet, `1` = CI-V. Uloženo v EEPROM byte 47. |

### `/config/download` a `/config/upload` — nová pole

| Pole | Typ | Popis |
|------|-----|-------|
| `trx2conntype` | uint8 | Typ připojení TRX2 (0=TrxNet, 1=CI-V). |
| `trx3conntype` | uint8 | Typ připojení TRX3 (0=TrxNet, 1=CI-V). |

*Poznámka: CI-V připojení pro TRX2/3 (conntype=1) není v aktuálním firmware implementováno — volba je uložena v EEPROM a připravena pro budoucí release.*

---

*Aktualizováno 2026-05-20*
