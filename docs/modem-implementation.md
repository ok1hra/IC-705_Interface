# Implementace digitálních modemů pro stránku DATA

Tento manuál popisuje, jak přidat **dekodér (RX)** a **kodér (TX)** pro nový digitální
mód na webovou stránku **DATA** (audio vodopád). Modemy běží **v prohlížeči** (vanilla
JS), ne ve firmwaru — ESP32 přeposílá časované audio a vlastní bezpečné PTT.
Jednoduchý modem lze přidat přes registr v [data/data.js](../data/data.js).
Náročné slotované modemy mají samostatné assety a Worker; produkční JS8Call je
v souborech `data/js8-*`.

> JS8Call je zvláštní případ: vyžaduje metadata `AUD1`, sample counter,
> Web Worker/WASM a firmwarem vlastněný TX prebuffer. Pro další jednoduchý
> modem se firmware nemění, pokud mu stačí existující `AUD1` kontrakt.

---

## 1. Architektura a datový tok

```
IC-705 ──LAN audio (uLaw 8k)──► ESP32 ──WebSocket :83 /audiows──► prohlížeč
                                                                     │
                                    ┌────────────────────────────────┤
                                    ▼                                ▼
                             FFT → vodopád                    activeDecoder.pushSamples()
                                                                     │  onText()
                                                                     ▼
                                                              pole „Dekodér (RX)"

  pole „Kodér (TX)" ──► activeEncoder.encode(text) ──► onAudio(Float32) ──► (M3) ESP32 → radio
```

**Formát audio vzorků**, které modem dostává i produkuje:

| Vlastnost | Hodnota |
|---|---|
| Typ | `Float32Array`, hodnoty v rozsahu −1.0 … +1.0 |
| Kanály | mono |
| Vzorkovací frekvence | `sampleRate` (globální, aktuálně **8000 Hz**) |
| Velikost bloku (RX) | proměnná — ~512 vzorků z radia (slučované rámce), 256 ze syntetického zdroje |

> **Nikdy nepředpokládej pevnou velikost bloku ani pevný `sampleRate`.** Vždy čti
> `sampleRate` z konstruktoru a bufferuj si vzorky sám (viz příklad dekodéru).

---

## 2. Rozhraní (kontrakty)

Bázové třídy a registr jsou už v [data/data.js](../data/data.js). Nový modem je objekt
předaný do `registerModem()`:

```js
registerModem("myid", {
  label: "Můj mód",     // text v rozbalovacím menu
  Decoder: MyDecoder,   // třída odvozená od Decoder, nebo null
  Encoder: MyEncoder,   // třída odvozená od Encoder, nebo null
});
```

- `Decoder`/`Encoder` mohou být `null` — panel pak zobrazí „…zatím není implementován".
- Při výběru módu v menu se vytvoří nová instance (`new MyDecoder(sampleRate)`), takže
  konstruktor dostane aktuální vzorkovací frekvenci.

### 2.1 Dekodér (RX)

```js
class Decoder {
  constructor(sampleRate) { this.sampleRate = sampleRate; this._onText = null; this._onEvent = null; }
  pushSamples(float32, metadata) {} // metadata obsahují AUD1 časovou osu
  onText(cb) { this._onText = cb; return this; }
  onEvent(cb) { this._onEvent = cb; return this; }
  _emit(text) { if (this._onText) this._onText(text); }  // pošli dekódovaný text do RX pole
  reset() {}
}
```

- `pushSamples(float32)` — sem teče audio. Bufferuj a zpracovávej po svých rámcích.
- `this._emit("...")` — přidá text do pole „Dekodér (RX)". Volej klidně po znacích.

### 2.2 Kodér (TX)

```js
class Encoder {
  constructor(sampleRate) { this.sampleRate = sampleRate; this._onAudio = null; this.toneHz = 1500; }
  setToneOffset(hz) { this.toneHz = hz; }  // audio tón / střed pásma (TX marker ve vodopádu)
  encode(text) {}                          // -> vygeneruj audio a volej this._emit(...)
  onAudio(cb) { this._onAudio = cb; return this; }
  _emit(float32) { if (this._onAudio) this._onAudio(float32); }
}
```

- `setToneOffset(hz)` se volá automaticky, když uživatel klikne do vodopádu (nastaví TX
  tón). Ulož si ho a použij jako nosnou/střed pásma.
- `encode(text)` — vygeneruj `Float32Array` audia a předej přes `this._emit()`.

> **JS8 TX je asynchronní.** Prohlížeč posílá `tx.prepare` a očíslované 20ms
> `AUD1 TX_PCM16/48k` bloky. Firmware je převádí do ICOM uLaw/8k streamu,
> předbufferuje a jako jediný smí zaklíčovat PTT. Abort, výpadek WS, špatná
> kontinuita, overflow, underrun a watchdog vždy končí PTT OFF. UI navíc vyžaduje
> ruční potvrzení RF bezpečnosti a režim USB/USB-D.
>
> ⚠️ **Nutné nastavení rádia:** v IC-705 musí být **zdroj modulace nastaven na WLAN/síť**
> (Menu → SET → Connectors → MOD Input). Bez toho rádio sice zaklíčuje a audio přijme,
> ale nemoduluje ho → nulový výkon. (Ověřeno 2026-07-18.)

---

## 3. Příklad: minimální dekodér (detektor nosné)

Ukázka, kterou lze **spustit hned** — každých ~0.1 s zjistí nejsilnější tón v audiu a
vypíše jeho kmitočet. Demonstruje bufferování, frekvenční analýzu (Goertzel) i emisi textu.

```js
class CarrierDecoder extends Decoder {
  constructor(sampleRate) {
    super(sampleRate);
    this.buf = new Float32Array(2048);
    this.pos = 0;
    this.last = 0;
  }
  pushSamples(x) {
    for (let i = 0; i < x.length; i++) {
      this.buf[this.pos++] = x[i];
      if (this.pos >= this.buf.length) { this.pos = 0; this._analyze(); }
    }
  }
  _analyze() {
    // najdi nejsilnější kmitočet jednoduchým Goertzelem přes pásmo 300–2700 Hz
    let bestHz = 0, bestMag = 0;
    for (let f = 300; f <= 2700; f += 25) {
      const w = 2 * Math.PI * f / this.sampleRate;
      const c = 2 * Math.cos(w);
      let s0 = 0, s1 = 0, s2 = 0;
      for (let i = 0; i < this.buf.length; i++) { s0 = this.buf[i] + c * s1 - s2; s2 = s1; s1 = s0; }
      const mag = s1 * s1 + s2 * s2 - c * s1 * s2;
      if (mag > bestMag) { bestMag = mag; bestHz = f; }
    }
    const now = Date.now();
    if (bestMag > 5 && now - this.last > 500) {   // práh + rate-limit
      this.last = now;
      this._emit(`nosná ~${bestHz} Hz\n`);
    }
  }
}

registerModem("carrier", { label: "Detektor nosné", Decoder: CarrierDecoder, Encoder: null });
```

Vlož tenhle blok do [data/data.js](../data/data.js) (klidně těsně za existující
`registerModem(...)` řádky), regeneruj gzip a nahraj LittleFS (viz §6). V menu přibude
„Detektor nosné"; při otevření **`/data?test=1`** (bench režim bez radia) začne vypisovat
kmitočty přeladovaných tónů syntetického signálu.

---

## 4. Příklad: kostra kodéru (BFSK)

```js
class BfskEncoder extends Encoder {
  constructor(sampleRate) { super(sampleRate); this.baud = 45.45; this.shift = 170; }
  encode(text) {
    const sr = this.sampleRate, spb = Math.round(sr / this.baud);   // vzorků na bit
    const markHz  = this.toneHz + this.shift / 2;
    const spaceHz = this.toneHz - this.shift / 2;
    const bits = this._toBits(text);                                 // vlastní kódování
    const out = new Float32Array(bits.length * spb);
    let phase = 0, n = 0;
    for (const bit of bits) {
      const hz = bit ? markHz : spaceHz;
      const dphi = 2 * Math.PI * hz / sr;
      for (let i = 0; i < spb; i++) { out[n++] = 0.5 * Math.sin(phase); phase += dphi; }
    }
    this._emit(out);   // -> txStream: PTT + odeslání na radio po 20 ms rámcích
  }
  _toBits(text) { /* ... převod textu na bity dle protokolu ... */ return []; }
}

registerModem("bfsk", { label: "BFSK 45", Decoder: null, Encoder: BfskEncoder });
```

---

## 5. Užitečné globály a pomůcky v data.js

| Symbol | Význam |
|---|---|
| `sampleRate` | aktuální vzorkovací frekvence (Hz) |
| `FFT_SIZE`, `HOP_SIZE`, `BIN_COUNT` | parametry vodopádové FFT |
| `state.txToneHz` | aktuální TX tón (Hz), řídí ho klik do vodopádu |
| `setTxTone(hz)` | programově nastaví TX tón + marker |
| `registerModem(id, def)` | registrace módu |
| `appendDecoded(text)` | (interně) přidání textu do RX pole — dekodér volá `_emit` |

**Mapování osy X vodopádu:** kmitočet `f` je na vodorovné pozici `f / (sampleRate/2)`
(0.0 = 0 Hz vlevo, 1.0 = Nyquist vpravo). Hodí se, když chceš kreslit vlastní značky
pásma přes overlay (`wfOverlay`).

---

## 6. Nasazení a testování

1. Přidej třídy a `registerModem()`; velký modem ulož do samostatných assetů.
2. Ověř syntaxi: `node --check data/data.js`.
3. Regeneruj komprimované assety: `bash tools/gzip-assets.sh`.
4. Vytvoř deployment tree: `tools/prepare-spiffs-tree.sh data build/spiffs-data`.
5. LittleFS sestavuj z deployment tree, ne přímo z `data/`; JS8 zdroje a jejich
   Brotli kopie by jinak zabraly místo dvakrát.

Bezpečný upload bez přejmenování `data/`:

```sh
tools/upload-spiffs.sh                              # pouze sestaví a ověří
tools/upload-spiffs.sh --write --port /dev/ttyUSB0 # zapíše jen LittleFS
```

Skript čte offset i velikost přímo z nainstalovaného `no_ota.csv`. Pro ESP32
core 2.0.14 jde o offset `0x210000` a velikost `0x1E0000`; marketingový název
menu „2MB SPIFFS“ není přesná binární velikost oddílu. Oddíl si ponechává
historický název `spiffs`, ale firmware v něm používá LittleFS.

Doporučený běžný release workflow po prvním kompletním nahrání desky:

1. V Arduino IDE nastav `ESP32 Dev Module` a `No OTA (2MB APP/2MB SPIFFS)`.
2. Zvol `Sketch -> Export Compiled Binary`.
3. Zavři Serial Monitor a spusť jediný příkaz:

```sh
tools/upload-firmware-spiffs.sh --port /dev/ttyUSB0
```

Skript ověří magic a stáří exportované binárky, velikost `app0`, vynutí runtime
rezervu a sestaví LittleFS.
Před zápisem navíc přečte partition table z desky a vyžaduje přesnou shodu s
`no_ota.csv`; při neshodě nic nezapíše. Potom jedním voláním esptoolu zapíše
aplikaci i LittleFS. Bootloader, partition table, NVS a coredump zůstanou
nedotčené. Při změně partition schématu je nutné znovu provést plný upload přes
Arduino IDE.

JS8 assety se pro LittleFS nasazují jako gzip. DATA stránka běží z ESP32 přes
obyčejné HTTP, na kterém prohlížeče nemusí nabízet Brotli. Firmware navíc vybírá
kompresi jen z hodnot uvedených klientem v `Accept-Encoding`.

Výjimkou jsou dva největší soubory (`js8-jsc.bin.br` a
`js8-decoder.wasm.br`), které Worker stáhne jako obyčejná binární data a rozbalí
lokálním MIT Brotli WASM decoderem. Release build vyžaduje `p7zip-full`; čitelné
JS zdroje mají kontrolované minifikované protějšky generované příkazem
`tools/minify-spiffs-js.sh`.
5. Otevři `/data`, vyber svůj mód v menu.
   - **Bez radia:** otevři **`/data?test=1`** → dekodér dostává syntetické audio.
   - **S radiem (TRX1 v LAN):** audio teče z IC-705 (uLaw 8 kHz) automaticky po
     otevření stránky.

---

## 7. Poznámky a omezení

- **JS8 RX/TX je softwarově integrovaný.** RF TX zatím vyžaduje ověření na
  umělé zátěži; nezkoušej první přenos přímo do antény.
- **Vzorkovací frekvence 8 kHz** (pásmo do 4 kHz) je zvolená kvůli šetrnosti k
  jednovláknové smyčce ESP32. Pokud budoucí mód potřebuje víc, je nutné změnit
  `AUDIO_RX_CODEC`/`AUDIO_RX_SAMPLE` ve [icomLanClient.h](../icomLanClient.h) (např.
  LPCM16/16 kHz) a případně přidat FreeRTOS úlohu pro audio socket — viz
  [docs/icom-lan-implementace.md](icom-lan-implementace.md).
- **Náročnost dekodéru:** JS8 běží ve Workeru. Další drahé dekodéry tam dej také,
  aby neblokovaly waterfall a UI.
- **Codec musí sedět:** prohlížeč dekóduje uLaw tabulkou `ULAW[]`; když ve firmwaru
  změníš `AUDIO_RX_CODEC`, uprav i dekódování v `WsAudioSource.onmessage`.
