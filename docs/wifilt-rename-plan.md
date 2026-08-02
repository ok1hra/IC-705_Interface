# WIFILT — přejmenování projektu

**Dohodnuto 2026-08-02.** IC-705 IP interface → **WIFILT — Web interface for Icom LAN
Transceivers**. Důvod: projekt se po přidání autodetekce typu rádia (`1588f01`) už neomezuje
na IC-705.

Tento dokument je implementační kontrakt. Rozhodnutí v něm padla v grilovací session, ne při
psaní kódu — pokud se něco ukáže jinak, opravit **tady**, ne potichu v commitu.

---

## 1. Pět vrstev, ne jedna náhrada

`IC-705` se v repu vyskytuje v pěti významech s naprosto různým rizikem. Bez tohoto dělení
je „nahraď IC-705 za WIFILT" destruktivní operace.

| Vrstva | Co to je | Příklad | Verdikt |
|---|---|---|---|
| **1** | název projektu | `<h1>IC-705 IP interface</h1>`, sériový banner | **měnit** |
| **2** | síťová identita (viditelná i funkční) | `deviceHostname="ic705"`, SSID `IC705-if`, `ic705-config.json`, ADIF `PROGRAMID` | **měnit** (viz §3) |
| **3** | identifikátory v kódu | `transceiverType=="IC-705-LAN"`, localStorage `ic705.*`, BroadcastChannel, IndexedDB | měnit, ale odděleně (viz §4) |
| **4** | **fakt o rádiu, ne o projektu** | `"IC-705": 10` (W), `trxLabels`, „Connect the IC-705 to WLAN", `IC-705 → 0xA4` | **nesahat** |
| **5** | citace zdrojáku | `[IC-705_Interface.ino:6644](../IC-705_Interface.ino#L6644)` — 82× v `docs/` | mechanicky s přejmenováním sketche |

Cizí kód (`wfview-master/`, `mercury/…/hamlib-*/`, `JS8Call-improved-master/`) se nesahá vůbec.

### Pravidlo pro vrstvu 4 — jak UI pojmenovává rádio

Zákaz: **UI nesmí odkazovat na typ, který není připojený.**

| Stav | Co UI píše |
|---|---|
| připojeno | `state.radioName` — co rádio samo hlásí v caps paketu; nikdy hardcoded |
| nepřipojeno, model zapamatován | `trx{N}model` z `/setup-data.json` |
| nepřipojeno, nic známého | zástupné `TRX1/2/3` (+ label operátora) |
| lookup tabulky | klíče zůstávají modely (`IC-705`, `IC-7610`, …) |

Opravit při tom: `data.js:4472` `setRadioPower(…, radioName="IC-705")` → default `""`.
Hardcoded default modelu v test hooku je přesně ten zakázaný případ.

Autodetekce, na které to stojí:
- [`icomLanClient.h:187`](../icomLanClient.h#L187) `radioModelName()` — z caps, prázdné dokud caps nedorazí
- [`IC-705_Interface.ino:4505`](../IC-705_Interface.ino#L4505) `radioModelLearnTick()` — učí **jen přes LAN**
- `radioSlots[slot].model` — persistentní přes reboot; `/state` → `radioName`, `/setup-data.json` → `trx{N}model`

---

## 2. Jmenné tvary

**Plný tvar `WIFILT — Web interface for Icom LAN Transceivers`** na pěti místech prvního
kontaktu: SETUP `<h1>`, README první řádek, titul manuálu, `<h1>` flasher stránky, sériový
banner. Nikde jinde.

**Titulky stránek: `<Stránka> — WIFILT`** na *všech* sedmi stránkách, včetně těch tří, které
dnes jméno projektu nenesou (`WSPR beacon`, `JS8Call-ICOM`, `DXC`).

Rozlišující token je vpředu záměrně: `DXC` a `/log` se otevírají jako samostatná okna, operátor
má běžně 3–4 taby jednoho zařízení, a prohlížeč zkracuje `<title>` od konce. `WIFILT · Setup`
by dalo čtyři nerozlišitelné taby.

**README dostane nový titul, ne přepsaný starý.** `ESP32 QRPlog for IC-705` je zavádějící
dvakrát — QRPlog je jedna ze sedmi stránek a IC-705 už není jediné rádio.

**Sériový banner, dvouřádkově** (jednořádkově má 101 znaků a zalomí se na 80-kolonové konzoli
doprostřed názvu; dnešní má 71):

```
-------- DivaDroid International | WIFILT status --------
  WIFILT - Web interface for Icom LAN Transceivers
```

Dnešní stav pro srovnání — projekt **žádné kanonické jméno nemá**, šest variant:
`IC-705 IP interface` (SETUP h1, banner), `IC-705 IP Interface` (manuál), `IC-705 Interface`
(flasher, notices), `IC-705_Interface` (repo, sketch, Changelog), `ESP32 QRPlog for IC-705`
(README).

---

## 3. Síťová identita — čistý řez, bez aliasů

Nasazená zařízení neexistují, projekt je ve fázi vývoje. Žádné přechodové aliasy nikde.

| Dnes | Nově |
|---|---|
| `deviceHostname = "ic705"` | `"wifilt"` |
| SSID `IC705-if` | `WIFILT-AP` |
| `ic705-config.json` | `wifilt-config.json` |
| ADIF `PROGRAMID` `IC705-Log` | `WIFILT-Log` |
| `setup.html:448` fallback `\|\| 'ic705'` | `\|\| 'wifilt'` |
| `ic705.test` ve 3 smoke harnessech | `wifilt.test` |

`deviceHostname` je jeden string pro tři lookup cesty — DHCP hostname (`http://wifilt/`),
mDNS (`http://wifilt.local`) a AP portál. Komentář na
[`IC-705_Interface.ino:300`](../IC-705_Interface.ino#L300) říká proč: operátor píše jedno
jméno bez ohledu na to, který mechanismus jeho síť umí. **Nezavádět druhé jméno** — dvě jména
znamenají dvě jména v dokumentaci navždy. (Šlo by to jen sestupem na IDF
`mdns_delegate_hostname_add()`; Arduino `ESPmDNS` umí jeden hostname. Zamítnuto.)

Ověřeno: restore configu kontroluje jen **suffix** (`file name must end with config.json`,
[`setup.html:1130`](../data/setup.html#L1130)), takže změna prefixu projde.

### Změna hostname je změna originu — MUSÍ jít do Changelogu

`http://ic705.local` → `http://wifilt.local` je z pohledu prohlížeče **jiný origin**. Ztratí se
veškerý browser-side stav bez ohledu na jména klíčů:

- `contestLogDb` — **celý QSO log** ([`log-db.js:10`](../data/log-db.js#L10))
- `datasyncDb`, `ic705-js8-file-transfers`
- pět localStorage klíčů (`ic705.wspr.v1`, `ic705.data.js8-settings`, `…js8-email-gateways.v1`,
  `…js8-aprs-recent.v1`, `…trx-help-seen.v1`)

Do Changelogu k této revizi patří: **„export the log from `http://ic705.local` before
updating"** (LOGSYNC / `/datasync`).

Mírnící okolnost: origin je fragmentovaný už dnes — `ic705.local`, `ic705/` a `192.168.1.x` jsou
tři různé originy. Kdo bookmarkoval po IP, nic nepocítí. Rename přidává čtvrtý, nezavádí problém.

**Důsledek pro vrstvu 3:** protože se stav ztratí tak či tak, jsou jména klíčů zdarma → přejmenovat.
`contestLogDb` a `datasyncDb` prefix `ic705` nemají, netýká se jich to. BroadcastChannel
`ic705-dxc-spots` / `ic705-dxc-action` mají oba konce v repu ([`dxc.html`](../data/dxc.html#L7) ↔
[`log.js:2336`](../data/log.js#L2336)) — přejmenovat zároveň.

---

## 4. `transceiverType` → `"ICOM-LAN"` (samostatný commit)

Ověřeno, že se **nikdy nezobrazuje**: dropdown v SETUPu má `value="lan"` a label `ICOM-LAN`
([`setup.html:141`](../data/setup.html#L141)) — už dnes model-neutrální. `"IC-705-LAN"` žije jen
jako interní `transceiverType`, ukládá se jako `trxprofile`, porovnává se v JS na 6 místech.

Původně vrstva 3 → nesahat. Předpoklady se změnily: žádná nasazená zařízení + UI ten stav už
jmenuje `ICOM-LAN`. Jméno dnes lže — komentář ve
[`wspr-core.js:300`](../data/wspr-core.js#L300) na past sám upozorňuje.

Samostatný commit, aby se dal revertovat bez rozbití brandingu.

**Provedeno — a migrační kód nebyl potřeba.** Plán počítal s „přijímat i starou hodnotu", ale
všechny čtecí cesty už dnes na LAN hodnotu *normalizují* přes else-branch, takže se stará
hodnota nikde nematchuje pozitivně a sama spadne na novou konstantu:

- [`:893`](../IC-705_Interface.ino#L893) `if (type=="IC-7610-CI-V" || type=="TRXNET") {keep} else {LAN}`
- [`:2333`](../IC-705_Interface.ino#L2333) derivuje z EEPROM transport bajtu, ne ze stringu
- [`:2411`](../IC-705_Interface.ino#L2411) derivuje z `radioSlots[0].transport` enumu
- [`:2849`](../IC-705_Interface.ino#L2849) `else if (trx.length()>0) → LAN` (restore z backupu)

**Zbývá stejná vada u CI-V:** `"IC-7610-CI-V"` pojmenovává generický CI-V transport podle
jednoho modelu rádia. Na rozdíl od LAN se ale **matchuje pozitivně** na čtyřech místech výše,
takže jeho přejmenování na `"ICOM-CIV"` migraci potřebuje. Mimo rozsah přejmenování.

---

## 5. Trademark notice

> Icom is a registered trademark of Icom Incorporated. WIFILT is an independent software project
> and is not affiliated with, endorsed by, or sponsored by Icom Incorporated.

Čtyři místa: `README.md`, `docs/user-manual.md`, `data/THIRD-PARTY-NOTICES.txt` (v zařízení; už
se servíruje, dnes začíná `IC-705 Interface — third-party software notices`), patička gh-pages
flasheru.

**Plus oprava:** `setup.html` **nemá patičku vůbec** (stejně jako `log.html`, `bd.html`,
`datasync.html`) — odkaz na `/THIRD-PARTY-NOTICES.txt` mají jen
[`data.html:419`](../data/data.html#L419) a [`wspr.html:277`](../data/wspr.html#L277). SETUP je
přitom jediná stránka s plným názvem včetně taglinu a v AP režimu první, co nový uživatel vidí.
Přidat mu stejnou patičku `GitHub | Licenses`, jakou má `data.html`.

Hodnota disclaimeru stojí na taglinu: „Web interface for Icom LAN Transceivers" je popisné
užití cizí známky, disclaimer musí být dosažitelný odtud, kde tagline stojí. Právní věta se
**nedává** pod `<h1>` do WiFi formuláře — na čtyřpalcovém telefonu jen zabírá místo.

`log.html`, `bd.html`, `datasync.html` zůstávají bez patičky — tagline nenesou.

---

## 6. Dokumentace

Z 248 zásahů v `docs/*.md` je **samostatné rozhodnutí jen u ~4**:

| Kategorie | Počet | Řídí se |
|---|---|---|
| citace zdrojáku `IC-705_Interface.ino:1234` | 82 | přejmenováním sketche (§7 commit 1) |
| hostname URL | 11 | §3 |
| kódové identifikátory `IC-705-LAN`, `TRX_IC705*` | 26 | §4 |
| **próza o rádiu** (`IC-705 → 0xA4`, „OVĚŘENO PROTI SKUTEČNÉMU IC-705") | **~125** | **zůstává** (vrstva 4) |
| próza o projektu / hostname | ~4 | mění se |

82 citací nese čísla řádků a přejmenování souboru je neposune → odkazy zůstanou platné.

**Plná péče (próza)** — 6 dokumentů: `user-manual.md`, `find-device-ip.md`, `trxnet.md`,
`trx-http-api.md` (uživatelské) + `how-to-regenerate-manual.md`, `how-to-bugfix.md` (procedury,
které se znovu spouští).

**Jen mechanické opravy** — zbývajících 17 (`*-implementace.md`, `*-plan.md`, `*-redesign.md`,
audity). Jsou to datované záznamy s razítky typu „OVĚŘENO 2026-07-14"; přepsat v nich prózu
znamená zfalšovat log toho, co se kdy zjistilo. Odkazy a URL v nich opravit **musím**, jinak
zbydou rozbité odkazy.

Mrtvý papír, neplést s živým kódem: `"ic705if"` existuje **jen** jako navrhovaný snippet
v [`icom-lan-implementace.md:293`](icom-lan-implementace.md#L293), nikdy se to neimplementovalo
(`lanClient.begin()` žádný identity string nebere).

`docs/how-to-regenerate-manual.md:107` generuje screenshoty z `http://wifilt.local/setup`
a hostname je v manuálu na 6 místech → **regenerace všech screenshotů manuálu**.

---

## 7. Pořadí commitů

`REV` bump z dnešního `20260802`.

| # | Commit | Poznámka |
|---|---|---|
| 1 | adresář + `IC-705_Interface.ino` → `WIFILT.ino` | **samostatně** — zneplatní build cache a cwd. Smazat `prototype/*/build*` (absolutní cesty). Opravit `gh-pages.sh` (4×), `upload-firmware-spiffs.sh` (5×), `extract_sketch_aud1.py`, 82 citací v docs. Arduino váže jméno `.ino` na jméno adresáře. |
| 2 | síťová identita | §3 celá + klíče vrstvy 3 |
| 3 | branding + trademark | §2, §5, `RADIO_FULL_POWER_W` += `"IC-7760": 200`, **regenerace assetů** |
| 4 | `hw/` + `3Dprint/` | přejmenovat soubory, vytlačený text `WIFILT` na [`ic-705-interface-3.scad:138`](../3Dprint/ic-705-interface-3.scad#L138), regenerovat STL/3MF/`preview*.png` |
| 5 | repo + Pages URL | až po skutečném přejmenování na GitHubu: `fw-version.js` (`MANIFEST_URL`, `FLASHER_URL`), 22 README odkazů, patičky 6 stránek |
| 6 | `transceiverType` → `"ICOM-LAN"` | §4, revertovatelné |
| 7 | multi-model nápověda | §8 — **stejný REV**, ne „někdy potom" |

`hw/` a `3Dprint/` jsou pojmenované po **projektu**, ne po rádiu — `ic-705-interface-3.scad`
je „IC-705 interface PCB box", krabička na interface PCB, ne držák tvarovaný pro rádio. Patří
tedy do vrstvy 1.

### Past: předkomprimované assety

Firmware servíruje **`.br` → `.gz` → plain**
([`IC-705_Interface.ino:2117`](../IC-705_Interface.ino#L2117)). `.gitignore` ignoruje
`/data/*.br` a `/data/*.js.min`, ale **14 `.gz` je trackovaných**.

Nejpravděpodobnější způsob, jak si tohle přejmenování „nebude fungovat": upravím
`data/setup.html`, zařízení dál servíruje `setup.html.br` se starým stringem. Lokálně se to
chová **hůř** než v čerstvém klonu — tam `.br` vůbec není a spadne se na `.gz`.

Pořadí: `tools/minify-spiffs-js.sh` (hlídá `.minified-js.sha256`) → `tools/gzip-assets.sh` →
`tools/build-js8-assets.sh`. Commit musí obsahovat **regenerované `.gz`**.

### Verifikace: povolený seznam zbytků

Mrtvý kód se **nechává ležet** → rename sweep ho musí explicitně vynechat, jinak ho globální
replace smete. Bez tohoto seznamu bude grep `ic705` hlásit falešné nálezy navždy:

- `TRX_IC705_SEL` — [`IC-705_Interface.ino:1496`](../IC-705_Interface.ino#L1496), zbytek po BT selektoru
- `IC-705-BT` — tamtéž
- `IC705-%02X%02X%02X` — uvnitř `#if defined(BLUETOOTH)`, a `#define BLUETOOTH` je zakomentovaný na [řádku 115](../IC-705_Interface.ino#L115)
- `prototype/*/build*` CMake artefakty s absolutními cestami (mazané v commitu 1)
- `wfview-master/`, `mercury/…/hamlib-*/`, `JS8Call-improved-master/`
- ~125 prozaických zmínek vrstvy 4 v `docs/`
- **tento dokument.** Zaplaceno chybou při commitu 3: sweep `ic705.local → wifilt.local`
  přes `docs/*.md` zasáhl i plán, a protože plán je z poloviny *tabulka starý→nový*, zbyly
  z něj řádky jako `wifilt-config.json → wifilt-config.json` a z varování „export the log
  from `http://ic705.local`" bezcenné „z `wifilt.local`". Dokument **o** přejmenování musí
  být ze sweepu vyňatý, jinak si smaže vlastní obsah. Totéž platí pro záznam v `Changelog.md`.

---

## 8. Multi-model nápověda (7. krok, stejný REV)

Nahrazuje dnešní IC-705-only dialog. Ručně přepínatelná mezi typy, **otevře se vždy detekovaný**.

### Struktura: jedna šablona + jedna tabulka, ne pět textů

Dialog vytáhnout z `data.html` (dnes duplikovaný ve `wspr.html`) do sdíleného
**`data/trx-help.js`**, který si nese vlastní markup i CSS a na stránku se přidá jedním
`<script>` + jedním voláním — precedent [`lan-gate.js`](../data/lan-gate.js#L10)
a `wake-lock.js`.

Osm kroků má na všech pěti rádiích **stejnou semantiku** (připrav stanici → připoj na síť →
zapni Network Control → nasměruj RX audio → nastav data-mode hodnoty → ulož/nahraj → ověř
režim → dokonči nastavení stránky). Liší se dosazené hodnoty.

Jedna tabulka `ICOM_MODELS` (`number, label, watts, transport, menuRoot, hasPreset, civAddr,
bands`) **sloučená s `RADIO_FULL_POWER_W`**, která už z poloviny existuje. Pět esejí by se
rozešlo — až se změní doporučení pro `MOD Level`, opraví se tři z pěti. A sloučením nemůže
`IC-7760` chybět, protože watty a návod jsou jeden záznam.

| Rádio | Č. | Transport | Menu root | PRESET | Full power |
|---|---|---|---|---|---|
| IC-705 | 705 | WLAN | `SET → WLAN Set` | ano | 10 W |
| IC-7300MK2 | 7300 | LAN | `SET → Network` | ano | 100 W |
| IC-7760 | 7760 | LAN | `SET → Network` | ano | **200 W ← dnes chybí** |
| IC-7610 | 7610 | LAN | `SET → Network` | **ne** | 100 W |
| IC-9700 | 9700 | LAN | `SET → Network` | **ne** | 100 W, VHF/UHF only |

U IC-7610 a IC-9700 je „Preset Memory Clear" preset **anténního tuneru**, ne paměť nastavení.
PRESET blok se u nich nevynechá do prázdna — zastoupí ho krok „nastav tyto hodnoty ručně, rádio
je neumí uložit jako preset", ať operátor ví, že o obnovu jedním dotykem nepřišel omylem.

IC-9700 je VHF/UHF-only → seznam JS8 dial frekvencí pro HF pásma u něj nemá smysl.

Podklady: `docs/IC-7300MK2_ENG_{BM_2,Advanced_0}.pdf`, `docs/IC-7610_ENG_{IM_Basic_5,AM_6a}.pdf`,
`docs/IC-9700_ENG_Basic_0a.pdf`, `docs/IC-7760_ENG_{Basic_6a,Basic_1,Advanced_0}.pdf`.

### Mapování detekovaného stringu → návod: parsovat číslo

Číslo je určující. Jedna sdílená normalizační funkce pro **oba** účely (výběr návodu i lookup
výkonu), ne dvě různá pravidla pro tentýž string.

Kolizi `7300` (původní IC-7300 vs MK2) řeší sám kód: `radioModelLearnTick()` se učí **výhradně
přes LAN**, a původní IC-7300 žádný Ethernet nemá, Network Control neumí, caps nikdy nepošle.
Takže **číslo 7300 přes LAN ⇒ vždy MK2**. Rozlišovat MK2 ve stringu není potřeba.

**Neznámé číslo → generický „Icom LAN" návod**, nikdy fallback na IC-705 (to je zakázaný případ
z §1). Nahoře `Detected: <co rádio hlásí> — no specific guide yet, showing the common Icom LAN
procedure`, switcher zůstává otevřený. Generický průnik: Network Control ON, porty
50001/50002/50003, user/pass, `LAN AF/IF Output → AF`, `AF SQL → OFF (Open)`, USB-D,
`DATA MOD → LAN`.

Zamítnuto: prázdný switcher s „vyber si" — first-run operátor neví, co vybrat, generický
průnik ho dovede k funkčnímu spojení.

### Jedna větev v generickém návodu: přemostěné rádio

Jediný případ, kdy je číslo správné a návod přesto špatný. **wfview server** (vendorovaný
v `wfview-master/`) umí sériové rádio přemostit na síť týmž Icom protokolem — přihlásí se
IC-7300 nebo IC-7100, ohlásí své číslo, ale žádné `SET → Network` menu fyzicky nemá. Neřeší se
změnou návrhu, jen jednou větou: *„If your radio is bridged by a wfview/RS-BA1 server, configure
the network on the server, not in the radio menu."*

### Gate card

[`lan-gate.js:68`](../data/lan-gate.js#L68) dnes říká „IC-705 is tested. Other Icom
transceivers…" — odkazuje na typ, který není připojený. `lan-gate.js` už `/setup-data.json`
čte a už zná slot, takže umí říct zapamatovaný `trx{N}model`, i když link neběží. Bez
zapamatovaného modelu → zástupné `TRX1/2/3`, žádný model.

### Ostrá chyba k vytažení dopředu (commit 3)

`RADIO_FULL_POWER_W` ve [`wspr-core.js:305`](../data/wspr-core.js#L305) **neobsahuje IC-7760**
→ `fullPowerWatts()` vrátí `null` → připojený IC-7760 dnes **nesmí odvysílat WSPR**
(„unknown model"). IC-7760 je 200 W (`Default: 200W (AM: 50W)`, `IC-7760_ENG_Basic_6a.pdf`).
Není to feature, nemá čekat na návody.

---

## 9. Co drží nápovědu u renamu

Ospravedlnění celého přejmenování je „už se neomezuje jen na IC-705", a nápověda je **jediné
místo, kde se ten slib operátorovi ukáže**. Rename bez ní je release, který přejmenuje věci
a pořád vysvětluje jen IC-705. Proto stejný REV.

Zároveň je to jediná část, kterou **grep neověří** — potřebuje napsat nový obsah z pěti PDF
a kontrolu na rádiu. Proto samostatný commit až po renamu: kdyby per-model návody potřebovaly
víc práce, nesmí držet rename jako rukojmí.
