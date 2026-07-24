const BD_ROWS = 16;
const BD_DEFAULTS = [
  { fMin: 1810,   fMax: 2000,   outputs: 0x0001 },
  { fMin: 3500,   fMax: 3800,   outputs: 0x0002 },
  { fMin: 5351,   fMax: 5367,   outputs: 0x0004 },
  { fMin: 7000,   fMax: 7200,   outputs: 0x0008 },
  { fMin: 10100,  fMax: 10150,  outputs: 0x0010 },
  { fMin: 14000,  fMax: 14350,  outputs: 0x0020 },
  { fMin: 18068,  fMax: 18168,  outputs: 0x0040 },
  { fMin: 21000,  fMax: 21450,  outputs: 0x0080 },
  { fMin: 24890,  fMax: 24990,  outputs: 0x0100 },
  { fMin: 28000,  fMax: 29700,  outputs: 0x0200 },
  { fMin: 50000,  fMax: 54000,  outputs: 0x0400 },
  { fMin: 70000,  fMax: 70500,  outputs: 0x0800 },
  { fMin: 144000, fMax: 146000, outputs: 0x1000 },
  { fMin: 430000, fMax: 440000, outputs: 0x2000 },
  { fMin: 0,      fMax: 0,      outputs: 0x0000 },
  { fMin: 0,      fMax: 0,      outputs: 0x0000 },
];

let bdCurrentFreqKhz = 0;
let bdSaveTimer = null;

function bdScheduleSave() {
  const statusEl = document.getElementById('bd-status');
  statusEl.textContent = 'Saving…';
  clearTimeout(bdSaveTimer);
  bdSaveTimer = setTimeout(bdSave, 2000);
}

async function bdInit() {
  // Fetch config and status independently — one failure must not block the other
  const [cfg, status] = await Promise.all([
    fetch('/api/bd-config').then(r => r.json()).catch(() => ({})),
    fetch('/api/status').then(r => r.json()).catch(() => ({})),
  ]);

  const rows = (cfg.rows && cfg.rows.length === BD_ROWS) ? cfg.rows : BD_DEFAULTS;
  bdRenderTable(rows);
  bdPopulateSource(status, cfg.source || 1);
  bdStartPolling();
}

function bdPopulateSource(status, selectedSource) {
  const sel = document.getElementById('bd-source');
  sel.innerHTML = '';
  sel.appendChild(new Option('TRX1 — ' + (status.trx1Label || 'IC-705'), '1'));
  if (status.trx2Label && status.trx2Label !== 'TRX2') {
    sel.appendChild(new Option('TRX2 — ' + status.trx2Label, '2'));
  }
  if (status.trx3Label && status.trx3Label !== 'TRX3') {
    sel.appendChild(new Option('TRX3 — ' + status.trx3Label, '3'));
  }
  sel.value = String(selectedSource);
  if (!sel.value) sel.value = '1';
}

function bdRenderTable(rows) {
  const tbody = document.getElementById('bd-tbody');
  tbody.innerHTML = '';
  for (let i = 0; i < BD_ROWS; i++) {
    const row = rows[i] || { fMin: 0, fMax: 0, outputs: 0 };
    const tr = document.createElement('tr');
    tr.className = 'bd-row';
    let html =
      `<td><input type="number" class="bd-fmin" min="0" value="${row.fMin}"></td>` +
      `<td><input type="number" class="bd-fmax" min="0" value="${row.fMax}"></td>`;
    for (let bit = 0; bit < 16; bit++) {
      const checked = (row.outputs & (1 << bit)) ? 'checked' : '';
      html += `<td><input type="checkbox" class="bd-chk" ${checked}></td>`;
    }
    tr.innerHTML = html;
    tbody.appendChild(tr);
  }
}

function bdHighlightTable(freqKhz) {
  const rows = document.querySelectorAll('.bd-row');
  const numRows = rows.length;

  // Clear all highlights
  rows.forEach(tr => {
    tr.querySelectorAll('td').forEach(td => td.classList.remove('bd-cell-green', 'bd-cell-red'));
  });
  document.querySelectorAll('.bd-col-header').forEach(th => th.classList.remove('bd-active'));

  // Pass 1: determine active rows, their lastChecked output, and cache DOM refs
  const rowInfo = [];
  for (let r = 0; r < numRows; r++) {
    const tr = rows[r];
    const tds = tr.querySelectorAll('td');
    const chks = tr.querySelectorAll('.bd-chk');
    const fMin = parseInt(tr.querySelector('.bd-fmin').value) || 0;
    const fMax = parseInt(tr.querySelector('.bd-fmax').value) || 0;
    const active = fMin > 0 && fMax > 0 && freqKhz >= fMin && freqKhz <= fMax;
    let lastChecked = -1;
    if (active) {
      for (let bit = 15; bit >= 0; bit--) {
        if (chks[bit].checked) { lastChecked = bit; break; }
      }
    }
    rowInfo.push({ active, lastChecked, tds, chks });
  }

  // Pass 2: per column — find first and last active row that has this output checked
  const colRange = [];
  for (let bit = 0; bit < 16; bit++) {
    let first = -1, last = -1;
    for (let r = 0; r < numRows; r++) {
      if (!rowInfo[r].active) continue;
      if (rowInfo[r].chks[bit].checked) {
        if (first === -1) first = r;
        last = r;
      }
    }
    colRange.push({ first, last });
  }

  // Apply red column ranges first (lower priority)
  // Red extends from row 0 down to the last active row with this output checked
  for (let bit = 0; bit < 16; bit++) {
    const { last } = colRange[bit];
    if (last === -1) continue;
    document.getElementById(`bdh-${bit}`).classList.add('bd-active');
    for (let r = 0; r <= last; r++) {
      rowInfo[r].tds[bit + 2].classList.add('bd-cell-red');
    }
  }

  // Apply green row ranges second (higher priority — overrides red)
  for (let r = 0; r < numRows; r++) {
    const { active, lastChecked, tds } = rowInfo[r];
    if (!active) continue;
    tds[0].classList.add('bd-cell-green');
    tds[1].classList.add('bd-cell-green');
    if (lastChecked >= 0) {
      for (let bit = 0; bit <= lastChecked; bit++) {
        tds[bit + 2].classList.remove('bd-cell-red');
        tds[bit + 2].classList.add('bd-cell-green');
      }
    }
  }
}

function bdStartPolling() {
  // TRX1: poll /state every 1 s
  setInterval(async () => {
    const source = parseInt(document.getElementById('bd-source').value);
    if (source !== 1) return;
    try {
      const data = await fetch('/state').then(r => r.json());
      const freqKhz = Math.round((data.frequency || 0) / 1000);
      if (freqKhz !== bdCurrentFreqKhz) {
        bdCurrentFreqKhz = freqKhz;
        bdHighlightTable(freqKhz);
      }
    } catch (_) {}
  }, 1000);

  // TRX2/3: poll /api/status every 2 s
  setInterval(async () => {
    const source = parseInt(document.getElementById('bd-source').value);
    if (source === 1) return;
    try {
      const s = await fetch('/api/status').then(r => r.json());
      const freqHz = source === 2 ? (s.trx2freq || 0) : (s.trx3freq || 0);
      const freqKhz = Math.round(freqHz / 1000);
      if (freqKhz !== bdCurrentFreqKhz) {
        bdCurrentFreqKhz = freqKhz;
        bdHighlightTable(freqKhz);
      }
    } catch (_) {}
  }, 2000);
}

async function bdSave() {
  const rows = [];
  document.querySelectorAll('.bd-row').forEach(tr => {
    const fMin = parseInt(tr.querySelector('.bd-fmin').value) || 0;
    const fMax = parseInt(tr.querySelector('.bd-fmax').value) || 0;
    let outputs = 0;
    tr.querySelectorAll('.bd-chk').forEach((chk, bit) => {
      if (chk.checked) outputs |= (1 << bit);
    });
    rows.push({ fMin, fMax, outputs });
  });
  const source = parseInt(document.getElementById('bd-source').value);
  const statusEl = document.getElementById('bd-status');
  try {
    const r = await fetch('/api/bd-config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ source, rows })
    });
    const j = await r.json();
    statusEl.textContent = j.ok ? 'Saved.' : 'Save error.';
  } catch (_) {
    statusEl.textContent = 'Connection error.';
  }
  setTimeout(() => { statusEl.textContent = ''; }, 3000);
}

document.getElementById('bd-defaults').addEventListener('click', () => {
  bdRenderTable(BD_DEFAULTS);
  bdScheduleSave();
});
document.getElementById('bd-source').addEventListener('change', () => {
  bdCurrentFreqKhz = 0;
  bdScheduleSave();
});

document.getElementById('bd-tbody').addEventListener('change', bdScheduleSave);
document.getElementById('bd-tbody').addEventListener('input', (e) => {
  if (e.target.classList.contains('bd-fmin') || e.target.classList.contains('bd-fmax')) {
    bdScheduleSave();
  }
});

bdInit();
