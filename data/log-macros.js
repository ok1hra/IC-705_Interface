'use strict';
/**
 * log-macros.js — contest macro generator
 *
 * Macros: CQ, TXEXCH, TXEXCHSP, TXEXCHSP2, TU
 * Transport: HTTP POST /cmd {type:"sendCw", text} — firmware routes CW vs FSK/RTTY by current TRX mode.
 */

(function (global) {

  // ── Mode classification ────────────────────────────────────────────────────

  function modeGroup(mode) {
    const m = (mode || '').toUpperCase();
    if (['CW','CW-R','CWR'].includes(m))                  return 'CW';
    if (['RTTY','RTTY-R','RTTYR','FSK'].includes(m))       return 'RTTY';
    if (['SSB','LSB','USB','FM','AM','DV'].includes(m))     return 'PHONE';
    return 'CW';  // safe default
  }

  function isVhfPlus(freqHz) {
    return Number(freqHz) > 49_000_000;
  }

  // ── CW number: 3-char padded, 0→T and 9→N when abbrev enabled ─────────────

  function cwNumber(n, abbrev) {
    const s = String(n).padStart(3, '0');
    if (!abbrev) return s;
    return s.replace(/0/g, 'T').replace(/9/g, 'N');
  }

  // ── UTC HHMM ───────────────────────────────────────────────────────────────

  function utcHHMM() {
    const d = new Date();
    return String(d.getUTCHours()).padStart(2,'0') + String(d.getUTCMinutes()).padStart(2,'0');
  }

  function utcHHMMcw(abbrev) {
    const s = utcHHMM();
    if (!abbrev) return s;
    return s.replace(/0/g, 'T').replace(/9/g, 'N');
  }

  // ── Build macro text ───────────────────────────────────────────────────────

  /**
   * buildMacro(type, ctx) → string
   *
   * ctx = {
   *   mode, freqHz, stationCall, call, exchangeType,
   *   exchange, qsoNumber, prevQsoNumber, myLocator, cwAbbrev
   * }
   * type: 'CQ' | 'TXEXCH' | 'TXEXCHSP' | 'TXEXCHSP2' | 'TU'
   */
  function buildMacro(type, ctx) {
    const mg     = modeGroup(ctx.mode);
    const vhf    = isVhfPlus(ctx.freqHz);
    const abbrev = ctx.cwAbbrev !== false;
    const rst    = abbrev ? '5nn' : '599';
    const nr     = cwNumber(ctx.qsoNumber  || 1, abbrev);
    const prevNr = cwNumber(ctx.prevQsoNumber || Math.max(1, (ctx.qsoNumber || 1) - 1), abbrev);
    const loc    = ctx.myLocator || '';
    const myCall = ctx.stationCall || 'MYCALL';
    const dxCall = ctx.call        || '';

    switch (type) {

      case 'CQ':
        if (mg === 'RTTY') return '\r\n ' + myCall + ' ' + myCall + ' ' + myCall + ' TEST';
        if (mg === 'CW')   return myCall + ' ' + myCall + ' TEST';
        return '';  // PHONE — manual

      case 'TXEXCH': {
        if (mg === 'RTTY') {
          const nrStr  = String(ctx.qsoNumber || 1).padStart(3, '0');
          const excStr = ctx.exchangeType === 'NRUTC' ? ('599-' + nrStr + '-' + utcHHMM())
                       : ctx.exchangeType === 'NRLOC' ? ('599-' + nrStr + '-' + loc)
                       : ctx.exchangeType === 'NR'    ? '599-' + nrStr
                       : (ctx.exchange ? '599-' + ctx.exchange : '599');
          return '\r\n ' + dxCall + ' ' + dxCall + ' ' + excStr + ' ' + excStr;
        }
        if (mg === 'CW') {
          if (ctx.exchangeType === 'NR' || ctx.exchangeType === 'NRUTC') {
            const nrPart = ctx.exchangeType === 'NRUTC' ? nr + '-' + utcHHMMcw(abbrev) : nr;
            return vhf ? dxCall + ' ' + rst + ' ' + nrPart + ' ' + loc
                       : dxCall + ' ' + rst + ' ' + nrPart;
          }
          if (ctx.exchangeType === 'NRLOC') return dxCall + ' ' + rst + ' ' + nr + ' ' + loc;
          return dxCall + ' ' + rst + ' ' + ctx.exchange;
        }
        return '';
      }

      case 'TXEXCHSP': {
        if (mg === 'RTTY') {
          const nrStr  = String(ctx.qsoNumber || 1).padStart(3, '0');
          const excStr = ctx.exchangeType === 'NRUTC' ? ('599-' + nrStr + '-' + utcHHMM())
                       : ctx.exchangeType === 'NRLOC' ? ('599-' + nrStr + '-' + loc)
                       : ctx.exchangeType === 'NR'    ? '599-' + nrStr
                       : (ctx.exchange ? '599-' + ctx.exchange : '599');
          return '\r\n ' + dxCall + ' ' + dxCall + ' ' + excStr + ' ' + excStr;
        }
        if (mg === 'CW') {
          if (ctx.exchangeType === 'NR' || ctx.exchangeType === 'NRUTC') {
            const nrPart = ctx.exchangeType === 'NRUTC' ? nr + '-' + utcHHMMcw(abbrev) : nr;
            return vhf ? rst + ' ' + nrPart + ' ' + loc + ' tu' : rst + ' ' + nrPart;
          }
          if (ctx.exchangeType === 'NRLOC') return rst + ' ' + nr + ' ' + loc + ' tu';
          return ctx.exchange ? rst + ' ' + ctx.exchange : rst + ' tu';
        }
        return '';
      }

      case 'TXEXCHSP2': {
        const pn    = ctx.prevQsoNumber || Math.max(1, (ctx.qsoNumber || 1) - 1);
        const pnCw  = cwNumber(pn, abbrev);
        const pnStr = String(pn).padStart(3, '0');
        if (mg === 'RTTY') {
          const excStr = ctx.exchangeType === 'NRUTC' ? ('599-' + pnStr + '-' + utcHHMM())
                       : ctx.exchangeType === 'NRLOC' ? ('599-' + pnStr + '-' + loc)
                       : ctx.exchangeType === 'NR'    ? '599-' + pnStr
                       : (ctx.exchange ? '599-' + ctx.exchange : '599');
          return '\r\n ' + dxCall + ' ' + dxCall + ' ' + excStr + ' ' + excStr;
        }
        if (mg === 'CW') {
          if (ctx.exchangeType === 'NR' || ctx.exchangeType === 'NRUTC') {
            const pnPart = ctx.exchangeType === 'NRUTC' ? pnCw + '-' + utcHHMMcw(abbrev) : pnCw;
            return vhf ? rst + ' ' + pnPart + ' ' + loc : rst + ' ' + pnPart;
          }
          if (ctx.exchangeType === 'NRLOC') return rst + ' ' + pnCw + ' ' + loc;
          return ctx.exchange ? rst + ' ' + ctx.exchange : rst + ' tu';
        }
        return '';
      }

      case 'TU':
        if (mg === 'RTTY') return dxCall + ' tu ' + myCall;
        if (mg === 'CW')   return 'tu ' + myCall;
        return '';

      case 'CALLTU':
        if (mg === 'RTTY') return dxCall + ' tu ' + myCall;
        if (mg === 'CW')   return dxCall + ' tu';
        return '';

      default:
        return '';
    }
  }

  // ── Send a macro via HTTP POST /cmd ───────────────────────────────────────

  function sendMacro(type, ctx) {
    const text = buildMacro(type, ctx);
    if (!text) return Promise.resolve(false);  // PHONE mode — nothing to send
    if (ctx._oi3) {
      return fetch('/oi3/send', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ trx: ctx._trx, text })
      }).then(r => r.ok).catch(() => false);
    }
    return fetch('/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ type: 'sendCw', text })
    }).then(r => r.ok).catch(() => false);
  }

  // ── Init (no-op, kept for API compatibility) ───────────────────────────────

  function init() {}

  // ── Export ─────────────────────────────────────────────────────────────────

  global.LogMacros = { buildMacro, sendMacro, modeGroup, init };

}(window));
