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
    return Number(freqHz) > 140_000_000;
  }

  // ── CW number padding: 3 chars, leading zeros → T  (TT1, TT7, T42, 123) ──

  function cwNumber(n) {
    const s = String(n);
    if (s.length >= 3) return s;
    return s.padStart(3, 'T').replace(/0/g, 'T');
  }

  // ── UTC HHMM ───────────────────────────────────────────────────────────────

  function utcHHMM() {
    const d = new Date();
    return String(d.getUTCHours()).padStart(2,'0') + String(d.getUTCMinutes()).padStart(2,'0');
  }

  // ── Build macro text ───────────────────────────────────────────────────────

  /**
   * buildMacro(type, ctx) → string
   *
   * ctx = {
   *   mode, freqHz, stationCall, call, exchangeType,
   *   exchange, qsoNumber, prevQsoNumber, myLocator
   * }
   * type: 'CQ' | 'TXEXCH' | 'TXEXCHSP' | 'TXEXCHSP2' | 'TU'
   */
  function buildMacro(type, ctx) {
    const mg     = modeGroup(ctx.mode);
    const vhf    = isVhfPlus(ctx.freqHz);
    const nr     = cwNumber(ctx.qsoNumber  || 1);
    const prevNr = cwNumber(ctx.prevQsoNumber || Math.max(1, (ctx.qsoNumber || 1) - 1));
    const exch   = ctx.exchangeType === 'NR'     ? nr :
                   ctx.exchangeType === 'NRUTC'  ? (nr + '-' + utcHHMM()) :
                   (ctx.exchange || '');
    const prevExch = ctx.exchangeType === 'NR'    ? prevNr :
                     ctx.exchangeType === 'NRUTC' ? (prevNr + '-' + utcHHMM()) :
                     (ctx.exchange || '');
    const myCall = ctx.stationCall || 'MYCALL';
    const dxCall = ctx.call        || '';
    const loc    = ctx.myLocator   || '';

    switch (type) {

      case 'CQ':
        if (mg === 'RTTY') return '\r\n ' + myCall + ' ' + myCall + ' ' + myCall + ' TEST';
        if (mg === 'CW')   return myCall + ' ' + myCall + ' TEST';
        return '';  // PHONE — manual

      case 'TXEXCH':
        if (mg === 'RTTY') {
          const excStr = ctx.exchangeType === 'NRUTC'
            ? ('599-' + String(ctx.qsoNumber).padStart(3,'0') + '-' + utcHHMM())
            : '599-' + (ctx.exchangeType === 'NR' ? String(ctx.qsoNumber).padStart(3,'0') : ctx.exchange);
          return '\r\n ' + dxCall + ' ' + dxCall + ' ' + excStr + ' ' + excStr;
        }
        if (mg === 'CW') {
          if (ctx.exchangeType === 'NR' || ctx.exchangeType === 'NRUTC') {
            return vhf
              ? dxCall + ' 5nn ' + nr + ' ' + loc
              : dxCall + ' 5nn ' + nr;
          }
          return dxCall + ' 5nn ' + ctx.exchange;
        }
        return '';  // PHONE

      case 'TXEXCHSP':
        if (mg === 'RTTY') {
          const excStr = ctx.exchangeType === 'NRUTC'
            ? ('599-' + String(ctx.qsoNumber).padStart(3,'0') + '-' + utcHHMM())
            : '599-' + (ctx.exchangeType === 'NR' ? String(ctx.qsoNumber).padStart(3,'0') : ctx.exchange);
          return '\r\n ' + dxCall + ' ' + dxCall + ' ' + excStr + ' ' + excStr;
        }
        if (mg === 'CW') {
          if (ctx.exchangeType === 'NR' || ctx.exchangeType === 'NRUTC') {
            return vhf ? '5nn ' + nr + ' ' + loc + ' tu' : '5nn ' + nr;
          }
          return '5nn ' + ctx.exchange;
        }
        return '';

      case 'TXEXCHSP2': {
        const pn = ctx.prevQsoNumber || Math.max(1,(ctx.qsoNumber||1)-1);
        const pnCw = cwNumber(pn);
        if (mg === 'RTTY') {
          const excStr = ctx.exchangeType === 'NRUTC'
            ? ('599-' + String(pn).padStart(3,'0') + '-' + utcHHMM())
            : '599-' + (ctx.exchangeType === 'NR' ? String(pn).padStart(3,'0') : ctx.exchange);
          return '\r\n ' + dxCall + ' ' + dxCall + ' ' + excStr + ' ' + excStr;
        }
        if (mg === 'CW') {
          if (ctx.exchangeType === 'NR' || ctx.exchangeType === 'NRUTC') {
            return vhf ? '5nn ' + pnCw + ' ' + loc : '5nn ' + pnCw;
          }
          return '5nn ' + ctx.exchange;
        }
        return '';
      }

      case 'TU':
        if (mg === 'RTTY') return dxCall + ' tu ' + myCall;
        if (mg === 'CW')   return 'tu ' + myCall;
        return '';

      default:
        return '';
    }
  }

  // ── Send a macro via HTTP POST /cmd ───────────────────────────────────────

  function sendMacro(type, ctx) {
    const text = buildMacro(type, ctx);
    if (!text) return Promise.resolve(false);  // PHONE mode — nothing to send
    if (ctx._oi3 && ctx._trxIp) {
      return fetch('/oi3/send', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ip: ctx._trxIp, text })
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
