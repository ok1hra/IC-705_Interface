#!/usr/bin/env python3
"""
ICOM LAN (RS-BA1 / wfview) protocol tester — steps 1+2 of the IC-705-LAN path.

Step 1 (default): control-port handshake + login + token:
  AreYouThere -> IAmHere -> AreYouReady -> ready -> Login -> token confirm
  -> capabilities/conninfo (radio name, MAC, busy state)

Step 2 (--civ): request the stream, open the CI-V data port and read the
frequency — proves the whole CAT path end to end, still PC-only.

Usage:
  python3 icom-lan-login-test.py <radio-ip> <username> <password> [--civ] [-v]

Exit codes:
  0  success (login OK; with --civ also CI-V data received)
  2  radio rejected username/password
  3  no answer to AreYouThere (unreachable / Network Control OFF)
  4  handshake stalled (share the output)
  5  login OK but stream/CI-V failed (share the output)

Protocol reference: wfview-master/src/radio/{icomudpbase,icomudphandler,
icomudpcivdata}.cpp + include/packettypes.h. Fields are little-endian EXCEPT
payloadsize, innerseq, resetcap, the sample-rate/port fields of the stream
request and the CI-V sendseq, which are big-endian.
"""

import argparse
import secrets
import select
import socket
import struct
import sys
import time

# --- passcode table (icomudpbase.h, verified byte-for-byte) ------------------
SEQ = [0] * 32 + [
    0x47, 0x5D, 0x4C, 0x42, 0x66, 0x20, 0x23, 0x46, 0x4E, 0x57, 0x45, 0x3D,
    0x67, 0x76, 0x60, 0x41, 0x62, 0x39, 0x59, 0x2D, 0x68, 0x7E,
    0x7C, 0x65, 0x7D, 0x49, 0x29, 0x72, 0x73, 0x78, 0x21, 0x6E, 0x5A, 0x5E,
    0x4A, 0x3E, 0x71, 0x2C, 0x2A, 0x54, 0x3C, 0x3A, 0x63, 0x4F,
    0x43, 0x75, 0x27, 0x79, 0x5B, 0x35, 0x70, 0x48, 0x6B, 0x56, 0x6F, 0x34,
    0x32, 0x6C, 0x30, 0x61, 0x6D, 0x7B, 0x2F, 0x4B, 0x64, 0x38,
    0x2B, 0x2E, 0x50, 0x40, 0x3F, 0x55, 0x33, 0x37, 0x25, 0x77, 0x24, 0x26,
    0x74, 0x6A, 0x28, 0x53, 0x4D, 0x69, 0x22, 0x5C, 0x44, 0x31,
    0x36, 0x58, 0x3B, 0x7A, 0x51, 0x5F, 0x52,
] + [0]


LOCAL_BASE = 50000  # overridden by --local-base


def passcode(text):
    out = bytearray()
    for i, ch in enumerate(text[:16]):
        p = ord(ch) + i
        if p > 126:
            p = 32 + p % 127
        out.append(SEQ[p])
    return bytes(out)


def hexdump(data):
    return " ".join(f"{b:02x}" for b in data)


def decode_civ_freq(frame):
    """Return Hz from a CI-V frequency frame (cmd 0x00/0x03), else None."""
    # FE FE to from cmd d0 d1 d2 d3 d4 FD — BCD, LSB first
    if len(frame) < 11 or frame[0] != 0xFE or frame[1] != 0xFE:
        return None
    if frame[4] not in (0x00, 0x03):
        return None
    bcd = frame[5:10]
    hz = 0
    for b in reversed(bcd):
        hz = hz * 100 + ((b >> 4) & 0xF) * 10 + (b & 0xF)
    return hz


# --- shared UDP channel primitives -------------------------------------------
class UdpChannel:
    def __init__(self, radio_ip, remote_port, tag, verbose, local_port=0):
        self.radio = (radio_ip, remote_port)
        self.tag = tag
        self.verbose = verbose
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # kappanhang binds the local port to the same number as the radio's port
        # (50001/50002/50003) — the IC-705 ignores CI-V sent from other ports
        self.sock.bind(("0.0.0.0", local_port))
        self.local_port = self.sock.getsockname()[1]

        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(self.radio)
        self.local_ip = probe.getsockname()[0]
        probe.close()
        oct3, oct4 = (int(x) for x in self.local_ip.split(".")[2:4])
        self.my_id = (oct3 << 24) | (oct4 << 16) | (self.local_port & 0xFFFF)

        self.remote_id = 0
        self.send_seq = 1
        self.ping_seq = 0
        self.tx_buf = {}
        self.got_here = False
        self.next_ayt = 0.0
        self.next_ping = 0.0
        self.next_idle = 0.0

    def log(self, msg):
        print(f"[{self.tag}] {msg}")

    def _send(self, data, label):
        if self.verbose:
            self.log(f"-> {label}: {hexdump(data)}")
        self.sock.sendto(data, self.radio)

    def send_control(self, ptype, seq, label):
        pkt = struct.pack("<IHHII", 0x10, ptype, seq, self.my_id, self.remote_id)
        self._send(pkt, label)

    def send_tracked(self, pkt, label):
        pkt = bytearray(pkt)
        struct.pack_into("<H", pkt, 6, self.send_seq)
        self.tx_buf[self.send_seq] = bytes(pkt)
        if len(self.tx_buf) > 500:
            self.tx_buf.pop(min(self.tx_buf))
        self.send_seq = (self.send_seq + 1) & 0xFFFF
        self._send(bytes(pkt), label)

    def send_idle(self):
        pkt = bytearray(struct.pack("<IHHII", 0x10, 0, 0, self.my_id, self.remote_id))
        self.send_tracked(pkt, "idle")

    def send_ping(self):
        now = int((time.time() * 1000) % 86_400_000)
        pkt = struct.pack("<IHHIIBI", 0x15, 0x07, self.ping_seq,
                          self.my_id, self.remote_id, 0x00, now)
        self.ping_seq = (self.ping_seq + 1) & 0xFFFF
        self._send(pkt, "ping")

    def send_ping_reply(self, seq, tstamp):
        pkt = struct.pack("<IHHIIBI", 0x15, 0x07, seq,
                          self.my_id, self.remote_id, 0x01, tstamp)
        self._send(pkt, "ping-reply")

    def tick(self, now):
        """Periodic sends; AreYouThere until got_here, then ping+idle."""
        if not self.got_here:
            if now >= self.next_ayt:
                self.send_control(0x03, 0x00, "are-you-there")
                self.next_ayt = now + 0.5
            return
        if now >= self.next_ping:
            self.send_ping()
            self.next_ping = now + 0.5
        if now >= self.next_idle:
            self.send_idle()
            self.next_idle = now + 0.1

    def handle_common(self, r):
        """Ping replies + retransmit requests. Returns True if consumed."""
        n = len(r)
        ptype, seq = struct.unpack_from("<HH", r, 4)
        if n == 0x15 and ptype == 0x07:
            reply, tstamp = struct.unpack_from("<BI", r, 0x10)
            if reply == 0x00:
                self.send_ping_reply(seq, tstamp)
            elif reply == 0x01 and not getattr(self, "_pong_seen", False):
                self._pong_seen = True
                self.log("<- first pong (radio answers our pings on this channel)")
            return True
        if n == 0x10 and ptype == 0x01:
            data = self.tx_buf.get(seq)
            if data:
                self.log(f"retransmit request seq=0x{seq:04x} -> resending")
                self._send(data, "retransmit")
            return True
        return False


# --- control port (50001) ----------------------------------------------------
class ControlPort(UdpChannel):
    def __init__(self, radio_ip, port, username, password, verbose):
        super().__init__(radio_ip, port, "ctl", verbose, local_port=LOCAL_BASE+1)
        self.username = username
        self.password = password
        self.auth_seq = 0x30
        self.tok_request = secrets.randbits(16)
        self.token = 0
        self.got_ready = False
        self.login_ok = False
        self.login_err = None
        self.token_ok = False
        self.radios = []            # (name, mac bytes, civ)
        self.conninfo = None        # (busy, computer)
        self.stream_status = None   # (error, civport, audioport)
        self.no_audio = False
        self.log(f"local {self.local_ip}:{self.local_port} -> {radio_ip}:{port} "
                 f"myId=0x{self.my_id:08x}")

    def send_login(self):
        p = bytearray(0x80)
        struct.pack_into("<IHH", p, 0, 0x80, 0, 0)
        struct.pack_into("<II", p, 8, self.my_id, self.remote_id)
        struct.pack_into(">I", p, 0x10, 0x70)
        p[0x14] = 0x01
        p[0x15] = 0x00
        struct.pack_into(">H", p, 0x16, self.auth_seq); self.auth_seq += 1
        struct.pack_into("<H", p, 0x1A, self.tok_request)
        u = passcode(self.username)
        w = passcode(self.password)
        p[0x40:0x40 + len(u)] = u
        p[0x50:0x50 + len(w)] = w
        name = b"esp32-test"
        p[0x60:0x60 + len(name)] = name
        self.send_tracked(p, "LOGIN")
        self.log(f"login sent (user={self.username!r}, tokrequest=0x{self.tok_request:04x})")

    def send_token(self, magic, label):
        p = bytearray(0x40)
        struct.pack_into("<IHH", p, 0, 0x40, 0, 0)
        struct.pack_into("<II", p, 8, self.my_id, self.remote_id)
        struct.pack_into(">I", p, 0x10, 0x30)
        p[0x14] = 0x01
        p[0x15] = magic
        struct.pack_into(">H", p, 0x16, self.auth_seq); self.auth_seq += 1
        struct.pack_into("<H", p, 0x1A, self.tok_request)
        struct.pack_into("<I", p, 0x1C, self.token)
        # NOTE: wfview also sends resetcap=0x0798 at 0x24 here, but the IC-705
        # ignores such auth packets entirely (kappanhang sends zeros)
        self.send_tracked(p, label)

    def send_stream_request(self, civ_local_port, audio_local_port):
        name, mac, _civ, commoncap, guid = self.radios[0]
        p = bytearray(0x90)
        struct.pack_into("<IHH", p, 0, 0x90, 0, 0)
        struct.pack_into("<II", p, 8, self.my_id, self.remote_id)
        struct.pack_into(">I", p, 0x10, 0x80)
        p[0x14] = 0x01
        p[0x15] = 0x03
        struct.pack_into(">H", p, 0x16, self.auth_seq); self.auth_seq += 1
        struct.pack_into("<H", p, 0x1A, self.tok_request)
        struct.pack_into("<I", p, 0x1C, self.token)
        if commoncap == 0x8010:
            struct.pack_into("<H", p, 0x27, 0x8010)      # commoncap + mac identity
            p[0x2A:0x30] = mac
        else:
            p[0x20:0x30] = guid                          # guid identity (wfview useGuid)
            self.log("using GUID identity (commoncap != 0x8010)")
        nm = name.encode()[:32]
        p[0x40:0x40 + len(nm)] = nm
        u = passcode(self.username)
        p[0x60:0x60 + len(u)] = u
        if self.no_audio:
            p[0x70] = 0                                   # rxenable off
            p[0x71] = 0                                   # txenable off
            p[0x72] = 0                                   # rxcodec none
            p[0x73] = 0                                   # txcodec none
            struct.pack_into(">I", p, 0x74, 0)            # rxsample
            struct.pack_into(">I", p, 0x78, 0)            # txsample
        else:
            p[0x70] = 1                                   # rxenable
            p[0x71] = 1                                   # txenable
            p[0x72] = 0x04                                # rxcodec LPCM 16bit/1ch (wfview default)
            p[0x73] = 0x04                                # txcodec
            struct.pack_into(">I", p, 0x74, 48000)        # rxsample
            struct.pack_into(">I", p, 0x78, 48000)        # txsample
        struct.pack_into(">I", p, 0x7C, civ_local_port)
        struct.pack_into(">I", p, 0x80, audio_local_port)
        struct.pack_into(">I", p, 0x84, 150)              # txbuffer (ms, wfview default)
        p[0x88] = 1                                       # convert
        self.send_tracked(p, "STREAM-REQUEST")
        self.log(f"stream request sent (civ_local={civ_local_port} "
                 f"audio_local={audio_local_port} radio={name!r})")

    def handle(self, r):
        if self.handle_common(r):
            return
        n = len(r)
        ptype, seq = struct.unpack_from("<HH", r, 4)
        sentid = struct.unpack_from("<I", r, 8)[0]

        if n == 0x10:
            if ptype == 0x04 and not self.got_here:
                self.got_here = True
                self.remote_id = sentid
                self.log(f"<- I AM HERE remoteId=0x{self.remote_id:08x}")
                self.send_control(0x06, 0x01, "are-you-ready")
            elif ptype == 0x06 and not self.got_ready:
                self.got_ready = True
                self.log("<- I AM READY -> sending login")
                self.send_login()
        elif n == 0x40:
            response = struct.unpack_from("<I", r, 0x30)[0]
            self.log(f"<- AUTH response reply=0x{r[0x14]:02x} reqtype=0x{r[0x15]:02x} "
                     f"response=0x{response:08x}")
            if r[0x14] == 0x02 and r[0x15] == 0x05:
                self.token_ok = True   # radio acknowledged the 0x05 auth
        elif n == 0x60:
            error = struct.unpack_from("<I", r, 0x30)[0]
            tokreq = struct.unpack_from("<H", r, 0x1A)[0]
            token = struct.unpack_from("<I", r, 0x1C)[0]
            conn = r[0x40:0x50].split(b"\0")[0].decode(errors="replace")
            self.log(f"<- LOGIN RESPONSE error=0x{error:08x} token=0x{token:08x} "
                     f"connection={conn!r}")
            if error == 0xFEFFFFFF:
                self.login_err = "invalid-credentials"
                self.log("** RADIO REJECTED USERNAME/PASSWORD **")
            elif tokreq == self.tok_request:
                self.token = token
                self.login_ok = True
                self.log("** LOGIN OK — confirming token **")
                # kappanhang: 0x02 confirm (unanswered) + 0x05 auth, whose reply
                # (0x40 with bytes 0x14/0x15 = 02 05) gates the stream request
                self.send_token(0x02, "token-confirm")
                self.send_token(0x05, "token-auth")
        elif n == 0x50:
            error = struct.unpack_from("<I", r, 0x30)[0]
            civport = struct.unpack_from(">H", r, 0x42)[0]
            audioport = struct.unpack_from(">H", r, 0x46)[0]
            self.stream_status = (error, civport, audioport)
            self.log(f"<- STATUS error=0x{error:08x} civport={civport} "
                     f"audioport={audioport}")
        elif n == 0x90:
            busy = struct.unpack_from("<I", r, 0x60)[0]
            computer = r[0x64:0x74].split(b"\0")[0].decode(errors="replace")
            name = r[0x40:0x60].split(b"\0")[0].decode(errors="replace")
            self.conninfo = (busy, computer)
            self.log(f"<- CONNINFO name={name!r} busy={busy} computer={computer!r}")
        elif n >= 0x42 and (n - 0x42) % 0x66 == 0:
            count = (n - 0x42) // 0x66      # numradios field is unreliable
            self.log(f"<- CAPABILITIES ({count} radio(s) by packet length)")
            self.caps_time = time.monotonic()
            self.radios = []
            for i in range(count):
                base = 0x42 + i * 0x66
                guid = bytes(r[base:base + 0x10])
                commoncap = struct.unpack_from("<H", r, base + 0x07)[0]
                mac = bytes(r[base + 0x0A:base + 0x10])
                name = r[base + 0x10:base + 0x30].split(b"\0")[0].decode(errors="replace")
                civ = r[base + 0x52]
                self.radios.append((name, mac, civ, commoncap, guid))
                self.log(f"   radio[{i}] name={name!r} civ=0x{civ:02x} "
                         f"commoncap=0x{commoncap:04x} "
                         f"mac={':'.join(f'{b:02x}' for b in mac)} "
                         f"guid={hexdump(guid)}")
        elif self.verbose:
            self.log(f"<- unhandled len=0x{n:02x}: {hexdump(r[:32])}...")


# --- audio port (handshake only; RTP payload is drained and dropped) ----------
class AudioPort(UdpChannel):
    def __init__(self, radio_ip, remote_port, verbose):
        super().__init__(radio_ip, remote_port, "aud", verbose, local_port=LOCAL_BASE+3)
        self.rtp_packets = 0

    def handle(self, r):
        if self.handle_common(r):
            return
        n = len(r)
        ptype = struct.unpack_from("<H", r, 4)[0] if n >= 6 else -1
        sentid = struct.unpack_from("<I", r, 8)[0] if n >= 12 else 0
        if n == 0x10 and ptype == 0x04:
            if not self.got_here:
                self.got_here = True
                self.remote_id = sentid
                self.log(f"<- I AM HERE remoteId=0x{self.remote_id:08x}")
                self.send_control(0x06, 0x01, "are-you-ready")
        elif n == 0x10:
            pass  # ready reply / idle — nothing to do on the audio channel
        else:
            self.rtp_packets += 1
            if self.rtp_packets == 1:
                self.log("<- audio stream flowing (drained)")


# --- CI-V data port -----------------------------------------------------------
class CivPort(UdpChannel):
    def __init__(self, radio_ip, remote_port, civ_addr, verbose):
        super().__init__(radio_ip, remote_port, "civ", verbose, local_port=LOCAL_BASE+2)
        self.civ_addr = civ_addr
        self.seq_b = 0            # open/close + data sequence, big-endian
        self.open_sent = False
        self.next_open = 0.0
        self.got_data = False
        self.here_time = None
        self.frames = 0
        self.scope_frames = 0
        self.freq_hz = None
        self.log(f"local port {self.local_port} -> radio civ port {remote_port}")

    def send_openclose(self, close):
        p = bytearray(0x16)
        struct.pack_into("<IHH", p, 0, 0x16, 0, 0)
        struct.pack_into("<II", p, 8, self.my_id, self.remote_id)
        struct.pack_into("<H", p, 0x10, 0x01C0)
        struct.pack_into(">H", p, 0x13, self.seq_b)
        # wfview sends 0x04 here, but the IC-705 only honours 0x05
        # (kappanhang serialstream.go, written specifically for the IC-705)
        p[0x15] = 0x00 if close else 0x05
        self.seq_b = (self.seq_b + 1) & 0xFFFF
        self.send_tracked(p, "civ-close" if close else "civ-open")

    def send_civ(self, frame):
        p = bytearray(0x15)
        struct.pack_into("<I", p, 0, 0x15 + len(frame))
        struct.pack_into("<II", p, 8, self.my_id, self.remote_id)
        p[0x10] = 0xC1
        struct.pack_into("<H", p, 0x11, len(frame))
        struct.pack_into(">H", p, 0x13, self.seq_b)
        self.seq_b = (self.seq_b + 1) & 0xFFFF
        self.send_tracked(bytes(p) + frame, f"CIV {hexdump(frame)}")

    # Over LAN the controller address is 0xE1 (RS-BA1 convention) — the radio
    # silently drops frames sent from 0xE0, which is the wired/BT controller.
    CTRL = 0xE1

    def request_freq(self):
        self.send_civ(bytes([0xFE, 0xFE, self.civ_addr, self.CTRL, 0x03, 0xFD]))

    def scope_output_off(self):
        # 27 11 00 = disable waveform data output over CI-V (radio display unaffected)
        self.send_civ(bytes([0xFE, 0xFE, self.civ_addr, self.CTRL, 0x27, 0x11, 0x00, 0xFD]))

    def tick(self, now):
        super().tick(now)
        # the IC-705 often never sends the "ready" (0x06) on the CI-V port —
        # wfview covers that with a watchdog; just open after a short grace
        if (self.got_here and not self.open_sent
                and now >= (self.here_time or 0) + 0.5):
            self.log("no ready packet — sending civ-open anyway")
            self.open_sent = True
            self.send_openclose(False)
            self.next_open = now + 0.5
        # keep re-sending open until CI-V data flows (wfview does the same)
        if self.open_sent and not self.got_data and now >= self.next_open:
            self.send_openclose(False)
            self.next_open = now + 0.5

    def handle(self, r):
        if self.handle_common(r):
            return
        n = len(r)
        ptype, seq = struct.unpack_from("<HH", r, 4)
        sentid = struct.unpack_from("<I", r, 8)[0]
        if n == 0x10:
            if ptype == 0x04:
                if not self.got_here:
                    self.got_here = True
                    self.here_time = time.monotonic()
                    self.remote_id = sentid
                    self.log(f"<- I AM HERE remoteId=0x{self.remote_id:08x}")
                    # every channel must complete the ready exchange (wfview base
                    # class + kappanhang start() both do this on the CI-V channel)
                    self.send_control(0x06, 0x01, "are-you-ready")
            elif ptype == 0x06:
                self.remote_id = sentid
                if not self.open_sent:
                    self.log("<- ready -> sending civ-open")
                    self.open_sent = True
                    self.send_openclose(False)
                    self.next_open = time.monotonic() + 0.5
        elif n > 0x15 and ptype != 0x01:
            plen = struct.unpack_from("<I", r, 0)[0]
            datalen = struct.unpack_from("<H", r, 0x11)[0]
            if (datalen + 0x15) & 0xFFFF == plen & 0xFFFF:
                if self.verbose and self.frames < 2:
                    self.log(f"<- raw data header: {hexdump(r[:0x15])}")
                self.got_data = True
                payload = bytes(r[0x15:])
                # one datagram may carry several CI-V frames back to back
                pos = 0
                while True:
                    s = payload.find(b"\xfe\xfe", pos)
                    if s < 0:
                        break
                    e = payload.find(b"\xfd", s)
                    if e < 0:
                        break
                    frame = payload[s:e + 1]
                    pos = e + 1
                    self.frames += 1
                    if len(frame) > 4 and frame[4] == 0x27:
                        self.scope_frames += 1
                        if self.scope_frames in (1, 100) or self.scope_frames % 500 == 0:
                            self.log(f"<- scope stream running ({self.scope_frames} frames)")
                        continue
                    hz = decode_civ_freq(frame)
                    if hz:
                        self.freq_hz = hz
                        self.log(f"<- CI-V freq = {hz/1e6:.6f} MHz")
                    else:
                        self.log(f"<- CI-V frame: {hexdump(frame[:24])}"
                                 f"{'...' if len(frame) > 24 else ''}")


def main():
    ap = argparse.ArgumentParser(description="ICOM LAN login/CI-V tester")
    ap.add_argument("radio_ip")
    ap.add_argument("username")
    ap.add_argument("password")
    ap.add_argument("-p", "--port", type=int, default=50001)
    ap.add_argument("--local-base", type=int, default=50000,
                    help="base for local UDP ports (ctrl=base+1, civ=+2, audio=+3); use a distinct base per simultaneous session")
    ap.add_argument("--civ", action="store_true",
                    help="after login also open the CI-V stream and read frequency")
    ap.add_argument("--keep-scope", action="store_true",
                    help="do not disable the scope/waterfall CI-V output stream")
    ap.add_argument("--no-audio", action="store_true",
                    help="request stream with rxenable=0 (test whether the radio "
                         "opens CI-V without an audio channel — matters for ESP32)")
    ap.add_argument("-t", "--timeout", type=float, default=None,
                    help="overall deadline in s (default 15, with --civ 30)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    deadline_s = args.timeout or (30.0 if args.civ else 15.0)

    global LOCAL_BASE
    LOCAL_BASE = args.local_base
    ctl = ControlPort(args.radio_ip, args.port, args.username, args.password,
                      args.verbose)
    civ = None
    audio = None
    stream_requested = False
    civ_deadline = None
    next_freq_poll = 0.0

    deadline = time.monotonic() + deadline_s
    settle_until = None

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            ctl.tick(now)
            if civ:
                if ctl.stream_status and civ.radio[1] == 0:
                    error, civport, audioport = ctl.stream_status
                    if error == 0xFFFFFFFF:
                        ctl.log("stream refused (error=0xffffffff) — try rebooting the radio")
                        break
                    civ.radio = (args.radio_ip, civport)
                    audio.radio = (args.radio_ip, audioport)
                    ctl.log(f"opening CI-V channel to port {civport}, "
                            f"audio to port {audioport}")
                if audio and audio.radio[1] and not args.no_audio:
                    audio.tick(now)
                if civ.radio[1]:
                    civ.tick(now)
                    if civ.open_sent and now >= next_freq_poll:
                        if next_freq_poll == 0.0 and not args.keep_scope:
                            civ.scope_output_off()
                        civ.request_freq()
                        next_freq_poll = now + 2.0
                    if civ.freq_hz and civ_deadline is None:
                        civ_deadline = now + 2.0   # collect a bit more, then finish
                    if civ_deadline and now >= civ_deadline:
                        break

            # step 2 trigger: auth 0x05 acknowledged + capabilities identity known
            if (args.civ and not stream_requested and ctl.login_ok and ctl.radios
                    and ctl.token_ok):
                busy, computer = ctl.conninfo or (0, "")
                if busy and computer not in ("", "esp32-test"):
                    ctl.log(f"radio busy by {computer!r} — cannot open stream")
                    break
                # civ + audio channels; audio does the handshake, payload is dropped
                ctl.no_audio = args.no_audio
                audio = AudioPort(args.radio_ip, 0, args.verbose)
                civ = CivPort(args.radio_ip, 0, ctl.radios[0][2], args.verbose)
                ctl.send_stream_request(civ.local_port, audio.local_port)
                stream_requested = True

            if not args.civ:
                if ctl.token_ok and settle_until is None:
                    settle_until = now + 3.0
                if settle_until is not None and now >= settle_until:
                    break
            if ctl.login_err:
                break

            socks = [ctl.sock] + ([civ.sock] if civ and civ.radio[1] else []) \
                    + ([audio.sock] if audio and audio.radio[1] else [])
            readable, _, _ = select.select(socks, [], [], 0.05)
            for s in readable:
                try:
                    data, _addr = s.recvfrom(4096)
                except OSError:
                    continue
                if s is ctl.sock:
                    ctl.handle(data)
                elif civ and s is civ.sock:
                    civ.handle(data)
                elif audio and s is audio.sock:
                    audio.handle(data)

    except KeyboardInterrupt:
        print("\n(interrupted — tearing down)")

    # polite teardown
    if civ and civ.open_sent:
        civ.send_openclose(True)
        civ.send_control(0x05, 0x00, "civ-disconnect")
    if ctl.login_ok:
        ctl.send_token(0x01, "token-release")
        time.sleep(0.2)
    ctl.send_control(0x05, 0x00, "disconnect")

    print("\n=== VERDICT ===")
    if not ctl.got_here:
        print("No 'I am here' — radio unreachable on this IP/port, or WLAN Remote/"
              "Network Control is OFF.")
        sys.exit(3)
    if ctl.login_err:
        print("Login rejected — wrong Network User ID/password (radio: WLAN SET -> "
              "Remote Settings -> Network User1).")
        sys.exit(2)
    if not ctl.login_ok:
        print("Handshake reached the radio but login never completed — share the output.")
        sys.exit(4)
    if args.civ:
        if civ and civ.freq_hz:
            print(f"FULL SUCCESS — login OK, CI-V stream open, frequency "
                  f"{civ.freq_hz/1e6:.6f} MHz ({civ.frames} CI-V frames received).")
            sys.exit(0)
        print("Login OK but CI-V stream did not deliver data — share the output.")
        sys.exit(5)
    print("LOGIN + TOKEN OK — the LAN path works. Radios seen:",
          [(n, f"civ=0x{c:02x}") for n, _m, c, _cc, _g in ctl.radios] or "-")
    sys.exit(0)


if __name__ == "__main__":
    main()
