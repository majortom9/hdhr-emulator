# hdhr-emulator

Makes a USB ATSC tuner on a Raspberry Pi look and behave like an
early-Gen3 SiliconDust HDHomeRun on the wire — all three protocol
layers (UDP discovery, TCP control, HTTP + raw UDP unicast data planes)
— driving the tuner **directly via the Linux DVB API**. No TVheadend or
any other intermediary: this daemon owns RF tuning, signal-quality
reporting, PID filtering, and ATSC channel mapping (PSIP/PAT/PMT
parsing) itself.

Written in C against the actual on-wire HDHomeRun format documented in
Silicondust's `libhdhomerun` (LGPL) — packet framing, TLV tags, CRC32,
GETSET path names, and the device-ID checksum algorithm were all
verified against that source. No code from `libhdhomerun` is linked or
copied; this is a clean-room reimplementation using only the protocol
facts (which aren't copyrightable expression) from its headers. The
Linux DVB ioctl usage (frontend tuning, demux PID filters) is written
and compile-checked directly against the real kernel headers
(`linux/dvb/frontend.h`, `linux/dvb/dmx.h`).

**Validated on real hardware** (Raspberry Pi 3B+, USB ATSC tuner,
`lgdt3306a` demodulator, live off-air antenna) — see
[ARCHITECTURE.md](ARCHITECTURE.md) for the technical detail behind the
non-obvious parts of the design, most of which exist because of specific
things discovered during that testing (a demodulator driver quirk, real
client-side timing assumptions in `libhdhomerun`, an ambiguous wire-format
detail in `hdhomerun_config_gui`'s own manual channel control).

## Architecture

```
USB ATSC tuner (8VSB, US OTA only)
        |
   /dev/dvb/adapterN/{frontend,demux,dvr}0
        |
   dvb_frontend.c  — S2API tuning (DTV_DELIVERY_SYSTEM=SYS_ATSC,
   |                  DTV_MODULATION=VSB_8), lock status, signal stats
   |                  (DTV_STAT_SIGNAL_STRENGTH/CNR/ERROR_BLOCK_COUNT)
   |
   dvb_scan.c      — tunes every known US ATSC RF frequency
   (atsc_freq.c)      (2-36 — post-repack US usable range), and for any
                       that lock, reads PAT + each
   mpeg_section.c     program's PMT + the mux's ATSC TVCT (via
   psip.c             mpeg_section.c's demux section-filter wrapper +
                       psip.c's pure PAT/PMT/TVCT parsers) to populate...
        |
   dvb_channel.c   — the virtual channel database: major.minor ->
                      {frequency, program_number, PMT/PCR/video/audio
                      PIDs}, shared by every tuner slot
        |
   dvb_stream.c    — on demand: (re)tunes a physical adapter, sets
                      demux PID filters for one program (or full-mux
                      passthrough), captures the resulting TS from the
                      DVR device into a ring buffer
        |
   tuner.c         — per-slot claim/release so a control-plane target=
                      push and an HTTP pull can't both tune the same
                      physical adapter at once (see "Tuner contention"
                      below — this replaced TVheadend's own internal
                      arbitration)
        |
+-------+----------+----------+
|                  |          |
discovery.c    control.c   http_server.c
UDP 65001      TCP 65001   TCP 80
(find me)     (get/set)    (discover.json, lineup.json,
                             lineup_status.json, /auto/vX.X)
                    |
                    v
               udp_stream.c
               raw UDP unicast to a control-set "target=ip:port",
               7 x 188-byte TS packets per datagram (1316 bytes) —
               the classic early-HDHomeRun push packing
```

## Tuner contention — new since the TVheadend version

TVheadend used to own all tuner arbitration internally; now that this
daemon drives `/dev/dvb/adapterN` directly, **only one session can hold
a given physical adapter tuned at a time**. `tuner.c`'s
`tuner_try_claim()`/`tuner_release()` is the single point of truth for
this, shared between:
- control.c's `/tunerN/target=udp://...` push
- http_server.c's `/auto/vX.X` pull (auto-allocates any idle tuner slot)
  and `/tunerN/vX.X` pull (claims a specific slot)

If all tuner slots are busy, HTTP requests get `503 Service Unavailable`
and control-plane `target=` sets get an `ERROR: unable to start stream`
GETSET reply. This mirrors genuine hardware's real physical constraint
(N tuners = N concurrent streams, no more) — it's not an artificial
limitation.

## TVCT parsing — validated, but still the part most likely to need a fix in a new market

`psip.c` implements the ATSC A/65 TVCT (terrestrial virtual channel
table) byte layout from the published field structure. It's both
**self-consistently round-trip tested** (`make test` /
`test/psip_test.c` — a synthetic TVCT section is encoded by hand and
confirmed to parse back to the same values) and **validated against real
off-air PSIP** in one market (dozens of real channels scanned in with
correct major.minor numbers and names). The self-test alone couldn't
catch a shared misunderstanding between the encoder and decoder, since
both were written from the same understanding of the spec — real-hardware
testing is what actually confirms the decoder side is right, and it has
been, but only against one broadcast market's specific PSIP encoder
behavior so far.

If real channel numbers/names come back wrong, empty, or `dvb_scan`
logs "TVCT" parse failures on a mux where you know PSIP is present,
that's the first place to look — the byte offsets are commented
in-line in `psip_parse_tvct()` in `src/psip.c`.

PAT and PMT parsing are extremely stable, universal MPEG-2 Systems
structures used identically across every DVB/ATSC implementation in
existence — much lower risk than TVCT.

## What's compile-verified vs. what needs real hardware

**Compile-checked against real Linux kernel DVB headers** (this
container has them; the ioctl struct layouts, constants, and enum
values are the genuine ones, not guessed):
- `dvb_frontend.c` — S2API tuning/status/stats
- `mpeg_section.c` — demux section-filter setup
- `dvb_stream.c` — demux PES-filter setup, DVR device capture

**Unit-tested with synthetic data, no hardware needed:**
- `psip.c` — PAT/PMT/TVCT parsing (`make test`)
- `device_id.c` — HDHomeRun device-ID checksum (see earlier project
  history / `--gen-device-id`)

**Validated against real hardware** (Raspberry Pi 3B+, USB ATSC tuner,
`lgdt3306a` demodulator, live off-air antenna, real clients including
`hdhomerun_config`/`hdhomerun_config_gui` — both the pi's installed copy
and a freshly-built-from-GitHub latest release, and independently
cross-checked against a raw `tune-s2` run):
1. `dvb_frontend_tune_8vsb()` achieves lock reliably. This specific
   driver has a real quirk worth knowing about if you hit it on other
   hardware: `lgdt3306a`'s internal search() retries a lock *inside a
   single blocking ioctl call*, in an uninterruptible kernel sleep, for
   up to several seconds on a dead frequency — confirmed via `dmesg` and
   `/proc/<tid>/stat`. Not abortable from userspace by any means. See
   [ARCHITECTURE.md](ARCHITECTURE.md) §4 for how the control protocol is
   designed around this.
2. TVCT parsing works correctly against real off-air PSIP — dozens of
   real channels scan in with correct major.minor numbers and names,
   cross-checked against known local broadcasts.
3. `DMX_SET_PES_FILTER` + multiple filters feeding one
   `/dev/dvb/adapterN/dvr0` interleaves cleanly.
4. Full-mux passthrough (`program=0`) not yet independently re-verified
   after the DVB-direct rewrite's later fixes — the mechanism itself
   (`0x2000` wildcard PID) is unchanged and was working in earlier
   testing.
5. Signal-stat scaling in `dvb_frontend_read_stats()` (ss/snq/seq) is
   calibrated against this specific driver's actual dB/relative-scale
   conventions (see the calibration comments in `dvb_frontend.c`) and
   independently cross-checked against a raw `tune-s2` reading of the
   same hardware (-50.0 dBm / 24.4 dB SNR mapped to ~97%/~70% via the
   calibration formulas, matching live `/tunerN/status` readings almost
   exactly). Different tuner hardware will need its own calibration pass
   the same way — the formulas' min/max reference points are
   driver-specific, not universal.

## Build

Needs a C11 compiler and the Linux DVB kernel headers (`linux/dvb/*.h`)
— present on essentially any mainline Linux distro's default header
set; no separate package needed on most systems (they ship with glibc's
`linux-headers` or are already present under `/usr/include/linux/dvb/`).
No more libcurl/cJSON dependency — this version talks to hardware, not
an HTTP API.

```sh
make
make test    # optional: run the PSIP parser self-test (no hardware needed)
```

## Run

Needs to bind ports 65001 (UDP+TCP) and 80 (TCP), and read/write
`/dev/dvb/adapterN/{frontend,demux,dvr}0`:

```sh
sudo ./hdhr-emulator ./config/hdhr-emulator.conf.example
```

or install as a systemd service (runs as root, simplest way to get both
the low-port binds and DVB device access without fiddling with
capabilities + group membership separately):

```sh
sudo make install
sudo $EDITOR /etc/hdhr-emulator.conf   # set tuner_count, adapter mapping, device_id, etc.
sudo systemctl enable --now hdhr-emulator
```

Startup runs the ATSC scan (all US RF channels, ~1-2 minutes) in the
background on tuner0's adapter — discovery/control/HTTP come up
immediately, but `lineup.json` will be empty or partial until the scan
finishes. Watch the log for `dvb_scan: complete — N virtual channel(s)
found`.

## Generating a stable device ID

```sh
./hdhr-emulator --gen-device-id
```

prints one fresh, checksum-valid ID and exits — paste it into
`device_id=` in your config so it survives restarts (matters for
Plex/Emby DVR pairing, which keys off it).

## Layout

```
src/
  hdhr_pkt.*       HDHomeRun wire format: CRC32, TLV framing, frame open/seal
  device_id.*      HDHomeRun device-ID checksum generate/validate
  config.*         config file loading
  tuner.*          per-tuner state + physical-adapter claim/release
  discovery.*       UDP 65001 — DISCOVER_REQ/RPY
  control.*        TCP 65001 — GETSET_REQ/RPY dispatch
  http_server.*     discover.json / lineup.json / lineup_status.json /
                     auto/vX.X (TCP 80)
  udp_stream.*      raw UDP unicast TS push (the "target=" mechanism)
  atsc_freq.*        US ATSC RF channel -> frequency table (public FCC
                      channel plan)
  dvb_frontend.*      Linux DVB S2API: tune 8VSB, read lock + signal stats
  mpeg_section.*      demux section-filter wrapper (PAT/PMT/PSIP reads)
  psip.*              pure PAT/PMT/ATSC-TVCT section parsers
  dvb_channel.*       virtual channel database (major.minor -> PIDs/freq)
  dvb_scan.*          orchestrates the full-band scan
  dvb_stream.*        PID-filtered TS capture from /dev/dvb/adapterN/dvrM
  main.c              wiring + thread startup
test/
  psip_test.c          PAT/PMT/TVCT round-trip self-test (make test)
config/                example config
systemd/                service unit
ARCHITECTURE.md         deeper technical detail — the "why" behind the
                         non-obvious parts of the design (see especially
                         if you're touching control.c's channel-SET
                         handling or dvb_frontend.c's stat reporting)
```

## Known simplifications

- **`/tunerN/channel` SET refuses immediately only if a live stream is
  active** — if it's busy because of another channel scan already in
  flight, the new request queues (a bounded FIFO) and gets picked up by
  the same in-flight background worker rather than refusing or blocking.
  See [ARCHITECTURE.md](ARCHITECTURE.md) §4 for the full design and why
  it's this complicated (short version: a real driver quirk on the
  hardware this was developed against, colliding with real client-side
  timeout assumptions in `libhdhomerun`).
- **`seq` (symbol quality) falls back to a legacy calculation** on
  drivers that don't populate the modern `DTV_STAT_ERROR_BLOCK_COUNT`/
  `DTV_STAT_TOTAL_BLOCK_COUNT` stats (confirmed the case for this
  project's `lgdt3306a` driver, and common on other USB ATSC demod ICs) —
  a windowed calculation off the legacy cumulative
  `FE_READ_UNCORRECTED_BLOCKS` counter instead, both while actively
  streaming and during a channel scan (see
  [ARCHITECTURE.md](ARCHITECTURE.md) §5). If your driver doesn't support
  *either* the modern or legacy stat, `seq=0` is a real
  driver limitation, not a bug here — check with `debug_signal_stats=1`
  to see which case you're in.

- **`lockkey`** is stored and reported but not enforced against
  concurrent writers. Fine for a single-household LAN tool.
- **No incremental rescan** — `scan_on_startup=0` currently means an
  empty lineup until next restart; there's no runtime "trigger a
  rescan" control yet (`dvb_scan_run()` exists and could be wired to
  one easily — see main.c's `scan_thread_main`).
- **8VSB (US OTA) only** — no QAM/cable support, per project scope.
- **Program=0 full-mux passthrough** relies on the DVB API's `0x2000`
  wildcard PID, which is common but not universally implemented
  identically across every driver — see "What's compile-verified vs.
  what needs real hardware" above.
- **PSI isn't rewritten** — when filtering to one program, the original
  PAT (which may still list other programs whose PMTs/AV aren't
  actually present in the filtered output) passes through as-is rather
  than being rewritten to reference only the selected program. Every
  player tested against real-world partial-mux captures handles this
  fine in practice, but it's not spec-pure.

## Next steps

Done, via real-hardware testing (see [ARCHITECTURE.md](ARCHITECTURE.md)):
pointing at a real tuner + antenna and confirming channels scan in with
sane names/numbers; cross-checking scanned channels against known local
broadcasts; discovery + manual channel control end to end via both
`hdhomerun_config` and `hdhomerun_config_gui` (including the GUI's manual
channel spinner and continuous scan-up/down controls).

Still worth doing:
1. `ffprobe http://<pi-ip>/auto/v<major>.<minor>` — confirms the whole
   PID-filter-through-DVR-capture path produces a decodable stream (the
   control-plane/scan side is now well-tested; the actual media pipeline
   hasn't been re-verified since the later fixes).
2. A stream pull through a real DVR client (Plex, Channels DVR,
   Jellyfin) end to end, not just `hdhomerun_config`/`hdhomerun_config_gui`.
3. Full-mux passthrough (`program=0`) re-verification (see "what's
   compile-verified vs. what's real-hardware-validated" above).
4. Wire up a runtime "trigger a rescan" control — `dvb_scan_run()`
   already exists and does the right thing, it's just not reachable from
   the control protocol yet (see "Known simplifications").
5. Second-tuner (`tuner1`+) validation under real concurrent load — most
   testing so far has exercised one physical adapter at a time.
