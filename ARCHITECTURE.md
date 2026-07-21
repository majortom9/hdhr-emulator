# Architecture

This document explains *how* hdhr-emulator is built and, more importantly,
*why* several non-obvious design decisions exist — most of them were forced
by real hardware behavior discovered during live testing on a Raspberry Pi
3B+ with a USB ATSC tuner (`lgdt3306a` demodulator). The README covers
build/run/config; this document covers the internals worth understanding
before changing them.

## 1. Process shape

One process, four long-lived threads, plus short-lived per-connection
threads:

```
main()
 ├─ discovery_thread_main   — UDP 65001, one recvfrom() loop, replies inline
 ├─ control_thread_main     — TCP 65001, accept() loop, spawns conn_thread_main per connection
 ├─ http_thread_main        — TCP 80,    accept() loop, spawns conn_thread_main per connection
 └─ scan_thread_main        — one-shot, only if scan_on_startup=1 (default)
```

`main()` joins the first three (they run forever); the scan thread is
detached and exits once the initial full-band scan completes. There is no
event loop / reactor — every connection gets its own OS thread, and
blocking I/O (including deliberately-slow DVB ioctls) is fine because it
only blocks that one thread.

Two structs are shared across (almost) everything, both allocated once in
`main()` and never freed:

- `struct hdhr_config g_cfg` — read-only after startup (config file parsed
  once). No locking needed; nothing mutates it at runtime.
- `struct hdhr_tuner g_tuners[HDHR_MAX_TUNERS]` — one slot per configured
  tuner, each with its own `pthread_mutex_t lock`. This is the one truly
  shared, mutable, contended structure in the program. See §3.

`struct control_ctx { cfg, tuners }` is a thin read-only view of both,
passed to `control_thread_main` and reused (not duplicated) for
`http_thread_main`, since both need the same two pointers.

## 2. The three wire protocols

All three are clean-room reimplementations against the *public* on-wire
format documented in Silicondust's `libhdhomerun` headers (LGPL) — tag
values, framing, checksum algorithms. No `libhdhomerun` code is linked or
copied. See `hdhr_pkt.h`'s own header comment for the exact byte layout.

### 2.1 Discovery (UDP 65001) — `discovery.c`

Single-packet request/reply, no state. A `DISCOVER_REQ` with no tags (or
wildcard `DEVICE_TYPE`/`DEVICE_ID`) matches everyone; a request naming a
specific device that isn't us is silently dropped, matching real firmware
(devices don't reply to queries that aren't for them).

One byte-width detail that mattered in practice: `TUNER_COUNT` (tag
`0x10`) is a **1-byte** TLV on the wire. An earlier version of this code
wrote it as 4 bytes via the generic `_u32` TLV writer — syntactically
valid TLV, but a strict client parser checking `len==1` silently rejects
it and falls back to assuming 1 tuner, which is why `hdhomerun_config_gui`
used to show only one device row for a 2-tuner box. Confirmed byte-for-byte
against a real HDHomeRun3's own discovery reply via `tcpdump`.

### 2.2 Control (TCP 65001) — `control.c`

Persistent connection, sequential request/reply, one thread per connection
(`conn_thread_main`). Frames are `GETSET_REQ` (a `GETSET_NAME` TLV, plus an
optional `GETSET_VALUE` TLV for a SET) → `GETSET_RPY` (a `GETSET_VALUE` or
`ERROR_MESSAGE` TLV). The full path→leaf table (`/sys/*`, `/tunerN/*`) is
documented in `control.c`'s own header comment — that's the authoritative
reference for what each endpoint does; this document focuses on the *why*
behind the harder ones.

### 2.3 HTTP (TCP 80) — `http_server.c`

`discover.json` / `lineup.json` / `lineup_status.json` (what Plex/Emby/
Channels DVR/Jellyfin actually poll) plus `/auto/vX.X` and `/tunerN/vX.X`
for the live MPEG-TS pull itself. One thread per connection
(`conn_thread_main`, distinct from control.c's function of the same name —
each file's is `static`). `/auto/vX.X` auto-picks any idle tuner via
`tuner_pool_claim_free()`; `/tunerN/vX.X` claims a specific slot. Streaming
just loops `dvb_stream_read()` → `write()` until either end closes.

## 3. Tuner claim/release — the one shared resource

`struct hdhr_tuner` (`tuner.h`) is the single point of truth for "is this
physical `/dev/dvb/adapterN` currently in use." Every consumer — an HTTP
pull, a control-plane `target=` UDP push, a manual `/tunerN/channel` scan,
the startup full-band scan — goes through the same claim/release pair:

- **`tuner_try_claim()`** — instant fail if busy. Used by anything that
  should refuse immediately rather than queue: HTTP pulls, UDP `target=`
  pushes, and (as of the fix described in §4) the *first* attempt at a
  `/tunerN/channel` SET.
- **`tuner_try_claim_wait(timeout_ms)`** — blocks up to `timeout_ms`,
  woken by `tuner_release()`'s `pthread_cond_signal()`. Used by the
  startup scan (`main.c`, claims **per frequency**, not once for the whole
  scan — see the comment at `scan_thread_main` for why the earlier
  once-for-everything version made tuner0 a multi-minute dead zone after
  every restart).
- **`tuner_release()`** — clears `busy`, closes any `active_stream`, wakes
  one waiter, and (see §17) re-engages a background "hold" on the
  selected channel if one isn't already open, so `/tunerN/status` keeps
  reporting live stats immediately after streaming ends rather than
  going stale.

Every field on `struct hdhr_tuner` that isn't set-once-at-init is guarded
by its own `pthread_mutex_t lock` (`tuner_lock()`/`tuner_unlock()`) — not a
global lock, so different tuner slots never contend with each other.

`busy`/`active_stream` specifically mean "actively streaming right
now" — a **separate**, lower-priority concept, `held_fd`, tracks
"this tuner has a channel selected" (which persists across streaming
start/stop, and is a completely different physical-hardware-in-use
question from an active stream). See §16.

## 4. `/tunerN/channel` SET — why it's the most complicated endpoint

This is the endpoint that took the most iteration, because it's the one
place where a real hardware quirk collided with client-side timing
assumptions baked into `libhdhomerun` itself. Worth reading in full before
touching `control.c`'s channel-SET code.

### The hardware constraint

This project's `lgdt3306a` demodulator driver retries a lock internally
**inside a single ioctl call**, 5 times (`lgdt3306a_search: loop=0`
through `loop=4` — easy to misread the `0..4` range as 4 attempts, but
that's 5 inclusive), before giving up on a dead frequency — confirmed
via `dmesg` and via
`/proc/<tid>/stat` showing the calling thread in `D` (uninterruptible
sleep) state for the whole duration. This is not abortable from userspace
by any means — not a closed fd, not a signal (a `SIGUSR1`+`pthread_kill`
abort attempt was tried and disproved live: the thread stayed in `D` state
straight through the signal delivery). A **successful** lock is fast (well
under 2s observed); a frequency that ultimately fails to lock is what
triggers the slow path (~several seconds to over ten, observed).

### The client constraint

`libhdhomerun`'s control protocol (`hdhomerun_control.c`,
`HDHOMERUN_CONTROL_RECV_TIMEOUT`) gives **2.5 seconds per attempt, with one
resend** — so roughly 5 seconds of total patience per single GETSET
request before the client gives up with "communication error sending
request to hdhomerun device." `hdhomerun_config`'s own `scan` subcommand
fires its *next* channel's SET as soon as it gets *any* reply for the
current one — it does not wait for a previous request to time out before
moving on.

Combine the two: if a `/tunerN/channel` SET's reply is held hostage on a
synchronous tune+lock+PSIP-read of a frequency that happens to hit the
slow path, that single request can blow the client's ~5s total patience —
and because the client already moved on to the *next* channel as soon as
it got a reply for the current one, any naive re-attempt at a purely
synchronous design just relocates the same problem one channel later.

### The design that actually works

1. **Claim non-blocking first** (`tuner_try_claim`). If the tuner's
   already busy because of another `/tunerN/channel` scan in flight (not a
   live stream), the new frequency is pushed onto a **bounded FIFO**
   (`t->pending_queue`, `tuner.h`) and the SET replies *immediately* with
   a `lock=none` placeholder. A live stream has no worker to hand off to,
   so that case still fails fast with `ERROR: tuner busy`.

   An earlier version used a single overwritable "pending" slot instead of
   a real queue, reasoning that "only the newest request matters." That
   was true when replies were still slow enough to naturally pace the
   client — but once replies became fast (this very fix), the client fired
   through the whole 35-channel band in about a second per channel, and
   every request but the last got silently clobbered before the
   in-flight worker ever reached it. Confirmed live: 33 of 35 channels in
   a manual scan never got attempted at all, including ones with real
   signal. A real FIFO (sized above `ATSC_FREQ_TABLE_COUNT`) fixed that.

2. **Tune + lock run in a detached background thread**
   (`channel_scan_thread_main`), never blocking the connection thread on
   the driver's worst case. The connection thread waits on a condition
   variable for up to `CHANNEL_SET_WAIT_MS` (1.8s — comfortably above a
   real lock's ~2s, comfortably below the driver's dead-frequency worst
   case) for just the *lock* result, not the full PSIP read, then replies
   with whatever's known. If the background thread is still working when
   the budget expires, it (not the connection thread) becomes responsible
   for finalizing tuner state once it does finish.

3. **The worker drains its own queue** instead of exiting after one
   frequency: once it finishes an attempt (locked or not), it checks
   `t->pending_queue` before calling `tuner_release()`, and loops around to
   the next queued frequency if there is one. This is what lets
   back-to-back scan requests hand off to the *same* in-flight worker
   rather than each new request racing to claim the tuner itself.

4. **`dvb_frontend_wait_lock()`'s own timeout tracks real elapsed time**
   (`clock_gettime(CLOCK_MONOTONIC)`), not a naive per-iteration counter.
   An earlier version just did `elapsed += step_ms` once per 50ms poll
   iteration, silently assuming every iteration takes about that long.
   That held while `FE_READ_STATUS` was the only ioctl in the loop — once
   a per-iteration progress callback (see §5) started doing *more* ioctls
   that could themselves occasionally block for multiple seconds on this
   driver, a handful of slow iterations could stretch a nominal 1.5s
   timeout into minutes of real wall-clock time before the loop's own
   counter ever caught up. Fixed by measuring actual time instead of
   counting iterations — general lesson: never assume a "fast, read-only"
   ioctl is safe to call frequently just because it isn't the tune/lock
   ioctl, on a driver already known to have uninterruptible-sleep quirks.

Net effect: every `/tunerN/channel` SET replies within `CHANNEL_SET_WAIT_MS`
regardless of how slow the underlying tune turns out to be, and every
queued frequency still eventually gets a genuine attempt — the client
never blocks long enough to time out, and the shared channel database
still ends up fully populated.

## 5. Live signal-stat exposure during a scan

`/tunerN/status` needs to report `ss=`/`snq=`/`seq=` accurately *while a
scan is still in progress*, not just once it's known to be locked or
failed — and this turned out to matter far more than cosmetics.

`libhdhomerun`'s own `hdhomerun_device_wait_for_lock()` polls
`/tunerN/status` every 250ms for up to 2.5s, and treats `ss < 45` as
"confirmed no signal, stop waiting immediately." If a scan's `/tunerN/channel`
SET replies before the tune is fully resolved (see §4 — it can, on
purpose), and the status the client polls right after still shows the
placeholder `ss=0`, the client can't tell "no signal" from "haven't
finished checking yet" and bails out on the very first poll — which
looked, from the outside, exactly like the client racing ahead
carelessly. It wasn't; it was working exactly as designed against
inaccurate input.

Similarly, `hdhomerun_channelscan.c`'s `channelscan_find_lock()` has a
*second* wait phase after basic lock: it polls for `symbol_error_quality`
(`seq=`) to reach 100 for up to **5 seconds** before it'll even start
checking for programs. Always reporting `seq=0` during a scan made this an
unconditional 5-second stall on every single locked channel.

The fix, in `control.c`:

- `dvb_frontend_wait_lock()` takes an optional progress callback
  (`dvb_frontend_progress_cb`), invoked from the *same thread that owns
  the frontend fd* on every poll iteration that completes — deliberately
  never a cross-thread ioctl, since a concurrent status-read could itself
  block behind this driver's known-uninterruptible search().
- `channel_scan_thread_main` passes `publish_scan_stats()`, which reads
  live `dvb_frontend_read_stats()` and writes a snapshot into
  `t->scan_stats` (tagged with `scan_stats_freq` so a reader can detect
  staleness the same way `finalize_lock_result()` does for `t->status`).
- `/tunerN/status`'s scan branch reports those live numbers — **except**
  the `lock=`/`none` word itself, which is read from `t->status` (the
  authoritative, once-finalized result), not from `scan_stats.has_lock`.
  `scan_stats` only refreshes every ~250ms, so right at the moment a
  channel actually achieves lock there's a window where the real result
  already says locked but the last-published snapshot hasn't caught up —
  confirmed live as a client reading `lock=none ss=97 snq=70` for a
  channel that had, in fact, already locked.
- This driver doesn't populate the modern DVBv5
  `DTV_STAT_ERROR_BLOCK_COUNT`/`DTV_STAT_TOTAL_BLOCK_COUNT` stats at all,
  so `symbol_quality_pct` (`seq=`) falls back to
  `dvb_frontend_legacy_seq_pct()` — a windowed calculation off the legacy
  cumulative `FE_READ_UNCORRECTED_BLOCKS` counter (needs two samples
  ≥0.5s apart on the *same* frequency to produce anything). Since polling
  stops the instant `FE_HAS_LOCK` is detected, the common fast-lock case
  otherwise never accumulates that window at all — so on a successful
  lock, `channel_scan_thread_main` deliberately publishes one more sample,
  sleeps 600ms, and publishes again, before starting the PSIP read. That
  short pause is what lets `channelscan_find_lock()`'s settle-wait see a
  real `seq=100` instead of always burning its full 5-second budget.

None of this helps the genuinely-blocked-inside-one-ioctl case (§4) — there's
no way to get partial progress out of a syscall that hasn't returned yet.
The FIFO queue remains the safety net for that; this section only improves
the (much more common) case where a channel resolves normally.

## 6. `"auto:"` is ambiguous by design

`parse_channel_value()` (`control.c`) has to distinguish two different
things sent under the same `"auto:"` prefix:

- The scan path (`hdhomerun_channelscan.c`'s `channelscan_find_lock()`)
  sends `"auto:<freq_hz>"` — e.g. `"auto:497000000"`.
- `hdhomerun_config_gui`'s manual channel-number up/down spinner sends
  `"auto:<channel_number>"` — e.g. `"auto:32"` — the small number *is* the
  RF channel number, not a frequency.

Real firmware evidently distinguishes by magnitude. Since ATSC channel
numbers here only ever run 2–36 and any real RF frequency is tens of
millions of Hz, the code treats values below 1000 as a channel number
(resolved via `atsc_channel_to_freq()`) and anything at or above as a raw
frequency. `"8vsb:"` stays unambiguous (always an explicit frequency) —
only `"auto:"` needs this. Confirmed via live `tcpdump` capture of the
GUI's actual spinner clicks; before this fix, the spinner silently tuned
to a literal 32 Hz "frequency" every time, which of course never locked.
The GUI's continuous Scan Up/Down toggle buttons are built entirely on top
of the same spinbutton mechanism (`gtk_spin_button_set_value()` in a
timer-driven loop), so this one fix made all three GUI channel-changing
controls (manual scan, the number spinner, the scan arrows) work
correctly at once.

`"us-bcast:<N>"` is accepted here too (`parse_channel_to_freq()`,
resolved the same way via `atsc_channel_to_freq()`), but that's this
project's own convenience addition, not something real firmware
actually accepts on `/tunerN/channel` — confirmed live against a
genuine HDHomeRun3: `set /tuner0/channel us-bcast:7` returns `"ERROR:
invalid channel"`, while `set /tuner0/channel auto:177000000` works.
`hdhomerun_channelscan.c` only ever sends `"auto:<freq_hz>"` (it
resolves a channel number to a frequency itself, client-side, from its
own channel-map table, before sending anything). Worth knowing if
you're driving a real device the same way you'd drive this emulator —
`us-bcast:N` won't work there.

## 7. Channel scanning & PSIP pipeline

`dvb_scan.c` provides two composable primitives rather than one monolithic
scan function, specifically so `control.c` can run them from a background
thread per-SET (§4) using the exact same logic the startup scan uses:

- **`dvb_scan_tune_and_lock()`** — opens the frontend, tunes 8VSB, waits
  for lock. On success, leaves the fd **open** and returns it (the caller
  decides what to do next); on failure, closes it itself.
- **`dvb_scan_read_psip()`** — given an already-locked fd, reads PAT (PID
  `0x0000`), each program's PMT (PID from the PAT), and the mux's ATSC
  TVCT (PID `0x1FFB`, via `mpeg_section.c`'s section-filter wrapper +
  `psip.c`'s pure parsers), merges anything found into the shared
  `dvb_channel` database, and **always** closes the fd before returning.

`main.c`'s `scan_thread_main()` and `control.c`'s
`channel_scan_thread_main()` are both just loops calling these two
functions in sequence around the tuner claim/release dance described in
§3/§4 — there is no separate "scan engine," just these two primitives
reused in two different concurrency contexts.

TVCT parsing (`psip_parse_tvct()` in `psip.c`) is the piece most likely to
need attention on a new broadcast market: PAT/PMT are extremely stable,
universal MPEG-2 Systems structures, but TVCT's self-test
(`test/psip_test.c`) can only catch a shared encoder/decoder
misunderstanding of the ATSC A/65 spec, not a real-world PSIP quirk.

## 8. Streaming pipeline

`dvb_stream.c` owns the actual RF-tuned-to-bytes-on-a-socket path, shared
by both HTTP pulls and UDP `target=` pushes:

1. `dvb_stream_open()` tunes the frontend (independently of whatever
   `/tunerN/channel` last did — a stream open is a fresh tune), sets up
   demux PID filters, and spawns a reader thread that pulls from
   `/dev/dvb/adapterN/dvrM` into an internal ring buffer. Which PIDs get
   filtered follows a precedence: an explicit `pid_filter` argument (from
   `/tunerN/filter` — see §11) wins outright if it's small enough to
   enumerate as individual demux filters; otherwise `program_override`
   picks specific program/PCR/video/audio PIDs, or the `0x2000` full-mux
   wildcard PID for `program=0` or a `pid_filter` too wide to enumerate.
2. Consumers (`http_server.c`'s `stream_channel_to_client`,
   `udp_stream.c`'s `push_thread_main`) call `dvb_stream_read()` in a loop
   and forward bytes onward — to a `write()` on the HTTP socket, or
   packed 7×188-byte TS packets (1316 bytes) per UDP datagram for the
   classic early-HDHomeRun push format.
3. `dvb_stream_frontend_fd()` gives `control.c`'s `/tunerN/status` handler
   read-only access to the same fd for live signal stats while genuinely
   streaming (as opposed to the scan-time path in §5, which never has an
   active `dvb_stream` at all).
4. `tuner_release()` calls `dvb_stream_close()` internally, so
   HTTP/UDP-push code never has to remember to do it themselves — closing
   the tuner claim and closing the stream are the same operation from a
   caller's perspective.

## 9. Reclaiming an abandoned `target=` push

UDP has no inherent "connection dropped" signal the way TCP does. A
`target=` push (`udp_stream.c`) keeps a tuner claimed and streaming to
whatever `ip:port` a client last set, and nothing about a `sendto()`
failing tells you the *client* is actually still there to receive it —
a client that crashes, gets `kill -9`'d, or just drops off the network
without sending `target=none` first leaves that tuner locked to a dead
destination forever. Reproduced live: `kill -9`'ing an in-progress
`hdhomerun_config save` left `/tunerN/target` reporting the dead
destination indefinitely, with no way to notice from the server side
alone.

Real `libhdhomerun` clients already send the fix for this, they just
weren't being listened to: `hdhomerun_video_thread_send_keepalive()`
(`hdhomerun_video.c`) sends a small UDP packet (the tuner's lockkey) to
the device's UDP port 5004 roughly once a second for as long as
`hdhomerun_device_stream_start()`/`stream_recv()` are being used.
`keepalive.c` binds that port and, for each packet, matches its
*source* address against `t->keepalive_match_addr`/`keepalive_match_port`
(binary form of the current push's destination, set by
`udp_push_start()`) — a match means "this tuner's client is still alive,"
since the client's own outbound keepalives share the same local socket
(and therefore the same `ip:port`) it registered via `target=`.

One thread does double duty: it binds the socket with a `SO_RCVTIMEO` of
`SWEEP_INTERVAL_MS` (2s), so every `recvfrom()` either gets a real
keepalive packet or simply times out — either way, it then sweeps every
tuner and reclaims (`udp_push_stop()`, the same path an explicit
`target=none` uses) any whose `last_keepalive_time` is more than
`KEEPALIVE_TIMEOUT_MS` (15s — no reference for real firmware's own
value, chosen as ~10x+ the client's ~1s cadence) stale. This piggybacks
the staleness check on the socket's own timeout rather than running a
separate timer thread, so a total silence (no keepalive traffic at all,
e.g. a raw non-`libhdhomerun` client that never implements this) still
gets swept on the same cadence as a real packet arriving would.

One subtlety that showed up in testing: `udp_push_stop()` blocks up to
~2s polling for the push thread to actually notice `stop_requested` and
exit — which can occasionally run longer than one sweep tick if
`dvb_stream_read()` is itself blocked waiting on the capture thread.
The sweep checks `t->udp_push_stop_requested` before re-triggering a
reclaim already in flight, otherwise a single stale push logged (and
re-polled) the same reclaim two or three times before it actually
finished stopping.

`udp_stream.c`'s `push_thread_main()` also gained a related fix while
this was being built: it now resets `t->target` to `"none"` on *every*
exit path (upstream EOF, a `sendto()` error, or this new reclaim), not
just an explicit `target=none` SET — before, `/tunerN/target` could keep
reporting a destination that had, in fact, already stopped receiving
anything.

## 10. `/tunerN/filter` — a lower-level override than `program`

Confirmed against a genuine HDHomeRun3 (same antenna feed, real device):
`filter` defaults to `"0x0000-0x1fff"` (the full 13-bit PID space,
i.e. unfiltered) whenever nothing more specific has been selected, and
setting `/tunerN/program` **automatically recomputes it** to that
program's actual PMT/PCR/video/audio PIDs — e.g. `"0x0030-0x0031 0x0034"`
for a channel whose PMT and PCR/video share adjacent PIDs. Interestingly,
the real device's recomputed string excludes PAT (`0x0000`) even though
PAT is still streamed regardless — the displayed filter reflects the
*program-specific* PIDs added on top of an implicit PAT, not the
literal complete PID set actually demuxed. `filter` is also
independently settable, and doing so takes over PID selection entirely,
overriding whatever `program` would have picked.

This project's implementation (`pid_filter.c`/`.h` for the wire format,
`dvb_stream_open()`'s new `pid_filter` parameter for applying it, `tuner.h`'s
`filter_override` string for tracking an explicit one) follows the same
shape:

- **GET** returns `filter_override` verbatim if a client has explicitly
  set one; otherwise `control.c`'s `compute_auto_filter()` derives it
  fresh from the tuner's current channel+program state, on every read —
  same PMT/PCR/video/audio resolution `dvb_stream_open()` itself would
  use, formatted (and range-merged) by `pid_filter_format()`, PAT
  excluded to match the real device's own display convention.
- **SET** parses via `pid_filter_parse()`; `"none"`/empty clears the
  override back to auto-derived. Real usage streams directly off
  `channel`+`program`+`filter` without ever touching `vchannel` — the
  same raw, PSIP-independent workflow §12 covers for `channel`+`program`+
  `target=`.
- **Precedence and clearing**: `filter_override` is cleared whenever
  `channel`, `vchannel`, or `program` changes — matching the real
  device's own auto-recompute-on-`program`-change behavior, and
  preventing a filter tied to a now-irrelevant mux/program from silently
  persisting onto whatever gets selected next.
- **Applying it** (`dvb_stream_open()`): an explicit filter small enough
  to fit as individual demux PES filters (`MAX_DEMUX_FDS - 1` PIDs,
  reserving one slot for PAT) gets enumerated one PID at a time, same
  mechanism `add_pid_if_new()` already used for program-based selection.
  A filter too wide to enumerate — including the real device's own
  `"0x0000-0x1fff"` default — falls back to the existing `0x2000`
  full-mux wildcard instead of failing outright: a hardware demux has
  nowhere near enough concurrent PID-filter slots to open one per PID
  across a range that size.

Same established scope as `program` (§4 predates this, but the
asymmetry is pre-existing, not new here): only the `target=` push path
consults `filter`/`program` at all. `http_server.c`'s HTTP passthrough
always streams the plain named channel's own default PIDs, ignoring
both.

## 11. `/sys/restart` and `/sys/copyright` — deliberate departures from real firmware

Two `/sys/` leaves exist specifically because *matching* real firmware
exactly would be the wrong choice here:

- **`/sys/restart <resource>`** is real (confirmed via a genuine
  HDHomeRun3's own `get help`), but the wire protocol has no real
  authentication — `lockkey` is an advisory courtesy value real clients
  set, not a security boundary — so implementing it unconditionally
  would let *any* client on the LAN stop the daemon. `allow_remote_restart`
  in the config (default `false`) makes it an explicit, deliberate
  admin opt-in instead. Real firmware's exact `<resource>` semantics
  aren't recoverable from the client library (it's a raw passthrough
  SET with no documented values) and don't matter here anyway — this
  daemon is one process regardless of what resource is named, so any
  accepted value triggers a full `exit(0)`. No in-place re-exec; a
  supervisor (systemd's `Restart=always`) is what's expected to bring it
  back, matching how `systemd/hdhr-emulator.service` is already set up.
  The reply is sent to the client *before* exiting — already handed to
  the kernel's socket buffer by the time `send_value_reply()` returns,
  so it survives the `exit()` — so a well-behaved client sees a clean
  acknowledgment rather than a communication error.
- **`/sys/copyright`** deliberately does **not** echo real firmware's
  actual value, a literal Silicondust copyright notice. Doing so would
  misattribute this project's own independently-written, clean-room
  code (see README.md's opening paragraphs) to Silicondust — a
  different kind of claim than the wire-protocol mimicry the rest of
  this daemon does, where matching real behavior *is* the entire point.
  The value here is this project's own short statement instead: name,
  repo link, and an explicit "not Silicondust software" disclaimer.

## 12. `target=` doesn't require `vchannel` — and neither should `filter`/`program`

`udp_push_start()` (`udp_stream.c`) originally required `t->vch_resolved`
— only ever set by `/tunerN/vchannel` — before it would start a push,
which meant the classic raw-protocol workflow (`set /tunerN/channel
auto:<freq>`, `set /tunerN/program <N or 0>`, `set /tunerN/target
udp://...`, never touching `vchannel` at all) failed outright with
`"ERROR: unable to start stream (no channel set?)"`, even though
`program=0`'s full-mux passthrough needs nothing more than a tuned
frequency to mean something.

Fixed by falling back to `dvb_channels_on_freq(t->tuned_frequency_hz, ...)`
when no named virtual channel is resolved: any channel already known on
that mux works as `dvb_stream_open()`'s base struct, since it only
supplies the PAT/PMT/PID lookup starting point — `dvb_stream_open()`
itself already resolves `program_override`/`pid_filter` against the
mux's siblings independently of which sibling was passed in. Real usage
commonly drives `target=` this way — a manual RF tune plus an explicit
program number or PID filter, with no PSIP name resolution involved at
all — so requiring `vchannel` first was a gap relative to real behavior,
not a deliberate restriction.

## 13. Never report a literal `0` for `ss=`/`snq=`/`seq=`

Two different situations could both produce a bare `0` in the wire
protocol's `ss=`/`snq=`/`seq=` fields, with no way for a client to tell
them apart: a genuinely weak-but-present reading clamped at the
calibration floor, and this daemon's own `-1` ("no reading available at
all") sentinel getting collapsed into `0` for display, since the
ASCII status line has no separate N/A slot. That distinction is more
than cosmetic — `libhdhomerun`'s own `hdhomerun_device_wait_for_lock()`
treats a low `ss` as "confirmed no signal, stop polling immediately"
(the exact behavior §5 is built around not tripping prematurely), so a
client genuinely cannot distinguish "nothing here" from "no data yet"
once both read `0`.

`dvb_frontend.c`'s `clamp_pct_floor1()` floors every *genuinely
available* reading at `1` instead of `0` (used by `scale_decibel_to_pct()`,
`scale_relative_to_pct()`, and both post-FEC error-ratio calculations);
`control.c`'s two status-line formatters collapse the `-1` sentinel to
`1` as well, for the same reason — rather than pick which of the two
cases gets to keep `0`, this daemon just never emits it in these fields
at all.

## 14. Raw signal-strength dBm is an AGC readout, not a power measurement

Worth internalizing before trusting `DTV_STAT_SIGNAL_STRENGTH` too
literally: the kernel DVBv5 API defines its decibel scale as "0.001 dBm
units, power" (see `dvb_frontend_read_stats()`'s own comment), which
reads like an absolute, calibrated measurement at the antenna
connector. It isn't, in practice. There's an AGC (automatic gain
control) loop between the tuner IC's variable-gain stages and the
demod — the demod (or sometimes the tuner) reads back whatever gain
setting the loop settled on to hit its target signal level, and the
driver converts *that* into a "dBm" number via its own lookup
table/formula. The number reported is downstream of that conversion,
not an independent measurement of what's actually arriving at the
F-connector.

This matters because the AGC-to-dBm conversion is characterized
per-driver, per-tuner-IC — two different tuners can legitimately report
different "dBm" for identical real RF power, and there's no way to tell
from the number alone whether that's a real difference in received
power or just a difference in how optimistically each driver's author
calibrated their conversion table. Confirmed directly: comparing
pi3b's tuner against a physically different USB ATSC dongle (`pi4`,
same LGDT3306A demod, different tuner front-end IC) on the same
antenna/preamp feed across three signal levels, pi3b consistently
reported ~6.9 dB *stronger* ss than pi4 for the identical actual
signal — yet at the weakest tested level, pi3b locked fewer real
channels than pi4 did (4/35 vs. 9/35), not more. The two metrics
flatly disagreed: the tuner with the more optimistic self-reported dBm
number was the one that actually struggled more. Whatever's costing
pi3b real sensitivity there isn't visible in its own dBm number, since
that number was never an independent measurement to begin with — see
`pi3b_vs_pi4_report_2026-07-19.md` (generated during this comparison,
not committed to the repo) for the full per-channel data.

Practical consequence: this is exactly why this project's own
`signal_strength_pct` floor/ceiling calibration (see the comments
above that field in `dvb_frontend_read_stats()`, and README.md's
"Calibrating signal stats against real hardware") has to be validated
against a real device's actual `ss%`/lock *behavior*, not treated as a
matter of getting the "true" dBm-to-percent conversion right in some
absolute sense — there isn't one. A calibration pass tuned against one
tuner's AGC characteristics doesn't transfer to a different tuner IC
without its own real-device comparison, even when both run the same
demod chip.

## 15. QAM/cable channel maps — implemented, UNTESTED against real signal

Everything in this section is compile-verified against this project's
own DVB hardware's kernel headers, and the frequency data is verified
against a real source (see below) — but none of it has ever actually
locked a real cable channel, unlike `us-bcast`'s thorough real-hardware
validation throughout the rest of this document. Treat it accordingly.

**Frequency tables** (`channel_map.c`/`.h`): a genuine HDHomeRun3's own
`get help` lists six channel maps (`us-bcast us-cable us-hrc us-irc
kr-bcast kr-cable`); `us-bcast` already had its own real-hardware-
validated table (`atsc_freq.c`) and was deliberately left alone rather
than folded into this more general system. The other five come from a
real copy of `libhdhomerun`'s `hdhomerun_channels.c`
(`hdhomerun_channelmap_range_*` tables) — protocol facts (channel
ranges, base frequencies, per-channel spacing), not copied code, same
clean-room approach as the rest of this project (see README.md). One
useful simplification fell out of that source directly: `kr-bcast`
isn't a distinct Korean frequency plan at all — libhdhomerun points it
at the exact same range table as `us-bcast`, just under a different
country-code label, so `channel_map.c` does the same rather than
maintaining a duplicate table.

**QAM tuning** (`dvb_frontend_tune_qam()`): `SYS_DVBC_ANNEX_B` (the
Linux DVB API's North American cable QAM delivery system — confirmed
present as an available delivery system on this project's own
LGDT3306A frontends via `dvb-fe-tool`, alongside ATSC) with
`QAM_AUTO` modulation and a fixed 5.360537 Msym/s symbol rate — the
near-universal standard for 6MHz-spaced North American in-band clear
QAM regardless of 64- vs. 256-QAM constellation. `QAM_AUTO` means the
driver blind-detects which constellation a given channel actually
uses, same "don't make the caller pre-know something the hardware can
figure out" spirit as `dvb_frontend_tune_8vsb()` not taking a
modulation parameter either — real cable operators mix 64-QAM and
256-QAM freely, sometimes on the same system.

**CVCT reuse**: ATSC A/65's CVCT (cable virtual channel table, table_id
`0xC9`) shares TVCT's (`0xC8`, terrestrial) wire format byte-for-byte —
`psip_parse_tvct()` doesn't even check `table_id` internally, so
`dvb_scan.c`'s `read_vct()` just opens a section filter on whichever
table_id the delivery system calls for and feeds it the same parser.

**No-CVCT fallback**: real cable operators inconsistently carry usable
CVCT at all. Dropping an entire mux just because PSIP isn't present
would throw away real, playable programs that PAT+PMT already resolved
fine — `dvb_scan_read_psip()` falls back to exposing each such program
directly as `<rf_channel>.<program_number>` (e.g. `"24.3"`) rather than
skipping it, matching how real hardware/software (e.g. TVheadend)
commonly handles PSIP-less clear QAM.

**Delivery system travels with the channel, not the tuner**:
`dvb_stream_open()` (the actual streaming tune, shared by `target=` and
HTTP pulls — see §8) only ever receives a `struct dvb_channel` pointer,
not whatever channelmap was active when that channel was originally
scanned. So `dvb_channel` itself carries a `delivery` field
(`HDHR_DELIVERY_ATSC_VSB`/`HDHR_DELIVERY_QAM`), stamped once at scan
time (`dvb_scan.c`) and read back by `dvb_stream_open()` to pick the
right tune function — without this, a scanned QAM channel would
correctly show up in the lineup but then fail to actually stream, since
the frontend would try to tune it as 8VSB.

**Channel-value parsing is channelmap-aware, but decoupled from the
value's own prefix**: `control.c`'s `parse_channel_value()` takes the
tuner's *currently active* `/tunerN/channelmap` as a parameter, and
`"auto:<N>"`/`"8vsb:<freq>"`/`"qam:<freq>"` all resolve against
whichever map that is — matching real behavior, where the modulation
actually used for tuning is driven by what `/tunerN/channelmap` is
configured to, not by which prefix string a client happened to send.
An explicit `"<mapname>:N"` prefix (e.g. `"us-cable:23"`) overrides
that and resolves against the named map regardless of what's currently
active — this is what the smoke test below actually exercised.
Setting `/tunerN/channelmap` itself clears any tuned channel/vchannel
state, since a channel *number* means something different in every map
(e.g. "7" is 177MHz in `us-bcast` but a different frequency in
`us-irc`).

**What was actually checked, and what wasn't**: with no real cable
feed available, verification stopped at compiling cleanly against real
kernel headers on both target Pis, and a live smoke test confirming the
plumbing doesn't crash or misbehave: `/sys/features` matches a real
device's exact response, `/tunerN/channelmap` GET/SET works and rejects
unknown maps, and `set /tunerN/channel us-cable:23` correctly resolved
to exactly 219000000 Hz (matching the verified table), attempted a real
`SYS_DVBC_ANNEX_B` tune, and failed to lock *gracefully* (no signal —
expected) with a correctly `"qam:"`-prefixed status string, all without
disturbing the daemon's other tuner or crashing. None of that proves
the QAM tuning parameters, CVCT parsing, or the no-CVCT fallback are
actually *correct* against a real cable signal — only that they don't
fall over. The `"qam:<freq_hz>"` string this daemon echoes back for
`/tunerN/channel` on a locked cable channel is also an unverified
guess at what real firmware would report (the ATSC equivalent,
`"8vsb:<freq_hz>"`, was confirmed against a real device's actual
display; there was no real cable-tuned device available to confirm
against here).

## 16. A selected channel stays engaged even when nothing's streaming

Confirmed live (2026-07-20): with a channel selected via `/tunerN/vchannel`
but nothing actively streaming it, `/tunerN/status` reported a
completely frozen `ss=91 snq=89 seq=100` across 8 polls spanning 16
seconds. That's `scan_stats` (§5) — a one-time snapshot from whichever
scan/lock-verification attempt last ran, never refreshed again once
that attempt concluded, by this project's original design: RF tuning
was deliberately lazy, deferred until an actual stream (`target=`/HTTP)
opened, specifically so a merely-*selected* channel didn't tie up a
physical tuner slot other consumers might want (§3's original
framing). Real firmware doesn't work this way — it engages a selected
tuner's frontend immediately and keeps it engaged continuously,
regardless of whether anyone's actively pulling video, and just treats
that tuner as unavailable to other consumers the whole time. This
matters for more than fidelity to real behavior: third-party signal-
monitoring tools that poll many HDHomeRuns' channel+status without
ever actually streaming them would see this daemon's stats go stale
the instant a scan/verify finished, which a tool comparing against
real devices could easily read as "this unit stopped working."

**`held_fd`** (`tuner.h`) is a background frontend fd, separate from
`active_stream`, kept open+tuned to `tuned_frequency_hz`/`tuned_delivery`
for as long as a channel stays selected — independent of whether
anything is actively streaming it. `/tunerN/status` (`control.c`) now
checks it as a second priority tier, between `active_stream` (unchanged,
still authoritative when actively streaming) and the `scan_stats`
snapshot (unchanged, still used for live progress *during* an
in-flight scan/lock attempt, before this tier's own hold exists yet):
reads `dvb_frontend_read_stats()` fresh on every single poll, exactly
like the streaming branch does, so `ss=`/`snq=`/`seq=`/`lock=` are
always genuinely current — confirmed live: `snq` visibly varied
(89%↔91%) across repeated polls with nothing streaming, and `lock=`
now correctly reflects the true demod state immediately (previously,
`scan_stats`'s fallback path sourced its `lock=` word from `t->status`
specifically to dodge a *different* staleness issue described in §5 —
but that word itself was set once by the scan and never updated
either, so a channel that was, in fact, solidly locked could still
read `lock=none` indefinitely until the next scan).

**Two new tuner fields** support this: `tuned_delivery` (set alongside
`tuned_frequency_hz` everywhere it's written — `tuner_bind_channel()`
and the raw `/tunerN/channel` SET path — so a later re-tune knows 8VSB
vs. QAM without a channel-database lookup) and `held_legacy_seq` (this
driver's `seq=` fallback, §5, needs its own persistent windowed state;
a held tune has no `struct dvb_stream` to own that the way active
streaming does, so it lives directly on the tuner instead, reset
whenever a new hold opens).

**Lifecycle, and why it's centralized in `tuner.c` rather than at each
call site**: `tuner_open_hold()`/`tuner_close_hold()` do the actual
open/tune/close work (non-blocking — `dvb_frontend_tune_8vsb()`/
`tune_qam()` don't wait for lock, so opening a hold never risks the
dead-frequency blocking hazard §4 goes to such length to avoid; a
held-but-unlocked channel's *later* status polls can still hit that
hazard inside `dvb_frontend_read_stats()`'s ioctls, same pre-existing
exposure the active-streaming branch already has, not something new
here). Rather than have every consumer (`vchannel` SET, the `channel`
SET scan thread, `udp_stream.c`, `http_server.c`) separately manage
opening/closing a hold around its own claim, the two existing choke
points every consumer already passes through absorb it entirely:

- **`tuner_try_claim()`/`tuner_try_claim_wait()`** close an existing
  hold the instant a claim succeeds — a held fd and an `active_stream`'s
  own frontend fd must never both be open on the same physical frontend
  device node at once, and starting a real stream always wins.
- **`tuner_release()`** re-opens a hold afterward (using the tuner's own
  remembered `tuned_frequency_hz`/`tuned_delivery`) if a channel is
  still logically selected and nothing already re-established one in
  the meantime.

This means `udp_stream.c` and `http_server.c` needed **zero changes** —
they already call `tuner_try_claim()` before streaming and
`tuner_release()` after, so the hold transparently steps out of the
way and comes back on its own. The only call sites that needed direct
changes are the two places a channel actually gets *selected*:
`vchannel` SET calls `tuner_open_hold()` synchronously right after
`tuner_bind_channel()` (cheap and non-blocking, so no background
thread needed); `channel` SET's existing background scan thread
(`channel_scan_thread_main`, §4) calls it once after its whole
pending-queue drains, using whichever frequency it processed last —
deliberately *not* per-frequency inside that loop, since a fast
multi-channel scan (`hdhomerun_config scan`) can process dozens of
queued frequencies in a couple of seconds, and opening+closing a hold
for each one that's about to be immediately superseded would be pure
waste. `channelmap` SET and an explicit `channel=none` both call
`tuner_close_hold()` directly, since either invalidates whatever was
held.

## 17. Debugging techniques that worked, worth reusing

- **`/proc/<pid>/task/*/stat` sampling** (`ps -eLo tid,stat,wchan,comm`,
  ~100ms interval, piped to a log file, run *concurrently* with a failing
  client command) is what actually proved the driver's block is
  uninterruptible-sleep and confirmed when a background worker thread has
  truly finished vs. just between log lines. Log-line-count "stabilizing"
  is not a reliable proxy for a background worker being done — long gaps
  between `dvb_scan: found` lines are normal when several consecutive
  frequencies fail to lock quickly, since only successes get logged.
- **`gdb -p <tid> -batch -ex bt`** on a live, seemingly-stuck thread
  pinpoints exactly which ioctl (and which calling function) is actually
  blocked, rather than guessing from symptoms.
- **Reading the real client source directly** beats reverse-engineering
  its behavior from `tcpdump` captures alone — a full copy of
  `libhdhomerun` + `hdhomerun_config(_gui)` source lives on the
  development Pi (see the project's memory notes for the exact paths);
  several bugs in this codebase were only fully understood once the
  actual client-side polling/timeout logic was read, not just observed.
- **`tcpdump` filtering on a shared host**: a bare `port 65001` filter on
  the `any` interface captures *all* control-port traffic touching the
  host, including unrelated local scripts acting as their own clients to
  *other* devices on the LAN. Filter to `dst <this-daemon's-own-IP> and
  port 65001` (found via `hdhomerun_config discover`, since a daemon can
  be reachable at more than one address) to isolate genuine inbound
  requests — and check `ps aux`/`ss -tnp` for other local processes before
  trusting a capture's contents as caused by whatever you just did in a
  client UI.
