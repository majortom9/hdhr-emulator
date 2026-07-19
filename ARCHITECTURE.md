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
  one waiter.

Every field on `struct hdhr_tuner` that isn't set-once-at-init is guarded
by its own `pthread_mutex_t lock` (`tuner_lock()`/`tuner_unlock()`) — not a
global lock, so different tuner slots never contend with each other.

## 4. `/tunerN/channel` SET — why it's the most complicated endpoint

This is the endpoint that took the most iteration, because it's the one
place where a real hardware quirk collided with client-side timing
assumptions baked into `libhdhomerun` itself. Worth reading in full before
touching `control.c`'s channel-SET code.

### The hardware constraint

This project's `lgdt3306a` demodulator driver retries a lock internally
**inside a single ioctl call**, several times, before giving up on a dead
frequency — confirmed via `dmesg` (`lgdt3306a_search: loop=0..4`) and via
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
   demux PID filters (either specific program/PCR/video/audio PIDs, or the
   `0x2000` full-mux wildcard PID for `program=0`), and spawns a reader
   thread that pulls from `/dev/dvb/adapterN/dvrM` into an internal ring
   buffer.
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

## 9. Debugging techniques that worked, worth reusing

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
