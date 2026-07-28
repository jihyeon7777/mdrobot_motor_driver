# Field report — `USE_LIMIT_SW=1` gives you the CTRL stop input but blocks reverse

**Status:** open. Characterised on hardware 2026-07-29; no fix applied.
**Affects:** `mdrobot` / `mdrobot_cpp` `enable()` + `set_velocity()`, and
`mdrobot_ros2_control` (`use_limit_sw` parameter).
**Impact:** on the units tested you can have the hardware stop input **or**
bidirectional velocity control, but not both. That makes a differential base with
`reverse_L: true` unable to drive at all with the stop input enabled.

Hardware: 2 × MD400 (`PID_VERSION` = 86), 19200 8N1, twin mode on one RS-485 bus at
slave ids 1 and 2, Raspberry Pi 5 / ROS 2 Jazzy.

---

## 1. What the operator wanted

An emergency stop that disables motor rotation through a **driver pin**, not by
cutting driver power — so the controller stays on the bus and the software can see
that the stop happened.

`docs/manual/README.md` already documents the way to do this: set
`USE_LIMIT_SW (17) = 1` and wire a switch to **CTRL pin 8 (START/STOP)**. The
manual is right about the pin. The problem is what else that register turns on.

## 2. What works — the stop input itself

The switch was found already wired and correct. Confirmed read-only by polling
`PID_DI (48)` while the operator pressed it, 4 cycles:

| state | `DI` | bits (`status.py:36` map) |
|---|---|---|
| released | `112` = `0b0111_0000` | `START_STOP`(b4), `ENC_B`(b5), `ENC_A`(b6) |
| pressed | `96` = `0b0110_0000` | b4 drops |

- Only b4 moves; `DIR`(b2) and `RUN_BRAKE`(b3) stay low throughout.
- **Both controllers see the transition in the same poll** — one switch, both units.
- Polarity is fail-safe: closed = run permitted, open = stop, so a broken wire
  stops the machine.

With `USE_LIMIT_SW=1`, opening the switch stopped a running motor from
**156 rpm to 0 within 0.3 s**, and it **stayed** stopped after the switch closed
again — the run latch is released, so `enable()` must be called to resume. That
latching behaviour is desirable and worth keeping.

So the stop path is good. Everything below is about the side effect.

## 3. The problem — `USE_LIMIT_SW=1` blocks negative `VEL_CMD`

With `USE_LIMIT_SW=1`, both controllers only turn for a **positive** `PID_VEL_CMD`.
Short low-speed bursts, `±156` rpm:

| controller | command | measured peak | |
|---|---|---|---|
| id1 | `+156` | `+158` rpm | runs |
| id1 | `−156` | `0` rpm | **does not run** |
| id2 | `+156` | `+158` rpm | runs |
| id2 | `−156` | `0` rpm | **does not run** |

The control case is decisive: with `USE_LIMIT_SW=0` the *same* left controller runs
happily on negative commands. In the staircase run it was commanded
`−156 … −1555` rpm at the wire (`reverse_L: true` negates every command) and
tracked to within 0.1 % at each step. The published `joint_states` values read
positive because `publish_joint_state` applies the same `direction` to feedback
that it applies to commands — the raw `VEL_CMD` and `speed_rpm` are both negative.
So negative commands are fine; enabling the stop input is what breaks them.

### Consequence for the ros2_control driver

`twin_controllers.yaml` sets `reverse_L: true`, so `motor_L` is always commanded
negative. With `use_limit_sw: 1` the observed result is:

| function | result |
|---|---|
| left wheel (negative command) | never turns |
| driving in reverse | impossible |
| spin-in-place (wheels opposed) | impossible |

The first bringup attempt with `USE_LIMIT_SW=1` showed exactly this: `motor_R`
spun up to 156 rpm while `motor_L` sat at 0 for the whole run.

## 4. Ruled out — the run-latch direction

`registers.py:28` documents `PID_START_STOP` as `0 stop, 1 CCW, 2 CW`, and both
`device.py:42` and `device.cpp:18` hardcode `START = 1`. The obvious theory was that
the latch direction outranks the sign of `VEL_CMD` once the CTRL inputs are
honoured, so arming with `2` would unlock reverse.

**It does not.** Full matrix on id1 at `USE_LIMIT_SW=1`:

| `START_STOP` | `VEL_CMD` | measured |
|---|---|---|
| 1 (CCW) | `+156` | `+158` rpm |
| 2 (CW) | `−156` | `0` |
| 2 (CW) | `+156` | `+158` rpm |
| 1 (CCW) | `−156` | `0` |

The latch value has no effect on anything — not on whether the motor runs, not on
which way it turns (always `+158`). Only the sign of `VEL_CMD` matters. Whatever
`START_STOP`'s `1 = CCW / 2 = CW` means on this firmware, it is not a direction
select for serial velocity drive.

## 5. Working hypothesis

With `USE_LIMIT_SW=1` the controller starts honouring the CTRL connector's
direction input (`DIR`, `DI` b2), which is **hard low** in this installation
because only pin 8 is wired. Low appears to mean "forward only", so the negative
command is refused rather than inverted.

Two independent ways of asking for reverse are refused **identically** — a negative
`VEL_CMD`, and `INV_SIGN_CMD=1` with a positive `VEL_CMD` (§6.1) — while forward
works in both cases. That is what a direction gate downstream of the command path
looks like, and it is the strongest evidence available without instrumenting the
pin.

Still **not proven**: it predicts that driving `DIR` high would make the motor run
for negative commands, and that has not been tested — the pin is not wired to
anything that can drive it.

## 6. Candidate fixes

### 6.1 `PID_INV_SIGN_CMD (16)` — tested, does **not** work

`registers.py:20` lists `PID_INV_SIGN_CMD = 16  # R/W reference command sign
inverse`, which looked like a way to obtain reverse while keeping `VEL_CMD`
positive — the only command shape that still runs at `USE_LIMIT_SW=1`.

Measured on id1, 156 rpm bursts, physical direction taken from the change in the
position accumulator rather than the reported speed sign:

| `USE_LIMIT_SW` | `INV_SIGN_CMD` | `VEL_CMD` | Δposition | shaft |
|---|---|---|---|---|
| 0 | 0 | `+156` | `+307` | forward |
| 0 | **1** | `+156` | `+306` | **forward — unchanged** |
| 0 | 0 | `−156` | `−306` | reverse |
| 1 | 0 | `+156` | `+307` | forward |
| 1 | **1** | `+156` | `0` | **does not turn** |
| 1 | 0 | `−156` | `0` | does not turn |

Two conclusions:

1. `INV_SIGN_CMD` has **no effect on `VEL_CMD`** at all — `+307` vs `+306` counts is
   the same motion. It presumably inverts some other reference source (analog or
   CTRL-connector), not the serial velocity command.
2. With `USE_LIMIT_SW=1` it makes things *worse*: setting it blocks the positive
   command that otherwise works. Requesting reverse by flag is refused exactly like
   requesting it by sign.

So this is not a fix, and it should not be tried again. Its one useful contribution
is the evidence for §5.

### 6.2 `PID_INPUT_TYPE (25)` — the remaining pure-software candidate

Reads `0` on both units and is documented only as "user input type". If it selects
*which* CTRL inputs are honoured, there may be a setting that takes the stop input
without the direction input. Worth a look in the vendor protocol map — with §6.1
ruled out, this is the only candidate left that needs no wiring change.

### 6.3 Wire `DIR` per controller

Drive each controller's `DIR` pin from a Pi GPIO. Restores both capabilities but
moves direction control out of the serial path, which means the hardware interface
has to sequence a GPIO write against a `VEL_CMD` write on every sign change — extra
latency and a new failure mode. Only worth it if 6.1 and 6.2 both fail.

### 6.4 Software stop instead of hardware stop

Keep `USE_LIMIT_SW=0` and have the driver poll `PID_DI` b4 in `read()`, commanding
zero and `disable()` when it opens. No wiring change — the switch is already on both
controllers. Costs one extra read per controller per cycle (~19 ms each at 19200,
which is real budget at 10 Hz on twin) and the stop is only as reliable as the
software and the bus. **Not a safety-rated stop**; it is a functional one.

## 7. Suggested API shape, whichever mechanism wins

The current `enable()` writes a fixed `UI_COM=1, COM_TAR_SPEED=0, START_STOP=1`
triple and `set_velocity()` writes a signed word. Neither has a place to express
"this controller's reverse is gated". Whatever the fix turns out to be, it would
help to:

- have `enable()` (or `on_configure`) **verify** that a negative command is
  actually accepted, and fail loudly if it is not, rather than leaving one wheel
  silently dead — the failure mode that cost the most time here was a wheel that
  reported `0 rpm` with no alarm and no error;
- expose the stop-input state (it is already in `PID_DI`) as a first-class read,
  so a `ros2_control` layer can surface "stopped by hardware" instead of
  interpreting a stalled wheel;
- document in `docs/manual/README.md` that `USE_LIMIT_SW = 1` is not free — the
  current text presents it as simply "add a hardware stop switch".

## 8. Reproducing

All of it is low-speed and read-mostly; the only configuration write is
`USE_LIMIT_SW`, which is restored afterwards.

1. Read the state: `USE_LIMIT_SW(17)`, `UI_COM(78)`, `INPUT_TYPE(25)`, `DI(48)`,
   `CTRL_STATUS(34)`, `INV_SIGN_CMD(16)` on each slave id.
2. Poll `DI(48)` while pressing the switch to confirm which bit moves.
3. Write `USE_LIMIT_SW=1`, `enable()`, then drive `+156` and `−156` rpm in turn and
   record `read_monitor().speed_rpm`.
4. Repeat step 3 with `START_STOP` armed as `2` to confirm the latch is irrelevant.
5. Repeat step 3 with `INV_SIGN_CMD=1` and a positive command, at
   `USE_LIMIT_SW` 0 then 1, reading `read_monitor().position` before and after each
   burst — the position delta is the only trustworthy direction indicator.
6. Restore `USE_LIMIT_SW=0` (and `INV_SIGN_CMD=0`) and confirm both directions work
   again.

Baseline for comparison: with `USE_LIMIT_SW=0` both units track a
156 → 1555 rpm staircase in both signs to within 0.1 %, drawing 0.80 mA/rpm of bus
current with no load (see the current-test logs referenced with this report).
