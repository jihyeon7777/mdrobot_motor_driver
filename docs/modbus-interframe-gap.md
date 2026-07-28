# Bug report — missing Modbus RTU inter-frame gap in `SerialTransport`

**Status:** fixed downstream on 2026-07-28; patch below is ready to apply upstream.
**Affects:** `mdrobot` (Python) and `mdrobot_cpp` (C++) — both transports.
**Severity:** high. The library only communicates reliably by accident on FTDI
adapters left at their default latency timer. On a correctly-tuned adapter, or on
any adapter that delivers bytes promptly, roughly **half of all transactions time
out**.

---

## 1. Summary

`SerialTransport.write()` sends a request frame immediately, with no enforced
silence since the previous frame. Modbus RTU requires **at least 3.5 character
times of bus silence** to delimit frames; without it a slave cannot tell where one
frame ends and the next begins, and silently drops the request.

The bug was masked because the FTDI `ftdi_sio` driver defaults `latency_timer` to
**16 ms**, which inadvertently supplied a gap far larger than the 1.75–2 ms the
spec requires. Lowering `latency_timer` to 1 ms — the standard fix for USB-serial
latency — removes the accidental gap and exposes the defect.

## 2. Symptom

On a twin bringup (`mdrobot_ros2_control`, two MD400 at slave ids 1 and 2,
19200 8N1), `on_configure` succeeds in pinging both controllers and then fails on
the next transaction:

```
opened /dev/ttyUSB0 slave_id=1: version=86 voltage=27.7V
opened /dev/ttyUSB0 slave_id=2: version=86 voltage=28.3V
[ERROR] on_configure failed: short read: got 0 want 2:
[ERROR] Failed to 'configure' hardware 'mdrobot_twin'
terminate called after throwing an instance of 'std::runtime_error'
  what():  Failed to set the initial state of the component : mdrobot_twin to active
```

The failing call is the `PID_USE_LIMIT_SW` write in `mdrobot_system.cpp`
(`write_register`, function 0x06). `got 0 want 2` is a **complete** timeout — zero
bytes in 300 ms — not a partial or corrupted frame.

Reproduced 3/3 times. Isolated reads (one transaction, then the port closed) always
succeed, which is why the problem looks intermittent rather than systematic.

## 3. Root cause

`SerialTransport.write()` (both languages) writes as soon as it is called. The
protocol layer issues `flush_input()` → `write()` → `read()` back to back, so when
one transaction's response has just been read, the next request goes out on the
wire microseconds later. The slave is still inside its own inter-frame timeout and
treats the new request bytes as a continuation of the previous frame, so the CRC
fails and it never replies.

## 4. Why it went unnoticed

`ftdi_sio` defaults `latency_timer` to 16 ms. That timer governs how long the chip
buffers data before shipping it upstream, so every host-side read is quantised to a
16 ms boundary. The resulting dead time between transactions is ~8× the required
gap, and it silently satisfied the protocol requirement.

Any of the following removes the accidental gap and breaks communication:

- setting `latency_timer=1` (a common and otherwise correct performance fix);
- a different USB-serial chip with a smaller or absent latency timer;
- a native UART (e.g. a Raspberry Pi GPIO UART via an RS-485 transceiver), where
  there is no USB buffering at all.

The third case matters: the library would fail out of the box on a GPIO-UART setup.

## 5. Evidence

All measurements on a Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy), FTDI FT232R,
two MD400 controllers (`version=86`) at slave ids 1 and 2, 19200 8N1.

### 5.1 The gap is the variable

Alternating reads between slave id 1 and id 2, 20 transactions per row, varying only
the delay inserted before each request. Read-only (function 0x03), `latency_timer=1`:

| inserted gap | failures | median round-trip |
|---|---|---|
| **0.0 ms** | **10 / 20** | 304.1 ms (timeouts) |
| 1.0 ms | 0 / 20 | 16.9 ms |
| 2.0 ms | 0 / 20 | 16.9 ms |
| 3.5 ms | 0 / 20 | 16.4 ms |
| 5.0 ms | 0 / 20 | 16.9 ms |
| 10.0 ms | 0 / 20 | 16.9 ms |

Exactly half fail at zero gap — every transaction that immediately follows a
completed response. Anything ≥ 1 ms is clean. Nothing else changed between rows.

### 5.2 What the latency timer was hiding

Round-trip time for a single `read_monitor`, 10 samples per row:

| `latency_timer` | median | min | max | theoretical wire time |
|---|---|---|---|---|
| 16 ms (default) | 32.0 ms | 31.8 | 32.1 | 13.0 ms |
| 1 ms | 17.0 ms | 16.9 | 17.0 | 13.0 ms |

The 32.0 ms figure with ±0.15 ms spread is quantisation to 2 × 16 ms, not wire
time — the tell-tale signature of the latency timer dominating the transaction.

### 5.3 Downstream cost of the workaround

Leaving `latency_timer` at 16 ms keeps the library working but makes a twin cycle
(4 transactions: 2 × `read_monitor` + 2 × velocity write) ≈ 128 ms, which overruns
the 100 ms budget at `update_rate: 10`:

```
Overrun detected! The controller manager missed its desired rate of 10 Hz.
The loop took 368.55 ms (missed cycles : 4)
   Read time : 325.1 ms,  Write time : 39.2 ms
```

So before this fix there was no good configuration: `latency_timer=16` overruns the
control loop, `latency_timer=1` breaks communication.

## 6. Fix

Track the timestamp of the last bus activity in the transport, and make `write()`
wait out the remaining inter-frame gap before starting a new frame. The gap is
3.5 character times at 11 bits per character, which the Modbus RTU spec fixes at
1.75 ms for baud rates above 19200:

```
gap = baudrate > 19200 ? 1.75 ms : 38.5 / baudrate
```

At 19200 that is 2.0 ms; at 9600, 4.0 ms.

The transport is the right layer: it is the only place that knows the baud rate and
sees every byte in both directions, so the accounting stays correct no matter how
the protocol layer chunks its reads.

### 6.1 Python — `src/mdrobot/mdrobot/transport.py`

```diff
 from __future__ import annotations
 
+import time
 from typing import Any, Protocol, runtime_checkable
 
 from .constants import DEFAULT_BAUDRATE, DEFAULT_TIMEOUT
 
 
+def interframe_delay(baudrate: int | None) -> float:
+    """Modbus RTU inter-frame silence: 3.5 character times (11 bits each).
+
+    The spec fixes the gap at 1.75 ms above 19200 baud. Without it the slave
+    never sees the frame boundary and silently drops the request. This stays
+    hidden on adapters whose USB latency timer supplies the gap by accident
+    (FTDI defaults to 16 ms), so it only surfaces once that timer is lowered.
+    """
+    if not baudrate or baudrate > 19200:
+        return 0.00175
+    return 38.5 / baudrate
+
+
 @runtime_checkable
 class Transport(Protocol):
```

```diff
     ) -> None:
-        import time
-
         import serial  # lazy import: pyserial is an optional dependency
 
         self.port = port
         self.baudrate = baudrate
+        self._interframe = interframe_delay(baudrate)
+        self._last_activity = 0.0
```

```diff
         obj = cls.__new__(cls)
         obj.port = getattr(serial_port, "port", None)
         obj.baudrate = getattr(serial_port, "baudrate", None)
+        obj._interframe = interframe_delay(obj.baudrate)
+        obj._last_activity = 0.0
         obj._serial = serial_port
         return obj
 
+    def _wait_interframe(self) -> None:
+        """Hold off until the bus has been silent for 3.5 character times."""
+        if not self._last_activity:
+            return
+        remaining = self._interframe - (time.monotonic() - self._last_activity)
+        if remaining > 0:
+            time.sleep(remaining)
+
     def write(self, data: bytes) -> int:
-        """Send data, wait for transmission to complete, and return bytes written."""
+        """Send data, wait for transmission to complete, and return bytes written.
+
+        Waits out the Modbus inter-frame gap first: this call starts a new frame,
+        and the slave only recognises it after enough silence on the bus.
+        """
+        self._wait_interframe()
         written = self._serial.write(data)
         self._serial.flush()
+        self._last_activity = time.monotonic()
         return written if written is not None else len(data)
 
     def read(self, size: int) -> bytes:
         """Read up to size bytes; returns fewer (or empty) on timeout."""
-        return self._serial.read(size)
+        data = self._serial.read(size)
+        self._last_activity = time.monotonic()
+        return data
```

Note `import time` moves to module scope — it was function-local in `__init__`, but
`_wait_interframe` and `read` now need it too.

`from_serial()` bypasses `__init__`, so it must initialise both new attributes.
`interframe_delay` accepts `None` (fakes and wrapped objects may not expose a
baudrate) and falls back to the 1.75 ms floor.

### 6.2 C++ — `src/mdrobot_cpp/include/mdrobot_cpp/transport.hpp`

```diff
 #pragma once
 
+#include <chrono>
 #include <cstdint>
 #include <memory>
 #include <string>
 #include <vector>
```

```diff
  private:
+  /// Hold off until the bus has been silent for 3.5 character times.
+  void wait_interframe();
+
   std::string port_;
   int baudrate_;
   double write_timeout_;
   int fd_ = -1;
+  std::chrono::steady_clock::duration interframe_{};
+  std::chrono::steady_clock::time_point last_activity_{};
 };
```

### 6.3 C++ — `src/mdrobot_cpp/src/transport.cpp`

In the constructor, after `tcsetattr` succeeds and before the settle sleep:

```diff
+  // Modbus RTU inter-frame silence: 3.5 character times (11 bits each); the
+  // spec fixes it at 1.75 ms above 19200 baud. Without it the slave never sees
+  // the frame boundary and silently drops the request. This stays hidden on
+  // adapters whose USB latency timer supplies the gap by accident (FTDI
+  // defaults to 16 ms), so it only surfaces once that timer is lowered.
+  const double gap = (baudrate > 19200) ? 0.00175 : 38.5 / baudrate;
+  interframe_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
+      std::chrono::duration<double>(gap));
+
   // Settle + flush (same as Python: USB-serial boot noise mitigation).
   if (settle > 0) {
```

```diff
+void SerialTransport::wait_interframe() {
+  if (last_activity_.time_since_epoch().count() == 0) return;  // first frame
+  const auto ready = last_activity_ + interframe_;
+  const auto now = std::chrono::steady_clock::now();
+  if (now < ready) std::this_thread::sleep_for(ready - now);
+}
+
 std::size_t SerialTransport::write(const uint8_t* data, std::size_t len) {
   if (fd_ < 0) throw std::runtime_error("Port not open");
+  // This call starts a new frame: the slave only recognises it after enough
+  // silence on the bus, so wait out the inter-frame gap first.
+  wait_interframe();
```

```diff
   ::tcdrain(fd_);  // wait for transmission to complete (like pyserial flush)
+  last_activity_ = std::chrono::steady_clock::now();
   return total;
 }
```

```diff
   buf.resize(static_cast<std::size_t>(n));
+  last_activity_ = std::chrono::steady_clock::now();
   return buf;
 }
```

The `EAGAIN`/`EWOULDBLOCK` early return in `read()` is deliberately **not** stamped —
nothing arrived, so it is not bus activity.

## 7. Verification

- **Unit tests:** 229 tests across `mdrobot` and `mdrobot_cpp`, 0 failures. The
  existing `FakeSerial`-based tests in `test/test_transport.py` pass unchanged; they
  report `baudrate = 19200`, so each write now costs ~2 ms of real sleep.
- **Hardware, bringup:** 3/3 successful `ros2 launch mdrobot_ros2_control
  bringup.launch.py device_type:=twin` at `latency_timer=1` (was 0/3 before).
- **Hardware, sustained:** 45 s continuous run at `update_rate: 10` (~450 cycles) —
  **0 overruns, 0 communication errors**, both controllers active, `/joint_states`
  publishing for `motor_L` / `motor_R`.
- **Cycle budget:** transaction 32 ms → 17 ms; twin cycle 128 ms → ~76 ms, which
  lands inside the 68–78 ms the `twin_controllers.yaml` comment originally
  estimated.

## 8. Notes for upstream

1. **Cost.** The gap adds ~2 ms per transaction at 19200. A twin cycle pays ~8 ms.
   This is unavoidable — it is a protocol requirement, not overhead — and it is far
   cheaper than the 16 ms/read the default latency timer was charging.

2. **The C++ `read()` is unusually latency-sensitive.** It is configured `VMIN=0`,
   `VTIME=timeout*10`, so `::read()` returns as soon as *any* byte is available, and
   the protocol layer loops to assemble the frame. Every iteration can pay a full
   latency-timer interval. This is consistent with the 325 ms read time observed at
   `latency_timer=16` (≈10 chunks × 16 ms + wire time), against the 32 ms that the
   Python path — where `pyserial.read(size)` blocks until `size` bytes — measures for
   the same transaction. Consider having the C++ transport loop internally until
   `size` bytes or the deadline, matching Python's semantics; it would make the C++
   path much less sensitive to adapter tuning. *(Inferred from timings; the chunk
   sizes were not directly instrumented.)*

3. **Independent of `settle`.** The existing `settle` parameter is a one-shot
   post-open delay for USB boot noise. It does not and cannot address the per-frame
   gap.

4. **Consider documenting the adapter requirement.** Even with this fix, FTDI's
   default `latency_timer=16` costs ~15 ms per transaction, which is what pushes a
   twin bringup past its 10 Hz budget. A note in `docs/manual/ros2_control.md`
   recommending a udev rule would save the next integrator the same investigation:

   ```
   ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
   ```

5. **Suggested regression test.** Assert that two consecutive `write()` calls on a
   transport constructed at 19200 are separated by at least 2 ms, using a fake serial
   port and a monotonic clock. This pins the behaviour without needing hardware.
