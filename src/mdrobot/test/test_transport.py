"""transport.py unit tests: SerialTransport (with a fake serial port injected).

No real pyserial port is opened; a fake is injected via SerialTransport.from_serial.
Read accumulation / frame assembly is verified together with ModbusClient.
"""

import time

import pytest

from mdrobot.crc import append_crc
from mdrobot.protocol import ModbusClient
from mdrobot.transport import SerialTransport, Transport, interframe_delay


class FakeSerial:
    """Minimal pyserial-compatible fake port to inject into SerialTransport.from_serial."""

    def __init__(self, to_read: bytes = b"", *, flush_clears: bool = True) -> None:
        self.port = "fake"
        self.baudrate = 19200
        self.is_open = True
        self.written = bytearray()
        self.flush_calls = 0
        self.reset_calls = 0
        self._rx = bytearray(to_read)
        self._flush_clears = flush_clears

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def flush(self) -> None:
        self.flush_calls += 1

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def reset_input_buffer(self) -> None:
        self.reset_calls += 1
        if self._flush_clears:
            self._rx.clear()

    def close(self) -> None:
        self.is_open = False


def test_satisfies_transport_protocol():
    fake = FakeSerial()
    transport = SerialTransport.from_serial(fake)
    assert isinstance(transport, Transport)


def test_write_returns_count_and_flushes():
    fake = FakeSerial()
    transport = SerialTransport.from_serial(fake)
    n = transport.write(b"\x01\x02\x03")
    assert n == 3
    assert fake.written == b"\x01\x02\x03"
    assert fake.flush_calls == 1  # wait for RS485 transmission to complete


def test_read_pulls_up_to_size():
    fake = FakeSerial(to_read=b"\xaa\xbb\xcc", flush_clears=False)
    transport = SerialTransport.from_serial(fake)
    assert transport.read(2) == b"\xaa\xbb"
    assert transport.read(2) == b"\xcc"
    assert transport.read(2) == b""


def test_flush_input_resets_buffer():
    fake = FakeSerial(to_read=b"\x01\x02")
    transport = SerialTransport.from_serial(fake)
    transport.flush_input()
    assert fake.reset_calls == 1
    assert transport.read(2) == b""  # flush cleared the buffer


def test_close_and_is_open():
    fake = FakeSerial()
    transport = SerialTransport.from_serial(fake)
    assert transport.is_open is True
    transport.close()
    assert transport.is_open is False


def test_context_manager_closes():
    fake = FakeSerial()
    with SerialTransport.from_serial(fake) as transport:
        assert transport.is_open is True
    assert fake.is_open is False


def test_modbus_client_read_over_fake_serial():
    """SerialTransport + ModbusClient integration: read_register decodes the response word."""
    # Read 1 word of PID_VERSION(1); response value 0x000D.
    response = append_crc(bytes((1, 0x03, 2, 0x00, 0x0D)))
    fake = FakeSerial(to_read=response, flush_clears=False)
    client = ModbusClient(SerialTransport.from_serial(fake), slave_id=1)

    value = client.read_register(1)
    assert value == 0x000D
    # Verify the request frame actually went out on the wire.
    expected_request = append_crc(bytes((1, 0x03, 0x00, 0x01, 0x00, 0x01)))
    assert bytes(fake.written) == expected_request


def test_modbus_client_short_read_raises():
    """A short response raises IncompleteResponseError."""
    from mdrobot.exceptions import IncompleteResponseError

    fake = FakeSerial(to_read=b"\x01\x03", flush_clears=False)  # header only, no body
    client = ModbusClient(SerialTransport.from_serial(fake), slave_id=1)
    with pytest.raises(IncompleteResponseError):
        client.read_register(1)


# --- Modbus RTU inter-frame gap -------------------------------------------
# Mirrors test_transport.cpp — keep both in step (CLAUDE.md mirroring rule).


class TestInterframeDelay:
    """The pure computation: 3.5 character times, pinned above 19200 baud."""

    @pytest.mark.parametrize("baud", [38400, 57600, 115200])
    def test_pinned_above_19200(self, baud):
        assert interframe_delay(baud) == 0.00175

    @pytest.mark.parametrize("baud", [19200, 9600, 4800])
    def test_three_and_a_half_characters_at_or_below(self, baud):
        # 3.5 characters x 11 bits = 38.5 bit times.
        assert interframe_delay(baud) == 38.5 / baud

    def test_default_rig_baud_is_two_milliseconds(self):
        # The rig runs 19200 8N1; this is the gap every transaction actually pays.
        assert interframe_delay(19200) == pytest.approx(0.002005, abs=1e-6)

    @pytest.mark.parametrize("baud", [None, 0, -1])
    def test_bogus_baud_falls_back_to_the_pinned_gap(self, baud):
        # Never hand _wait_interframe a zero or negative sleep.
        assert interframe_delay(baud) == 0.00175

    def test_shrinks_monotonically_with_baud(self):
        assert interframe_delay(4800) > interframe_delay(9600)
        assert interframe_delay(9600) > interframe_delay(19200)
        assert interframe_delay(19200) > interframe_delay(38400)


class TestWriteHoldsOff:
    """write() must not start a frame until the bus has been silent long enough."""

    def test_first_write_does_not_wait(self):
        # Nothing has touched the bus yet, so there is no gap to honour.
        t = SerialTransport.from_serial(FakeSerial())
        start = time.monotonic()
        t.write(b"\x01\x03")
        assert time.monotonic() - start < 0.001

    def test_second_write_waits_out_the_gap(self):
        t = SerialTransport.from_serial(FakeSerial())
        t.write(b"\x01\x03")
        start = time.monotonic()
        t.write(b"\x01\x03")
        # 19200 baud -> 2.005 ms. Allow scheduler slop but require most of it.
        assert time.monotonic() - start >= 0.0018

    def test_read_also_counts_as_bus_activity(self):
        # A reply occupies the bus too; the next request must clear it as well.
        t = SerialTransport.from_serial(FakeSerial(b"\xff\xff"))
        t.read(2)
        start = time.monotonic()
        t.write(b"\x01\x03")
        assert time.monotonic() - start >= 0.0018

    def test_gap_already_elapsed_costs_nothing(self):
        t = SerialTransport.from_serial(FakeSerial())
        t.write(b"\x01\x03")
        time.sleep(0.005)                       # longer than the 2.005 ms gap
        start = time.monotonic()
        t.write(b"\x01\x03")
        assert time.monotonic() - start < 0.001
