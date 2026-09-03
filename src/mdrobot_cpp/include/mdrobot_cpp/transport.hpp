// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file transport.hpp
/// Transport interface and POSIX termios serial implementation.

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mdrobot {

/// Modbus RTU inter-frame silence in seconds: 3.5 character times (11 bits each).
///
/// The spec fixes the gap at 1.75 ms above 19200 baud. Mirrors
/// `mdrobot.transport.interframe_delay` in the Python library — keep both in
/// step (see CLAUDE.md on the Python/C++ mirroring rule).
double interframe_delay(int baudrate);

/// Abstract transport interface — the protocol layer depends on this.
class Transport {
 public:
  virtual ~Transport() = default;

  /// Send all of data and return the number of bytes written.
  virtual std::size_t write(const uint8_t* data, std::size_t len) = 0;

  /// Read up to @p size bytes; may return fewer.
  virtual std::vector<uint8_t> read(std::size_t size) = 0;

  /// Discard any bytes left in the input buffer.
  virtual void flush_input() = 0;
};

/// POSIX termios serial transport for RS485 / Modbus RTU.
class SerialTransport : public Transport {
 public:
  /// Open a serial port. Throws on failure.
  SerialTransport(const std::string& port, int baudrate = 19200,
                  double timeout = 0.3, double settle = 0.2,
                  double write_timeout = 1.0);

  ~SerialTransport() override;

  // Non-copyable.
  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;

  std::size_t write(const uint8_t* data, std::size_t len) override;
  std::vector<uint8_t> read(std::size_t size) override;
  void flush_input() override;

  void close();
  bool is_open() const;

  const std::string& port() const { return port_; }
  int baudrate() const { return baudrate_; }

 private:
  /// Hold off until the bus has been silent for 3.5 character times.
  void wait_interframe();

  std::string port_;
  int baudrate_;
  double write_timeout_;
  int fd_ = -1;
  std::chrono::steady_clock::duration interframe_{};
  std::chrono::steady_clock::time_point last_activity_{};
};

}  // namespace mdrobot
