// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file test_transport.cpp
/// Modbus RTU inter-frame gap. Mirrors test_transport.py — keep both in step.
///
/// Only the pure computation is covered here. SerialTransport itself needs a
/// real termios fd, so the hold-off behaviour is exercised on the Python side
/// (a fake port can be injected there) and by the measurements in
/// docs/modbus-interframe-gap.md.

#include <gtest/gtest.h>

#include "mdrobot_cpp/transport.hpp"

namespace {

TEST(InterframeDelay, PinnedAboveNineteenTwoHundred) {
  // The spec stops shrinking the gap above 19200 baud.
  EXPECT_DOUBLE_EQ(mdrobot::interframe_delay(38400), 0.00175);
  EXPECT_DOUBLE_EQ(mdrobot::interframe_delay(57600), 0.00175);
  EXPECT_DOUBLE_EQ(mdrobot::interframe_delay(115200), 0.00175);
}

TEST(InterframeDelay, ThreeAndAHalfCharactersAtOrBelow) {
  // 3.5 characters x 11 bits = 38.5 bit times.
  EXPECT_DOUBLE_EQ(mdrobot::interframe_delay(19200), 38.5 / 19200);
  EXPECT_DOUBLE_EQ(mdrobot::interframe_delay(9600), 38.5 / 9600);
  EXPECT_DOUBLE_EQ(mdrobot::interframe_delay(4800), 38.5 / 4800);
}

TEST(InterframeDelay, DefaultRigBaudIsTwoMilliseconds) {
  // The rig runs 19200 8N1; this is the gap every transaction actually pays.
  EXPECT_NEAR(mdrobot::interframe_delay(19200), 0.002005, 1e-6);
}

TEST(InterframeDelay, NonPositiveBaudFallsBackToThePinnedGap) {
  // Never return zero or a negative sleep for a bogus baudrate.
  EXPECT_DOUBLE_EQ(mdrobot::interframe_delay(0), 0.00175);
  EXPECT_DOUBLE_EQ(mdrobot::interframe_delay(-1), 0.00175);
}

TEST(InterframeDelay, ShrinksMonotonicallyWithBaud) {
  EXPECT_GT(mdrobot::interframe_delay(4800), mdrobot::interframe_delay(9600));
  EXPECT_GT(mdrobot::interframe_delay(9600), mdrobot::interframe_delay(19200));
  EXPECT_GT(mdrobot::interframe_delay(19200), mdrobot::interframe_delay(38400));
}

}  // namespace
