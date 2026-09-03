// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file units.hpp
/// Physical unit conversion helpers — count/rpm ↔ rad / (rad/s).

#pragma once

#include <cmath>
#include <cstdint>

namespace mdrobot {

constexpr double TWO_PI = 2.0 * M_PI;
// --- slow-start / slow-down conversion (NOT yet hardware-verified) ---
// The seconds<->raw map itself has never been checked against a stopwatch; only the
// register addresses are verified (registers.hpp). The real full scale is set by
// PID_MAX_SS_TIME on the controller (15-60 s, default 15). Mirrors units.py.
constexpr int SLOW_RAW_MAX = 1023;
constexpr double SLOW_DEFAULT_FULL_SCALE_S = 15.0;

/// Convert position count to motor-shaft angle (rad).
double counts_to_rad(double count, double counts_per_rev);

/// Convert motor-shaft angle (rad) to nearest position count.
int rad_to_counts(double rad, double counts_per_rev);

/// Convert mechanical rpm to rad/s.
inline double rpm_to_rad_s(double rpm) { return rpm * TWO_PI / 60.0; }

/// Convert rad/s to mechanical rpm.
inline double rad_s_to_rpm(double rad_s) { return rad_s * 60.0 / TWO_PI; }

/// Convert slow time (seconds) to raw 0-1023. Clamped. NOT hardware-verified.
int slow_seconds_to_raw(double seconds, double full_scale_s = SLOW_DEFAULT_FULL_SCALE_S);

/// Convert raw 0-1023 to seconds. NOT hardware-verified.
double slow_raw_to_seconds(int raw, double full_scale_s = SLOW_DEFAULT_FULL_SCALE_S);

}  // namespace mdrobot
