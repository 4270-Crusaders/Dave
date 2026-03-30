#pragma once

#include <cstdint>

/**
 * LemLib chassis hardware + auton/teleop tuning. Port signs follow PROS (negative = reversed).
 * @see https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html
 */
namespace drive_constants {

// --- Motors (3+3 example; edit to match your robot) ---
inline constexpr std::int8_t kLeftMotorPort0 = -11;
inline constexpr std::int8_t kLeftMotorPort1 = 12;
inline constexpr std::int8_t kLeftMotorPort2 = -13;
inline constexpr std::int8_t kRightMotorPort0 = 18;
inline constexpr std::int8_t kRightMotorPort1 = 19;
inline constexpr std::int8_t kRightMotorPort2 = -20;

inline constexpr std::uint8_t kImuPort = 16;
inline constexpr std::int8_t kVerticalRotationPort = -5;
inline constexpr std::int8_t kHorizontalRotationPort = 6;

inline constexpr double kTrackWidthIn = 10.0;

/** Tracking + drive wheel diameter for LemLib (NEW 2" omni ≈ 2.125 in). */
inline constexpr float kWheelDiamIn = 2.125f;

inline constexpr float kDriveRpm = 600.f;
inline constexpr float kHorizontalDrift = 2.f;

/** Offsets from tracking center (inches); tune using LemLib odometry guide. */
inline constexpr float kVerticalWheelOffsetIn = -2.5f;
inline constexpr float kHorizontalWheelOffsetIn = -2.5f;

// --- Auton command settling ---
inline constexpr double kDriveCommandPosTolIn = 1.75;
inline constexpr double kDriveCommandAngleTolDeg = 2.5;
inline constexpr int kDriveCommandSettleMs = 80;

// --- Teleop (used with DriverInput + LemLib curves disabled in arcade) ---
inline constexpr std::int32_t kDriveTeleopDeadband = 6;
inline constexpr double kDriveTeleopExpoThrottle = 0.25;
inline constexpr double kDriveTeleopExpoTurn = 0.25;
inline constexpr std::int32_t kDriveTeleopMinOutputThrottle = 0;
inline constexpr std::int32_t kDriveTeleopMinOutputTurn = 0;
inline constexpr double kDriveTeleopTurnSteerPriority = 0.2;

// --- LemLib lateral controller (tune on field) ---
inline constexpr float kLinearKp = 10.f;
inline constexpr float kLinearKi = 0.f;
inline constexpr float kLinearKd = 3.f;
inline constexpr float kLinearWindup = 3.f;
inline constexpr float kLinearSmallErr = 1.f;
inline constexpr float kLinearSmallTimeout = 100.f;
inline constexpr float kLinearLargeErr = 3.f;
inline constexpr float kLinearLargeTimeout = 500.f;
inline constexpr float kLinearSlew = 20.f;

// --- LemLib angular controller ---
inline constexpr float kAngularKp = 2.f;
inline constexpr float kAngularKi = 0.f;
inline constexpr float kAngularKd = 10.f;
inline constexpr float kAngularWindup = 3.f;
inline constexpr float kAngularSmallErr = 1.f;
inline constexpr float kAngularSmallTimeout = 100.f;
inline constexpr float kAngularLargeErr = 3.f;
inline constexpr float kAngularLargeTimeout = 500.f;
inline constexpr float kAngularSlew = 0.f;

/** Expo curve gain for LemLib `ExpoDriveCurve` (throttle / steer). */
inline constexpr float kExpoCurveGain = 1.019f;

} // namespace drive_constants
