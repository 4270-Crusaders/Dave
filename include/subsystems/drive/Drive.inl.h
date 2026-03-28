#pragma once

#include "subsystems/drive/DriveConstants.h"

inline Drive::Drive() : chassis_{} {}

inline void Drive::periodic() {
	chassis_.tick();
}

inline void Drive::arcade(double throttle, double turn) {
	chassis_.arcade(static_cast<int>(throttle), static_cast<int>(turn));
}

inline void Drive::tank(int left, int right) { chassis_.tank(left, right); }

inline void Drive::curvature(int throttle, int turn) { chassis_.curvature(throttle, turn); }

inline void Drive::calibrate(bool calibrateIMU) {
	if (calibrateIMU) {
		chassis_.calibrateImu();
	}
}

inline drivetrain::Pose Drive::getPose(bool radians, bool standardPos) {
	return chassis_.getPose(radians, standardPos);
}

inline void Drive::setPose(float x, float y, float theta, bool radians) {
	chassis_.setPose(static_cast<double>(x), static_cast<double>(y), static_cast<double>(theta), radians);
}

inline void Drive::setPose(drivetrain::Pose pose, bool radians) {
	chassis_.setPose(pose.x, pose.y, pose.theta, radians);
}

inline void Drive::resetLocalPosition() { chassis_.resetLocalPosition(); }

inline void Drive::moveToPoint(float x, float y, int timeout, drivetrain::MoveToPointParams params, bool async) {
	chassis_.moveToPoint(x, y, timeout, params, async);
}

inline void Drive::moveToPose(float x, float y, float theta, int timeout, drivetrain::MoveToPoseParams params,
                              bool async) {
	chassis_.moveToPose(x, y, theta, timeout, params, async);
}

inline void Drive::turnToHeading(float theta, int timeout, drivetrain::TurnToHeadingParams params, bool async) {
	chassis_.turnToHeading(theta, timeout, params, async);
}

inline void Drive::turnToPoint(float x, float y, int timeout, drivetrain::TurnToPointParams params, bool async) {
	chassis_.turnToPoint(x, y, timeout, params, async);
}

inline void Drive::swingToHeading(float theta, drivetrain::DriveSide lockedSide, int timeout,
                                  drivetrain::SwingToHeadingParams params, bool async) {
	chassis_.swingToHeading(theta, lockedSide, timeout, params, async);
}

inline void Drive::swingToPoint(float x, float y, drivetrain::DriveSide lockedSide, int timeout,
                                drivetrain::SwingToPointParams params, bool async) {
	chassis_.swingToPoint(x, y, lockedSide, timeout, params, async);
}

inline void Drive::followPath(const drivetrain::Path& path, float lookahead, int timeout, bool forwards, bool async,
                              drivetrain::MoveToPointParams speedParams) {
	chassis_.followPath(path, lookahead, timeout, forwards, async, speedParams);
}

inline void Drive::waitUntilDone() { chassis_.waitUntilDone(); }

inline void Drive::waitUntil(float dist) { chassis_.waitUntil(static_cast<double>(dist)); }

inline bool Drive::isInMotion() const { return chassis_.isInMotion(); }

inline void Drive::cancelMotion() { chassis_.cancelMotion(); }

inline void Drive::cancelAllMotions() { chassis_.cancelMotion(); }

inline void Drive::setBrakeMode(pros::motor_brake_mode_e_t mode) { chassis_.setBrakeMode(mode); }
