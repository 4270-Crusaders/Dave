#pragma once

/**
 * FRC-style command factories for the drive subsystem (teleop defaults + motion commands).
 */
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/drivetrain/math.h"
#include "utils/command/command.h"
#include "utils/command/commandController.h"
#include "utils/command/functionalCommand.h"
#include "utils/command/runCommand.h"
#include "pros/misc.hpp"

#include <cmath>
#include <memory>

namespace DriveCommandDetail {

struct SettledGate {
	std::uint32_t near_since_ms = 0;
	void reset() { near_since_ms = 0; }
	/** True once `near_goal` has been true continuously for at least `settle_ms`. */
	bool update(std::uint32_t now_ms, bool near_goal, int settle_ms) {
		if (!near_goal) {
			near_since_ms = 0;
			return false;
		}
		if (near_since_ms == 0) {
			near_since_ms = now_ms;
		}
		return static_cast<int>(now_ms - near_since_ms) >= settle_ms;
	}
};

inline bool timedOut(std::uint32_t t0_ms, std::uint32_t now_ms, int timeout_ms) {
	return static_cast<int>(now_ms - t0_ms) >= timeout_ms;
}

inline double headingErrDegMag(double cur_deg, double target_deg) {
	double d = cur_deg - target_deg;
	while (d > 180.0) {
		d -= 360.0;
	}
	while (d < -180.0) {
		d += 360.0;
	}
	return std::abs(d);
}

} // namespace DriveCommandDetail

namespace DriveCommands {

inline RunCommand* arcadeDefaultCommand(Drive* drive, CommandController* controller) {
	return new RunCommand(
		[drive, controller]() {
			drive->arcade(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
			              controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		},
		{drive});
}

inline RunCommand* tankDefaultCommand(Drive* drive, CommandController* controller) {
	return new RunCommand(
		[drive, controller]() {
			drive->tank(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
			            controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		},
		{drive});
}

inline RunCommand* curvatureDefaultCommand(Drive* drive, CommandController* controller) {
	return new RunCommand(
		[drive, controller]() {
			drive->curvature(controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
			                 controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		},
		{drive});
}

inline Command* moveToPointCommand(Drive* drive, float x, float y, int timeout,
                                   drivetrain::MoveToPointParams params = {}) {
	auto t0 = std::make_shared<std::uint32_t>(0);
	auto gate = std::make_shared<DriveCommandDetail::SettledGate>();
	return new FunctionalCommand(
		[drive, x, y, timeout, params, t0, gate]() {
			gate->reset();
			*t0 = pros::millis();
			drive->moveToPoint(x, y, timeout, params, true);
		},
		[]() {},
		[drive](bool /*interrupted*/) { drive->cancelMotion(); },
		[drive, x, y, timeout, t0, gate]() {
			const std::uint32_t now = pros::millis();
			if (DriveCommandDetail::timedOut(*t0, now, timeout)) {
				return true;
			}
			const drivetrain::Pose p = drive->getPose(false);
			const bool near = std::hypot(static_cast<double>(x) - p.x, static_cast<double>(y) - p.y) <
			                  drive_constants::kDriveCommandPosTolIn;
			return gate->update(now, near, drive_constants::kDriveCommandSettleMs);
		},
		{drive});
}

inline Command* moveToPoseCommand(Drive* drive, float x, float y, float theta, int timeout,
                                  drivetrain::MoveToPoseParams params = {}) {
	auto t0 = std::make_shared<std::uint32_t>(0);
	auto gate = std::make_shared<DriveCommandDetail::SettledGate>();
	return new FunctionalCommand(
		[drive, x, y, theta, timeout, params, t0, gate]() {
			gate->reset();
			*t0 = pros::millis();
			drive->moveToPose(x, y, theta, timeout, params, true);
		},
		[]() {},
		[drive](bool /*interrupted*/) { drive->cancelMotion(); },
		[drive, x, y, theta, timeout, t0, gate]() {
			const std::uint32_t now = pros::millis();
			if (DriveCommandDetail::timedOut(*t0, now, timeout)) {
				return true;
			}
			// Translate then turn: don't require final heading until the chassis motion has finished.
			if (drive->isInMotion()) {
				gate->reset();
				return false;
			}
			const drivetrain::Pose p = drive->getPose(false);
			const bool pos_near = std::hypot(static_cast<double>(x) - p.x, static_cast<double>(y) - p.y) <
			                      drive_constants::kDriveCommandPosTolIn;
			const bool ang_near = DriveCommandDetail::headingErrDegMag(p.theta, static_cast<double>(theta)) <
			                      drive_constants::kDriveCommandAngleTolDeg;
			return gate->update(now, pos_near && ang_near, drive_constants::kDriveCommandSettleMs);
		},
		{drive});
}

inline Command* turnToHeadingCommand(Drive* drive, float theta, int timeout,
                                     drivetrain::TurnToHeadingParams params = {}) {
	auto t0 = std::make_shared<std::uint32_t>(0);
	auto gate = std::make_shared<DriveCommandDetail::SettledGate>();
	return new FunctionalCommand(
		[drive, theta, timeout, params, t0, gate]() {
			gate->reset();
			*t0 = pros::millis();
			drive->turnToHeading(theta, timeout, params, true);
		},
		[]() {},
		[drive](bool /*interrupted*/) { drive->cancelMotion(); },
		[drive, theta, timeout, t0, gate]() {
			const std::uint32_t now = pros::millis();
			if (DriveCommandDetail::timedOut(*t0, now, timeout)) {
				return true;
			}
			const drivetrain::Pose p = drive->getPose(false);
			const bool near = DriveCommandDetail::headingErrDegMag(p.theta, static_cast<double>(theta)) <
			                  drive_constants::kDriveCommandAngleTolDeg;
			return gate->update(now, near, drive_constants::kDriveCommandSettleMs);
		},
		{drive});
}

inline Command* turnToPointCommand(Drive* drive, float x, float y, int timeout,
                                   drivetrain::TurnToPointParams params = {}) {
	auto t0 = std::make_shared<std::uint32_t>(0);
	auto gate = std::make_shared<DriveCommandDetail::SettledGate>();
	return new FunctionalCommand(
		[drive, x, y, timeout, params, t0, gate]() {
			gate->reset();
			*t0 = pros::millis();
			drive->turnToPoint(x, y, timeout, params, true);
		},
		[]() {},
		[drive](bool /*interrupted*/) { drive->cancelMotion(); },
		[drive, x, y, timeout, t0, gate]() {
			const std::uint32_t now = pros::millis();
			if (DriveCommandDetail::timedOut(*t0, now, timeout)) {
				return true;
			}
			const drivetrain::Pose p = drive->getPose(true);
			const double aim = std::atan2(static_cast<double>(y) - p.y, static_cast<double>(x) - p.x);
			const bool near = std::abs(drivetrain::normalizeAngleRad(aim - p.theta)) <
			                  drivetrain::degToRad(drive_constants::kDriveCommandAngleTolDeg);
			return gate->update(now, near, drive_constants::kDriveCommandSettleMs);
		},
		{drive});
}

inline Command* swingToHeadingCommand(Drive* drive, float theta, drivetrain::DriveSide lockedSide, int timeout,
                                      drivetrain::SwingToHeadingParams params = {}) {
	auto t0 = std::make_shared<std::uint32_t>(0);
	auto gate = std::make_shared<DriveCommandDetail::SettledGate>();
	return new FunctionalCommand(
		[drive, theta, lockedSide, timeout, params, t0, gate]() {
			gate->reset();
			*t0 = pros::millis();
			drive->swingToHeading(theta, lockedSide, timeout, params, true);
		},
		[]() {},
		[drive](bool /*interrupted*/) { drive->cancelMotion(); },
		[drive, theta, timeout, t0, gate]() {
			const std::uint32_t now = pros::millis();
			if (DriveCommandDetail::timedOut(*t0, now, timeout)) {
				return true;
			}
			const drivetrain::Pose p = drive->getPose(false);
			const bool near = DriveCommandDetail::headingErrDegMag(p.theta, static_cast<double>(theta)) <
			                  drive_constants::kDriveCommandAngleTolDeg;
			return gate->update(now, near, drive_constants::kDriveCommandSettleMs);
		},
		{drive});
}

inline Command* swingToPointCommand(Drive* drive, float x, float y, drivetrain::DriveSide lockedSide, int timeout,
                                    drivetrain::SwingToPointParams params = {}) {
	auto t0 = std::make_shared<std::uint32_t>(0);
	auto gate = std::make_shared<DriveCommandDetail::SettledGate>();
	return new FunctionalCommand(
		[drive, x, y, lockedSide, timeout, params, t0, gate]() {
			gate->reset();
			*t0 = pros::millis();
			drive->swingToPoint(x, y, lockedSide, timeout, params, true);
		},
		[]() {},
		[drive](bool /*interrupted*/) { drive->cancelMotion(); },
		[drive, x, y, timeout, t0, gate]() {
			const std::uint32_t now = pros::millis();
			if (DriveCommandDetail::timedOut(*t0, now, timeout)) {
				return true;
			}
			const drivetrain::Pose p = drive->getPose(true);
			const double aim = std::atan2(static_cast<double>(y) - p.y, static_cast<double>(x) - p.x);
			const bool near = std::abs(drivetrain::normalizeAngleRad(aim - p.theta)) <
			                  drivetrain::degToRad(drive_constants::kDriveCommandAngleTolDeg);
			return gate->update(now, near, drive_constants::kDriveCommandSettleMs);
		},
		{drive});
}

inline Command* followPathCommand(Drive* drive, drivetrain::Path path, float lookahead, int timeout, bool forwards,
                                  drivetrain::MoveToPointParams speedParams = {110.f, 25.f}) {
	auto t0 = std::make_shared<std::uint32_t>(0);
	auto gate = std::make_shared<DriveCommandDetail::SettledGate>();
	return new FunctionalCommand(
		[drive, path, lookahead, timeout, forwards, speedParams, t0, gate]() {
			gate->reset();
			*t0 = pros::millis();
			drive->followPath(path, lookahead, timeout, forwards, true, speedParams);
		},
		[]() {},
		[drive](bool /*interrupted*/) { drive->cancelMotion(); },
		[drive, path, timeout, t0, gate]() {
			const std::uint32_t now = pros::millis();
			if (DriveCommandDetail::timedOut(*t0, now, timeout)) {
				return true;
			}
			if (path.empty()) {
				return true;
			}
			const drivetrain::Pose p = drive->getPose(true);
			const auto& end = path.back();
			const double dist = std::hypot(end.x - p.x, end.y - p.y);
			bool near = false;
			if (path.size() < 2) {
				near = dist < drive_constants::kDriveCommandPathEndPosTolIn;
			} else {
				const auto& prev = path[path.size() - 2];
				const double seg_bear = std::atan2(end.y - prev.y, end.x - prev.x);
				const double herr = std::abs(drivetrain::normalizeAngleRad(seg_bear - p.theta));
				near = dist < drive_constants::kDriveCommandPathEndPosTolIn &&
				       herr < drivetrain::degToRad(drive_constants::kDriveCommandPathEndHeadingTolDeg);
			}
			return gate->update(now, near, drive_constants::kDriveCommandSettleMs);
		},
		{drive});
}

} // namespace DriveCommands
