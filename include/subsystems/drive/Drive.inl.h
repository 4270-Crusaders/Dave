#pragma once

#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/localization/mcl_filter.h"

#include "pros/distance.hpp"

#include <memory>
#include <vector>

namespace drive_mcl_detail {

inline localization::MclFilter*& mclFilterPtr() {
	static localization::MclFilter* p = nullptr;
	return p;
}

inline std::vector<pros::Distance>*& mclDistVec() {
	static std::vector<pros::Distance>* v = nullptr;
	return v;
}

inline drivetrain::Pose& mclLastOdom() {
	static drivetrain::Pose pose{};
	return pose;
}

inline bool& mclHaveLast() {
	static bool h = false;
	return h;
}

} // namespace drive_mcl_detail

inline void localization_init_mcl(Drive* drive) {
	using namespace drive_mcl_detail;
	if (!drive_constants::kEnableMcl || drive == nullptr) {
		return;
	}
	delete mclFilterPtr();
	delete mclDistVec();
	mclFilterPtr() = nullptr;
	mclDistVec() = nullptr;
	mclHaveLast() = false;
	mclFilterPtr() = new localization::MclFilter();
	mclDistVec() = new std::vector<pros::Distance>();
	mclDistVec()->reserve(drive_constants::kMclDistanceMounts.size());
	for (const auto& m : drive_constants::kMclDistanceMounts) {
		mclDistVec()->emplace_back(m.port);
	}
	const drivetrain::Pose p = drive->getPose(true);
	mclFilterPtr()->resetAround(p.x, p.y, p.theta, 6.0, 0.4);
	mclLastOdom() = p;
	mclHaveLast() = true;
}

inline void localization_tick_mcl(Drive* drive) {
	using namespace drive_mcl_detail;
	if (!drive_constants::kEnableMcl || drive == nullptr || mclFilterPtr() == nullptr || mclDistVec() == nullptr) {
		return;
	}
	const drivetrain::Pose now = drive->getPose(true);
	if (!mclHaveLast()) {
		mclLastOdom() = now;
		mclHaveLast() = true;
		return;
	}
	mclFilterPtr()->predictFromOdom(mclLastOdom(), now);
	mclLastOdom() = now;
	std::vector<std::int32_t> readings;
	readings.reserve(mclDistVec()->size());
	for (auto& s : *mclDistVec()) {
		readings.push_back(s.get_distance());
	}
	mclFilterPtr()->updateAllDistances(readings.data(), readings.size());
	mclFilterPtr()->resampleIfNeeded();
}

inline drivetrain::Pose localization_get_mcl_estimate() {
	using namespace drive_mcl_detail;
	if (mclFilterPtr() == nullptr) {
		return {};
	}
	return mclFilterPtr()->estimateMean();
}

inline Drive::Drive() : chassis_{} {}

inline void Drive::periodic() {
	chassis_.tick();
	localization_tick_mcl(this);
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
