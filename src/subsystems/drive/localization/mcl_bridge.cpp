#include "subsystems/drive/localization/mcl_runtime.h"

#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/localization/mcl_filter.h"

#include "pros/distance.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace {

std::unique_ptr<localization::MclFilter>& mclFilterPtr() {
	static std::unique_ptr<localization::MclFilter> p;
	return p;
}

std::unique_ptr<std::vector<pros::Distance>>& mclDistVec() {
	static std::unique_ptr<std::vector<pros::Distance>> v;
	return v;
}

drivetrain::Pose& mclLastOdom() {
	static drivetrain::Pose pose{};
	return pose;
}

bool& mclHaveLast() {
	static bool h = false;
	return h;
}

} // namespace

void localization_init_mcl(Drive* drive) {
	if (!drive_constants::kEnableMcl || drive == nullptr) {
		return;
	}

	mclFilterPtr().reset();
	mclDistVec().reset();
	mclHaveLast() = false;

	mclFilterPtr() = std::make_unique<localization::MclFilter>();
	mclDistVec() = std::make_unique<std::vector<pros::Distance>>();
	mclDistVec()->reserve(drive_constants::kMclDistanceMounts.size());
	for (const auto& m : drive_constants::kMclDistanceMounts) {
		mclDistVec()->emplace_back(m.port);
	}

	const drivetrain::Pose p = drive->getPose(true);
	mclFilterPtr()->resetAround(p.x, p.y, p.theta, 6.0, 0.4);
	mclLastOdom() = p;
	mclHaveLast() = true;
}

void localization_tick_mcl(Drive* drive) {
	if (!drive_constants::kEnableMcl || drive == nullptr || !mclFilterPtr() || !mclDistVec()) {
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

drivetrain::Pose localization_get_mcl_estimate() {
	if (!mclFilterPtr()) {
		return {};
	}
	return mclFilterPtr()->estimateMean();
}

