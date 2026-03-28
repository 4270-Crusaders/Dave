#include "subsystems/drive/localization/mcl_runtime.h"

#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/localization/mcl_filter.h"

#include "pros/distance.hpp"

#include <cstdint>
#include <vector>

namespace drive_mcl_detail {

static localization::MclFilter* gFilter = nullptr;
static std::vector<pros::Distance>* gDists = nullptr;
static drivetrain::Pose gLastOdom{};
static bool gHaveLast = false;

} // namespace drive_mcl_detail

void localization_init_mcl(Drive* drive) {
	using namespace drive_mcl_detail;
	if (!drive_constants::kEnableMcl || drive == nullptr) {
		return;
	}
	delete gFilter;
	delete gDists;
	gFilter = new localization::MclFilter();
	gDists = new std::vector<pros::Distance>();
	gDists->reserve(drive_constants::kMclDistanceMounts.size());
	for (const auto& m : drive_constants::kMclDistanceMounts) {
		gDists->emplace_back(m.port);
	}
	const drivetrain::Pose p = drive->getPose(true);
	gFilter->resetAround(p.x, p.y, p.theta, 6.0, 0.4);
	gLastOdom = p;
	gHaveLast = true;
}

void localization_tick_mcl(Drive* drive) {
	using namespace drive_mcl_detail;
	if (!drive_constants::kEnableMcl || drive == nullptr || gFilter == nullptr || gDists == nullptr) {
		return;
	}
	const drivetrain::Pose now = drive->getPose(true);
	if (!gHaveLast) {
		gLastOdom = now;
		gHaveLast = true;
		return;
	}
	gFilter->predictFromOdom(gLastOdom, now);
	gLastOdom = now;
	std::vector<std::int32_t> readings;
	readings.reserve(gDists->size());
	for (auto& s : *gDists) {
		readings.push_back(s.get_distance());
	}
	gFilter->updateAllDistances(readings.data(), readings.size());
	gFilter->resampleIfNeeded();
}

drivetrain::Pose localization_get_mcl_estimate() {
	using namespace drive_mcl_detail;
	if (gFilter == nullptr) {
		return {};
	}
	return gFilter->estimateMean();
}

