#include "subsystems/drive/odometry/OdomFusion.h"

#include <cmath>
#include <limits>

namespace drive_odom {

OdomFusion::WheelData OdomFusion::takeFirstWheelDelta(std::vector<WheelSource>& wheels) {
	for (std::size_t i = 0; i < wheels.size(); ++i) {
		const double d = wheels[i].distanceDeltaIn ? wheels[i].distanceDeltaIn() : 0.0;
		if (!std::isfinite(d)) {
			wheels.erase(wheels.begin() + static_cast<std::ptrdiff_t>(i));
			--i;
			continue;
		}
		return {d, wheels[i].offsetIn};
	}
	return {};
}

std::optional<double> OdomFusion::wheelHeadingRad(std::vector<WheelSource>& wheels) {
	if (wheels.size() < 2) {
		return std::nullopt;
	}
	const double d1 = wheels[0].distanceTotalIn ? wheels[0].distanceTotalIn() : std::numeric_limits<double>::quiet_NaN();
	const double d2 = wheels[1].distanceTotalIn ? wheels[1].distanceTotalIn() : std::numeric_limits<double>::quiet_NaN();
	const double o1 = wheels[0].offsetIn;
	const double o2 = wheels[1].offsetIn;
	if (!std::isfinite(d1)) {
		wheels.erase(wheels.begin());
		return wheelHeadingRad(wheels);
	}
	if (!std::isfinite(d2)) {
		wheels.erase(wheels.begin() + 1);
		return wheelHeadingRad(wheels);
	}
	if (std::abs(o1 - o2) < 1e-9) {
		wheels.erase(wheels.begin() + 1);
		return wheelHeadingRad(wheels);
	}
	// LemLib returns `from_stRad((d1 - d2) / (o1 - o2)) + 90_stDeg`.
	// Our pose uses theta=0 pointing along +x, so we keep the raw wheel-derived heading.
	return (d1 - d2) / (o1 - o2);
}

std::optional<double> OdomFusion::imuHeadingRad(const std::vector<HeadingSource>& imus) {
	std::vector<double> angles;
	angles.reserve(imus.size());
	for (const auto& src : imus) {
		if (!src) {
			continue;
		}
		const double a = src();
		if (std::isfinite(a)) {
			angles.push_back(a);
		}
	}
	if (angles.empty()) {
		return std::nullopt;
	}
	return circularMeanRad(angles);
}

double OdomFusion::circularMeanRad(const std::vector<double>& angles_rad) {
	double s = 0.0;
	double c = 0.0;
	for (double a : angles_rad) {
		s += std::sin(a);
		c += std::cos(a);
	}
	return std::atan2(s, c);
}

void OdomFusion::update() {
	// Step 1: compute lateral deltas (vertical/horizontal). LemLib picks the first available wheel in each vector.
	WheelData horiz = takeFirstWheelDelta(horizontalWheels_);
	WheelData vert = takeFirstWheelDelta(verticalWheels_);

	// IME fallback: if no vertical tracking wheels exist, drive fwd from drivetrain motor encoders.
	if (verticalWheels_.empty() && imeDrive_) {
		const double dl = imeDrive_->leftDeltaIn ? imeDrive_->leftDeltaIn() : 0.0;
		const double dr = imeDrive_->rightDeltaIn ? imeDrive_->rightDeltaIn() : 0.0;
		if (std::isfinite(dl) && std::isfinite(dr)) {
			vert.distanceIn = (dl + dr) * 0.5;
			vert.offsetIn = 0.0;
		}
	}

	// Step 2: heading selection. Prefer IMUs; then horizontal wheels; then vertical wheels.
	std::optional<double> thetaOpt = imuHeadingRad(imuSources_);
	if (!thetaOpt) {
		thetaOpt = wheelHeadingRad(horizontalWheels_);
	}
	if (!thetaOpt) {
		thetaOpt = wheelHeadingRad(verticalWheels_);
	}
	if (!thetaOpt) {
		// Not enough sensors; do nothing.
		return;
	}
	const double theta = drivetrain::normalizeAngleRad(headingOffsetRad_ + *thetaOpt);

	// Step 3: local delta integration (Pilons).
	const double deltaTheta = drivetrain::normalizeAngleRad(theta - pose_.theta);
	double localY = vert.distanceIn;
	double localX = horiz.distanceIn;
	if (std::abs(deltaTheta) > 1e-9) {
		const double s = 2.0 * std::sin(deltaTheta / 2.0);
		localY = s * (vert.distanceIn / deltaTheta + vert.offsetIn);
		localX = s * (horiz.distanceIn / deltaTheta + horiz.offsetIn);
	}

	// Step 4: rotate into global and apply.
	const double thMid = pose_.theta + deltaTheta / 2.0;
	const double c = std::cos(thMid);
	const double s = std::sin(thMid);
	pose_.x += localY * c - localX * s;
	pose_.y += localY * s + localX * c;
	pose_.theta = theta;
}

} // namespace drive_odom

