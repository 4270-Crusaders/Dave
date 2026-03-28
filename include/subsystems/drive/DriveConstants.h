#pragma once

/**
 * Drive / odometry / MCL tuning (FRC-style constants for this subsystem only).
 */
#include <array>
#include <cstddef>
#include <cstdint>

namespace drive_constants {

	inline constexpr bool kEnableMcl = false;

	inline constexpr double kFieldMinX = 0.0;
	inline constexpr double kFieldMinY = 0.0;
	inline constexpr double kFieldMaxX = 144.0;
	inline constexpr double kFieldMaxY = 144.0;

	inline constexpr std::size_t kMclParticleCount = 280;
	inline constexpr double kMclSigmaX = 0.35;
	inline constexpr double kMclSigmaY = 0.35;
	inline constexpr double kMclSigmaTheta = 0.04;
	inline constexpr double kMclSigmaMeasureIn = 3.0;
	inline constexpr double kMclOutlierMaxIn = 28.0;
	inline constexpr double kMclOutlierLogPenalty = 22.0;
	inline constexpr double kMclResampleEssFrac = 0.45;
	inline constexpr double kMclResamplePosJitterIn = 0.0;
	inline constexpr double kMclResampleThetaJitterRad = 0.0;
	inline constexpr std::int32_t kDistanceInvalidMm = 2500;

	/** Auton command finish: pose must stay within these bounds for `kDriveCommandSettleMs`. */
	inline constexpr double kDriveCommandPosTolIn = 1.75;
	inline constexpr double kDriveCommandAngleTolDeg = 2.5;
	inline constexpr int kDriveCommandSettleMs = 80;

	/** `followPath` end: match chassis pure-pursuit exit (position + heading vs path segment). */
	inline constexpr double kDriveCommandPathEndPosTolIn = 2.5;
	inline constexpr double kDriveCommandPathEndHeadingTolDeg = 12.0;

	struct DistanceSensorMount {
		std::uint8_t port;
		double offset_x_in;
		double offset_y_in;
		double bearing_rad;
	};

	inline constexpr std::array<DistanceSensorMount, 2> kMclDistanceMounts = {
		{7, 6.0, 0.0, 0.0},
		{8, -2.0, 5.0, 1.57079632679},
	};

} // namespace drive_constants
