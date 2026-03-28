#pragma once

#include "subsystems/drive/drivetrain/types.h"
#include "subsystems/drive/drivetrain/math.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace drive_odom {

/**
 * LemLib-inspired tracking wheel odometry + IMU heading fusion.
 *
 * Design notes:
 * - Mirrors the Pilons-style local delta integration used by LemLib TrackingWheelOdom.
 * - Supports multi-IMU heading (circular mean) and wheel-heading fallback.
 * - Adds IME (drivetrain motor encoder) sources as a first-class option for forward/back odom
 *   when vertical tracking wheels are absent.
 *
 * Units:
 * - Distances in inches.
 * - Angles in radians internally in the returned `drivetrain::Pose`.
 */
class OdomFusion {
public:
	struct WheelSource {
		/** Distance delta since last update (inches). */
		std::function<double()> distanceDeltaIn;
		/** Total distance traveled (inches) for wheel-heading (optional). */
		std::function<double()> distanceTotalIn;
		/** Offset from tracking center (inches). */
		double offsetIn = 0.0;
	};

	/** Heading in radians (absolute, no user offset applied). */
	using HeadingSource = std::function<double()>;

	struct MotorImuDriveSource {
		/** Left side distance delta since last update (inches). */
		std::function<double()> leftDeltaIn;
		/** Right side distance delta since last update (inches). */
		std::function<double()> rightDeltaIn;
	};

	OdomFusion() = default;

	void setPose(const drivetrain::Pose& pose_rad) {
		// Maintain a heading offset between fused sensor heading and user-facing pose heading.
		headingOffsetRad_ += drivetrain::normalizeAngleRad(pose_rad.theta - pose_.theta);
		pose_ = pose_rad;
	}

	drivetrain::Pose getPose() const { return pose_; }

	void clear() {
		pose_ = {};
		headingOffsetRad_ = 0.0;
	}

	void setHeadingSources(std::vector<HeadingSource> imus) { imuSources_ = std::move(imus); }
	void setVerticalWheels(std::vector<WheelSource> wheels) { verticalWheels_ = std::move(wheels); }
	void setHorizontalWheels(std::vector<WheelSource> wheels) { horizontalWheels_ = std::move(wheels); }
	void setImeDriveSource(std::optional<MotorImuDriveSource> ime) { imeDrive_ = std::move(ime); }

	/**
	 * Update pose from available sensors.
	 * Caller should call at a consistent rate (e.g. every 10ms).
	 */
	void update();

private:
	struct WheelData {
		double distanceIn = 0.0;
		double offsetIn = 0.0;
	};

	static WheelData takeFirstWheelDelta(std::vector<WheelSource>& wheels);
	static std::optional<double> wheelHeadingRad(std::vector<WheelSource>& wheels);
	static std::optional<double> imuHeadingRad(const std::vector<HeadingSource>& imus);

	static double circularMeanRad(const std::vector<double>& angles_rad);

	drivetrain::Pose pose_{};
	double headingOffsetRad_ = 0.0;

	std::vector<HeadingSource> imuSources_{};
	std::vector<WheelSource> verticalWheels_{};
	std::vector<WheelSource> horizontalWheels_{};
	std::optional<MotorImuDriveSource> imeDrive_{};
};

} // namespace drive_odom

