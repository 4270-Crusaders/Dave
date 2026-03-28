#pragma once

#include "subsystems/drive/drivetrain/math.h"
#include "subsystems/drive/drivetrain/pid.h"
#include "subsystems/drive/drivetrain/types.h"
#include "subsystems/drive/control/ControlPrimitives.h"
#include "subsystems/drive/odometry/OdomFusion.h"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace drivetrain {

struct ChassisConfig {
	std::vector<std::int8_t> leftMotorPorts = {-11, 12, -13};
	std::vector<std::int8_t> rightMotorPorts = {18, 19, -20};
	double trackWidthIn = 10.0;
	double trackingWheelDiameterIn = 2.0;

	/** Rotation-sensor tracking wheel description (inches). */
	struct RotationWheelConfig {
		std::int8_t port = 0;
		double diameterIn = 2.0;
		double offsetIn = 0.0;
		double ratio = 1.0; // driven / driving
	};

	/** IMU ports (supports 0..N). Default uses legacy `imuPort`. */
	std::vector<std::uint8_t> imuPorts{};

	/** Vertical (forward) tracking wheels (supports 0..N). Default uses legacy `verticalRotationPort`. */
	std::vector<RotationWheelConfig> verticalWheels{};

	/** Horizontal (lateral) tracking wheels (supports 0..N). Default uses legacy `horizontalRotationPort`. */
	std::vector<RotationWheelConfig> horizontalWheels{};

	/** IME (motor encoder) fallback: drive wheel diameter when no vertical tracking wheels exist. */
	double driveWheelDiameterIn = 2.0;

	// Legacy single-sensor fields (kept for backward compatibility).
	std::uint8_t imuPort = 16;
	std::int8_t verticalRotationPort = -5;
	std::int8_t horizontalRotationPort = 6;
};

/**
 * Tank drive + 2 tracking wheels + IMU odometry, PID point/turn motions, pure pursuit on a polyline path.
 */
class Chassis {
public:
	explicit Chassis(ChassisConfig cfg = {});

	void tick();

	// Odometry & pose (x, y inches; theta radians internally; getPose can return degrees)
	void updateOdometry();
	double headingRad() const;
	Pose getPose() const;
	/** theta in radians unless radians==false (then degrees). standardPos ignored (reserved). */
	Pose getPose(bool radians, bool standardPos = false) const;
	void setPose(double x, double y, double theta, bool radians = false);
	void resetLocalPosition();

	void calibrateImu();

	// Manual driving [-127, 127]
	void arcade(int throttle, int turn);
	void tank(int left, int right);
	void curvature(int throttle, int turn);

	// Motions (async unless async=false)
	void moveToPoint(double x, double y, int timeoutMs, MoveToPointParams params = {}, bool async = true);
	void moveToPose(double x, double y, double theta, int timeoutMs, MoveToPoseParams params = {}, bool async = true);
	void moveToPoseBoomerang(double x, double y, double theta, int timeoutMs, MoveToPoseBoomerangParams params = {},
	                         bool async = true);
	void turnToHeading(double theta, int timeoutMs, TurnToHeadingParams params = {}, bool async = true);
	void turnToPoint(double x, double y, int timeoutMs, TurnToPointParams params = {}, bool async = true);
	void swingToHeading(double theta, DriveSide lockedSide, int timeoutMs, SwingToHeadingParams params = {},
	                    bool async = true);
	void swingToPoint(double x, double y, DriveSide lockedSide, int timeoutMs, SwingToPointParams params = {},
	                  bool async = true);
	void followPath(const Path& path, double lookaheadIn, int timeoutMs, bool forwards = true, bool async = true,
	                MoveToPointParams speedParams = {110.f, 25.f});

	void waitUntilDone();
	/** During MoveToPoint / MoveToPose phase 0, blocks until distance-to-target is below dist inches. */
	void waitUntil(double distInches);
	bool isInMotion() const;
	void cancelMotion();

	void setBrakeMode(pros::motor_brake_mode_e_t mode);

private:
	enum class Motion {
		None,
		MoveToPoint,
		MoveToPose,
		MoveToPoseBoomerang,
		TurnToHeading,
		TurnToPoint,
		SwingHeading,
		SwingPoint,
		FollowPath
	};

	void startMotion(Motion m, int timeoutMs);
	void finishMotion();
	void applyTank(int left, int right);
	static int clampVolt(int v);

	void stepMoveToPoint(double dt);
	void stepMoveToPose(double dt);
	void stepMoveToPoseBoomerang(double dt);
	void stepTurnToHeading(double dt);
	void stepTurnToPoint(double dt);
	void stepSwingHeading(double dt);
	void stepSwingPoint(double dt);
	void stepFollowPath(double dt);

	static int32_t deltaCentideg(int32_t prev, int32_t cur);
	double centidegToInches(int32_t dCenti) const;

	ChassisConfig cfg_;
	pros::MotorGroup leftMotors_;
	pros::MotorGroup rightMotors_;
	std::vector<pros::Imu> imus_{};
	std::vector<pros::Rotation> verticalRots_{};
	std::vector<pros::Rotation> horizontalRots_{};

	drive_odom::OdomFusion odom_{};

	struct RotationWheelAdapter {
		pros::Rotation* rot = nullptr;
		double inchesPerCentiDeg = 0.0;
		std::optional<std::int32_t> lastAngleCenti{};
		double offsetIn = 0.0;

		double totalIn() const {
			if (rot == nullptr) {
				return 0.0;
			}
			return static_cast<double>(rot->get_angle()) * inchesPerCentiDeg;
		}

		double deltaIn() {
			if (rot == nullptr) {
				return 0.0;
			}
			const std::int32_t cur = rot->get_angle();
			if (!lastAngleCenti.has_value()) {
				lastAngleCenti = cur;
				return 0.0;
			}
			const std::int32_t prev = *lastAngleCenti;
			lastAngleCenti = cur;
			const std::int32_t dCenti = Chassis::deltaCentideg(prev, cur);
			return static_cast<double>(dCenti) * inchesPerCentiDeg;
		}

		void reset() { lastAngleCenti.reset(); }
	};

	struct MotorImeAdapter {
		pros::MotorGroup* left = nullptr;
		pros::MotorGroup* right = nullptr;
		// motor group positions come in the motor encoder units; we will configure units to degrees.
		double inchesPerMotorDeg = 0.0;
		std::optional<double> lastLeftDeg{};
		std::optional<double> lastRightDeg{};

		double leftDeltaIn() {
			if (left == nullptr) {
				return 0.0;
			}
			const double cur = left->get_position();
			if (!lastLeftDeg.has_value()) {
				lastLeftDeg = cur;
				return 0.0;
			}
			const double d = cur - *lastLeftDeg;
			lastLeftDeg = cur;
			return d * inchesPerMotorDeg;
		}

		double rightDeltaIn() {
			if (right == nullptr) {
				return 0.0;
			}
			const double cur = right->get_position();
			if (!lastRightDeg.has_value()) {
				lastRightDeg = cur;
				return 0.0;
			}
			const double d = cur - *lastRightDeg;
			lastRightDeg = cur;
			return d * inchesPerMotorDeg;
		}

		void reset() {
			lastLeftDeg.reset();
			lastRightDeg.reset();
		}
	};

	MotorImeAdapter ime_{};
	std::vector<RotationWheelAdapter> verticalWheelAdapters_{};
	std::vector<RotationWheelAdapter> horizontalWheelAdapters_{};
	RotationWheelAdapter legacyVertWheel_{};
	RotationWheelAdapter legacyHorizWheel_{};

	double x_ = 0;
	double y_ = 0;
	double headingOffsetRad_ = 0;
	bool firstOdomSample_ = true;
	std::uint32_t lastTickMs_ = 0;

	Motion motion_ = Motion::None;
	std::uint32_t motionStartMs_ = 0;
	int motionTimeoutMs_ = 0;

	double targetX_ = 0;
	double targetY_ = 0;
	double targetTheta_ = 0;
	int movePosePhase_ = 0;
	DriveSide swingLocked_ = DriveSide::Left;

	MoveToPointParams movePointParams_{};
	MoveToPoseParams movePoseParams_{};
	MoveToPoseBoomerangParams movePoseBoomerangParams_{};
	TurnToHeadingParams turnHeadingParams_{};
	TurnToPointParams turnPointParams_{};
	SwingToHeadingParams swingHeadingParams_{};
	SwingToPointParams swingPointParams_{};

	Path followPath_{};
	double followLookahead_ = 8.0;
	bool followForwards_ = true;
	std::size_t followPathIndex_ = 0;
	MoveToPointParams followSpeedParams_{110.f, 25.f};

	Pid distPid_{3.5, 0.0, 0.12, 4000.0};
	Pid headingPid_{5.5, 0.0, 0.35, 3000.0};
	Pid turnPid_{6.0, 0.0, 0.4, 3000.0};

	// Boomerang motion internal state (slew tracking).
	double movePoseBoomerangPrevLat_ = 0.0;
	double movePoseBoomerangPrevAng_ = 0.0;

	static constexpr double kPosTolIn = 1.75;
	static constexpr double kAngleTolRad = 2.5 * kPi / 180.0;
};

} // namespace drivetrain

#include "subsystems/drive/drivetrain/detail/chassis.inl.h"
