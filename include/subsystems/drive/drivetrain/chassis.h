#pragma once

#include "subsystems/drive/drivetrain/math.h"
#include "subsystems/drive/drivetrain/pid.h"
#include "subsystems/drive/drivetrain/types.h"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <cstdint>
#include <vector>

namespace drivetrain {

struct ChassisConfig {
	std::vector<std::int8_t> leftMotorPorts = {-11, 12, -13};
	std::vector<std::int8_t> rightMotorPorts = {18, 19, -20};
	std::uint8_t imuPort = 16;
	std::int8_t verticalRotationPort = -5;
	std::int8_t horizontalRotationPort = 6;
	double trackWidthIn = 10.0;
	double trackingWheelDiameterIn = 2.0;
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
	pros::Imu imu_;
	pros::Rotation vertRot_;
	pros::Rotation horizRot_;

	double x_ = 0;
	double y_ = 0;
	double headingOffsetRad_ = 0;

	int32_t prevVertCenti_ = 0;
	int32_t prevHorizCenti_ = 0;
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

	static constexpr double kPosTolIn = 1.75;
	static constexpr double kAngleTolRad = 2.5 * kPi / 180.0;
};

#include "subsystems/drive/drivetrain/detail/chassis.inl.h"

} // namespace drivetrain
