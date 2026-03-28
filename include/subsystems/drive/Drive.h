#pragma once

#include "utils/command/subsystem.h"
#include "subsystems/drive/drivetrain/drivetrain.h"
#include "pros/motors.h"

/**
 * Drive subsystem: chassis (odom, teleop voltages, autonomous motions) + MCL tick in periodic().
 * Command factories live in commands/DriveCommands.h (FRC-style).
 */
class Drive : public Subsystem {
public:
	Drive();

	void periodic() override;

	void arcade(double throttle, double turn);
	void tank(int left, int right);
	void curvature(int throttle, int turn);

	void calibrate(bool calibrateImu = true);
	drivetrain::Pose getPose(bool radians = false, bool standardPos = false);
	void setPose(float x, float y, float theta, bool radians = false);
	void setPose(drivetrain::Pose pose, bool radians = false);
	void resetLocalPosition();

	void moveToPoint(float x, float y, int timeout, drivetrain::MoveToPointParams params = {}, bool async = true);
	void moveToPose(float x, float y, float theta, int timeout, drivetrain::MoveToPoseParams params = {}, bool async = true);
	void moveToPoseBoomerang(float x, float y, float theta, int timeout, drivetrain::MoveToPoseBoomerangParams params = {},
	                         bool async = true);
	void turnToHeading(float theta, int timeout, drivetrain::TurnToHeadingParams params = {}, bool async = true);
	void turnToPoint(float x, float y, int timeout, drivetrain::TurnToPointParams params = {}, bool async = true);
	void swingToHeading(float theta, drivetrain::DriveSide lockedSide, int timeout,
	                    drivetrain::SwingToHeadingParams params = {}, bool async = true);
	void swingToPoint(float x, float y, drivetrain::DriveSide lockedSide, int timeout,
	                  drivetrain::SwingToPointParams params = {}, bool async = true);
	void followPath(const drivetrain::Path& path, float lookahead, int timeout, bool forwards = true, bool async = true,
	                drivetrain::MoveToPointParams speedParams = {110.f, 25.f});

	void waitUntilDone();
	void waitUntil(float dist);
	bool isInMotion() const;
	void cancelMotion();
	void cancelAllMotions();

	void setBrakeMode(pros::motor_brake_mode_e_t mode);

	~Drive() override = default;

private:
	drivetrain::Chassis chassis_;
};

#include "subsystems/drive/Drive.inl.h"
