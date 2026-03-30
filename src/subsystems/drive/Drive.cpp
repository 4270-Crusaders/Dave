#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"

Drive::Drive()
    : left_motors_({drive_constants::kLeftMotorPort0, drive_constants::kLeftMotorPort1,
                    drive_constants::kLeftMotorPort2},
                   pros::MotorGearset::blue),
      right_motors_({drive_constants::kRightMotorPort0, drive_constants::kRightMotorPort1,
                     drive_constants::kRightMotorPort2},
                    pros::MotorGearset::blue),
      imu_(drive_constants::kImuPort),
      vertical_enc_(drive_constants::kVerticalRotationPort),
      horizontal_enc_(drive_constants::kHorizontalRotationPort),
      vertical_wheel_(&vertical_enc_, drive_constants::kWheelDiamIn, drive_constants::kVerticalWheelOffsetIn),
      horizontal_wheel_(&horizontal_enc_, drive_constants::kWheelDiamIn, drive_constants::kHorizontalWheelOffsetIn),
      drivetrain_(&left_motors_, &right_motors_, static_cast<float>(drive_constants::kTrackWidthIn),
                  drive_constants::kWheelDiamIn, drive_constants::kDriveRpm, drive_constants::kHorizontalDrift),
      linear_settings_(drive_constants::kLinearKp, drive_constants::kLinearKi, drive_constants::kLinearKd,
                       drive_constants::kLinearWindup, drive_constants::kLinearSmallErr,
                       drive_constants::kLinearSmallTimeout, drive_constants::kLinearLargeErr,
                       drive_constants::kLinearLargeTimeout, drive_constants::kLinearSlew),
      angular_settings_(drive_constants::kAngularKp, drive_constants::kAngularKi, drive_constants::kAngularKd,
                        drive_constants::kAngularWindup, drive_constants::kAngularSmallErr,
                        drive_constants::kAngularSmallTimeout, drive_constants::kAngularLargeErr,
                        drive_constants::kAngularLargeTimeout, drive_constants::kAngularSlew),
      sensors_(&vertical_wheel_, nullptr, &horizontal_wheel_, nullptr, &imu_),
      throttle_curve_(static_cast<float>(drive_constants::kDriveTeleopDeadband),
                      static_cast<float>(drive_constants::kDriveTeleopMinOutputThrottle),
                      drive_constants::kExpoCurveGain),
      steer_curve_(static_cast<float>(drive_constants::kDriveTeleopDeadband),
                   static_cast<float>(drive_constants::kDriveTeleopMinOutputTurn),
                   drive_constants::kExpoCurveGain),
      chassis_(drivetrain_, linear_settings_, angular_settings_, sensors_, &throttle_curve_, &steer_curve_) {}

void Drive::periodic() {}

void Drive::arcade(double throttle, double turn) {
	chassis_.arcade(static_cast<int>(throttle), static_cast<int>(turn), true);
}

void Drive::tank(int left, int right) { chassis_.tank(left, right, true); }

void Drive::curvature(int throttle, int turn) { chassis_.curvature(throttle, turn, true); }

void Drive::calibrate(bool calibrateImu) { chassis_.calibrate(calibrateImu); }

lemlib::Pose Drive::getPose(bool radians, bool standardPos) {
	return chassis_.getPose(radians, standardPos);
}

void Drive::setPose(float x, float y, float theta, bool radians) {
	chassis_.setPose(x, y, theta, radians);
}

void Drive::setPose(lemlib::Pose pose, bool radians) { chassis_.setPose(pose, radians); }

void Drive::resetLocalPosition() { chassis_.resetLocalPosition(); }

void Drive::moveToPoint(float x, float y, int timeout, lemlib::MoveToPointParams params, bool async) {
	chassis_.moveToPoint(x, y, timeout, params, async);
}

void Drive::moveToPose(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params, bool async) {
	chassis_.moveToPose(x, y, theta, timeout, params, async);
}

void Drive::moveToPoseBoomerang(float x, float y, float theta, int timeout, lemlib::MoveToPoseParams params,
                                bool async) {
	if (params.lead <= 0.0F) {
		params.lead = 0.72F;
	}
	chassis_.moveToPose(x, y, theta, timeout, params, async);
}

void Drive::turnToHeading(float theta, int timeout, lemlib::TurnToHeadingParams params, bool async) {
	chassis_.turnToHeading(theta, timeout, params, async);
}

void Drive::turnToPoint(float x, float y, int timeout, lemlib::TurnToPointParams params, bool async) {
	chassis_.turnToPoint(x, y, timeout, params, async);
}

void Drive::swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
                           lemlib::SwingToHeadingParams params, bool async) {
	chassis_.swingToHeading(theta, lockedSide, timeout, params, async);
}

void Drive::swingToPoint(float x, float y, lemlib::DriveSide lockedSide, int timeout,
                       lemlib::SwingToPointParams params, bool async) {
	chassis_.swingToPoint(x, y, lockedSide, timeout, params, async);
}

void Drive::follow(const asset& path, float lookahead, int timeout, bool forwards, bool async) {
	chassis_.follow(path, lookahead, timeout, forwards, async);
}

void Drive::waitUntilDone() { chassis_.waitUntilDone(); }

void Drive::waitUntil(float dist) { chassis_.waitUntil(dist); }

bool Drive::isInMotion() const { return chassis_.isInMotion(); }

void Drive::cancelMotion() { chassis_.cancelMotion(); }

void Drive::cancelAllMotions() { chassis_.cancelAllMotions(); }

void Drive::setBrakeMode(pros::motor_brake_mode_e_t mode) {
	chassis_.setBrakeMode(static_cast<pros::motor_brake_mode_e>(mode));
}
