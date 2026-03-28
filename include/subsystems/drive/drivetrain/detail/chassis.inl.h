#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

namespace {

struct SegClosest {
	std::size_t segIndex = 0;
	double t = 0;
	double px = 0;
	double py = 0;
};

inline SegClosest closestOnPolyline(const drivetrain::Path& path, double rx, double ry, std::size_t startSeg) {
	SegClosest out{};
	if (path.size() < 2) {
		return out;
	}
	double best = 1e100;
	for (std::size_t i = startSeg; i + 1 < path.size(); ++i) {
		double ax = path[i].x;
		double ay = path[i].y;
		double bx = path[i + 1].x;
		double by = path[i + 1].y;
		double abx = bx - ax;
		double aby = by - ay;
		double ab2 = abx * abx + aby * aby;
		double t = ab2 > 1e-9 ? drivetrain::clampd(((rx - ax) * abx + (ry - ay) * aby) / ab2, 0.0, 1.0) : 0.0;
		double qx = ax + t * abx;
		double qy = ay + t * aby;
		double d = std::hypot(rx - qx, ry - qy);
		if (d < best) {
			best = d;
			out.segIndex = i;
			out.t = t;
			out.px = qx;
			out.py = qy;
		}
	}
	return out;
}

inline bool advanceAlongPath(const drivetrain::Path& path, std::size_t segIndex, double tStart, double advance,
                             std::size_t& outSeg, double& outT, double& outX, double& outY) {
	if (path.size() < 2 || segIndex >= path.size() - 1) {
		return false;
	}
	std::size_t i = segIndex;
	double t = tStart;
	double remaining = advance;
	while (i < path.size() - 1) {
		double ax = path[i].x;
		double ay = path[i].y;
		double bx = path[i + 1].x;
		double by = path[i + 1].y;
		double segLen = std::hypot(bx - ax, by - ay);
		double usable = (1.0 - t) * segLen;
		if (remaining <= usable + 1e-6) {
			double u = t + remaining / std::max(segLen, 1e-6);
			outSeg = i;
			outT = drivetrain::clampd(u, 0.0, 1.0);
			outX = ax + outT * (bx - ax);
			outY = ay + outT * (by - ay);
			return true;
		}
		remaining -= usable;
		++i;
		t = 0.0;
	}
	outSeg = path.size() - 2;
	outT = 1.0;
	outX = path.back().x;
	outY = path.back().y;
	return true;
}

} // namespace

inline drivetrain::Chassis::Chassis(ChassisConfig cfg)
	: cfg_(std::move(cfg)),
	  leftMotors_(cfg_.leftMotorPorts, pros::v5::MotorGears::blue),
	  rightMotors_(cfg_.rightMotorPorts, pros::v5::MotorGears::blue),
	  imu_(cfg_.imuPort),
	  vertRot_(cfg_.verticalRotationPort),
	  horizRot_(cfg_.horizontalRotationPort) {}

inline int32_t drivetrain::Chassis::deltaCentideg(int32_t prev, int32_t cur) {
	int32_t d = cur - prev;
	const int32_t wrap = 36000;
	while (d > wrap / 2) {
		d -= wrap;
	}
	while (d < -wrap / 2) {
		d += wrap;
	}
	return d;
}

inline double drivetrain::Chassis::centidegToInches(int32_t dCenti) const {
	const double rev = (static_cast<double>(dCenti) / 100.0) / 360.0;
	return rev * kPi * cfg_.trackingWheelDiameterIn;
}

inline double drivetrain::Chassis::headingRad() const { return degToRad(imu_.get_rotation()) + headingOffsetRad_; }

inline void drivetrain::Chassis::updateOdometry() {
	const int32_t v = vertRot_.get_position();
	const int32_t h = horizRot_.get_position();
	if (firstOdomSample_) {
		prevVertCenti_ = v;
		prevHorizCenti_ = h;
		firstOdomSample_ = false;
		return;
	}
	const int32_t dv = deltaCentideg(prevVertCenti_, v);
	const int32_t dh = deltaCentideg(prevHorizCenti_, h);
	prevVertCenti_ = v;
	prevHorizCenti_ = h;
	const double fwd = centidegToInches(dv);
	const double lat = centidegToInches(dh);
	const double th = headingRad();
	const double c = std::cos(th);
	const double s = std::sin(th);
	x_ += fwd * c - lat * s;
	y_ += fwd * s + lat * c;
}

inline drivetrain::Pose drivetrain::Chassis::getPose() const { return {x_, y_, headingRad()}; }

inline drivetrain::Pose drivetrain::Chassis::getPose(bool radians, bool /*standardPos*/) const {
	if (radians) {
		return {x_, y_, headingRad()};
	}
	return {x_, y_, radToDeg(headingRad())};
}

inline void drivetrain::Chassis::setPose(double x, double y, double theta, bool radians) {
	x_ = x;
	y_ = y;
	const double thetaRad = radians ? theta : degToRad(theta);
	headingOffsetRad_ = thetaRad - degToRad(imu_.get_rotation());
}

inline void drivetrain::Chassis::resetLocalPosition() {
	x_ = 0;
	y_ = 0;
}

inline void drivetrain::Chassis::calibrateImu() {
	imu_.reset(true);
	firstOdomSample_ = true;
}

inline int drivetrain::Chassis::clampVolt(int v) {
	if (v > 127) {
		return 127;
	}
	if (v < -127) {
		return -127;
	}
	return v;
}

inline void drivetrain::Chassis::applyTank(int left, int right) {
	leftMotors_.move(clampVolt(left));
	rightMotors_.move(clampVolt(right));
}

inline void drivetrain::Chassis::arcade(int throttle, int turn) {
	throttle = clampVolt(throttle);
	turn = clampVolt(turn);
	applyTank(throttle + turn, throttle - turn);
}

inline void drivetrain::Chassis::tank(int left, int right) { applyTank(left, right); }

inline void drivetrain::Chassis::curvature(int throttle, int turn) {
	throttle = clampVolt(throttle);
	turn = clampVolt(turn);
	if (std::abs(throttle) < 8) {
		applyTank(turn, -turn);
		return;
	}
	double ratio = (127.0 - std::abs(turn)) / 127.0;
	int slower = static_cast<int>(std::lround(std::abs(throttle) * ratio));
	slower = clampVolt(slower);
	if (throttle >= 0) {
		if (turn >= 0) {
			applyTank(throttle, slower);
		} else {
			applyTank(slower, throttle);
		}
	} else {
		if (turn >= 0) {
			applyTank(throttle, -slower);
		} else {
			applyTank(-slower, throttle);
		}
	}
}

inline void drivetrain::Chassis::setBrakeMode(pros::motor_brake_mode_e_t mode) {
	leftMotors_.set_brake_mode_all(mode);
	rightMotors_.set_brake_mode_all(mode);
}

inline void drivetrain::Chassis::startMotion(Motion m, int timeoutMs) {
	cancelMotion();
	motion_ = m;
	motionStartMs_ = pros::millis();
	motionTimeoutMs_ = timeoutMs;
	distPid_.reset();
	headingPid_.reset();
	turnPid_.reset();
}

inline void drivetrain::Chassis::finishMotion() {
	motion_ = Motion::None;
	applyTank(0, 0);
}

inline bool drivetrain::Chassis::isInMotion() const { return motion_ != Motion::None; }

inline void drivetrain::Chassis::cancelMotion() {
	motion_ = Motion::None;
	movePosePhase_ = 0;
	applyTank(0, 0);
}

inline void drivetrain::Chassis::waitUntilDone() {
	while (isInMotion()) {
		pros::delay(10);
	}
}

inline void drivetrain::Chassis::waitUntil(double distInches) {
	while (isInMotion()) {
		if (motion_ == Motion::MoveToPoint || (motion_ == Motion::MoveToPose && movePosePhase_ == 0)) {
			const double dx = targetX_ - x_;
			const double dy = targetY_ - y_;
			if (std::hypot(dx, dy) < distInches) {
				break;
			}
		}
		pros::delay(10);
	}
}

inline void drivetrain::Chassis::tick() {
	const std::uint32_t now = pros::millis();
	double dt = (lastTickMs_ == 0) ? 0.01 : static_cast<double>(now - lastTickMs_) / 1000.0;
	lastTickMs_ = now;
	if (dt > 0.1) {
		dt = 0.1;
	}
	updateOdometry();
	if (motion_ == Motion::None) {
		return;
	}
	if (static_cast<int>(now - motionStartMs_) >= motionTimeoutMs_) {
		finishMotion();
		return;
	}
	switch (motion_) {
	case Motion::MoveToPoint:
		stepMoveToPoint(dt);
		break;
	case Motion::MoveToPose:
		stepMoveToPose(dt);
		break;
	case Motion::TurnToHeading:
		stepTurnToHeading(dt);
		break;
	case Motion::TurnToPoint:
		stepTurnToPoint(dt);
		break;
	case Motion::SwingHeading:
		stepSwingHeading(dt);
		break;
	case Motion::SwingPoint:
		stepSwingPoint(dt);
		break;
	case Motion::FollowPath:
		stepFollowPath(dt);
		break;
	default:
		break;
	}
}

inline void drivetrain::Chassis::moveToPoint(double x, double y, int timeoutMs, MoveToPointParams params, bool async) {
	targetX_ = x;
	targetY_ = y;
	movePointParams_ = params;
	startMotion(Motion::MoveToPoint, timeoutMs);
	if (!async) {
		waitUntilDone();
	}
}

inline void drivetrain::Chassis::moveToPose(double x, double y, double theta, int timeoutMs, MoveToPoseParams params,
                                            bool async) {
	targetX_ = x;
	targetY_ = y;
	targetTheta_ = degToRad(theta);
	movePoseParams_ = params;
	movePosePhase_ = 0;
	startMotion(Motion::MoveToPose, timeoutMs);
	if (!async) {
		waitUntilDone();
	}
}

inline void drivetrain::Chassis::turnToHeading(double theta, int timeoutMs, TurnToHeadingParams params, bool async) {
	targetTheta_ = degToRad(theta);
	turnHeadingParams_ = params;
	startMotion(Motion::TurnToHeading, timeoutMs);
	if (!async) {
		waitUntilDone();
	}
}

inline void drivetrain::Chassis::turnToPoint(double x, double y, int timeoutMs, TurnToPointParams params, bool async) {
	targetX_ = x;
	targetY_ = y;
	turnPointParams_ = params;
	startMotion(Motion::TurnToPoint, timeoutMs);
	if (!async) {
		waitUntilDone();
	}
}

inline void drivetrain::Chassis::swingToHeading(double theta, DriveSide lockedSide, int timeoutMs,
                                                SwingToHeadingParams params, bool async) {
	targetTheta_ = degToRad(theta);
	swingLocked_ = lockedSide;
	swingHeadingParams_ = params;
	startMotion(Motion::SwingHeading, timeoutMs);
	if (!async) {
		waitUntilDone();
	}
}

inline void drivetrain::Chassis::swingToPoint(double x, double y, DriveSide lockedSide, int timeoutMs,
                                              SwingToPointParams params, bool async) {
	targetX_ = x;
	targetY_ = y;
	swingLocked_ = lockedSide;
	swingPointParams_ = params;
	startMotion(Motion::SwingPoint, timeoutMs);
	if (!async) {
		waitUntilDone();
	}
}

inline void drivetrain::Chassis::followPath(const Path& path, double lookaheadIn, int timeoutMs, bool forwards,
                                            bool async, MoveToPointParams speedParams) {
	followPath_ = path;
	followLookahead_ = lookaheadIn;
	followForwards_ = forwards;
	followPathIndex_ = 0;
	followSpeedParams_ = speedParams;
	startMotion(Motion::FollowPath, timeoutMs);
	if (!async) {
		waitUntilDone();
	}
}

inline void drivetrain::Chassis::stepMoveToPoint(double dt) {
	const double dx = targetX_ - x_;
	const double dy = targetY_ - y_;
	const double dist = std::hypot(dx, dy);
	if (dist < kPosTolIn) {
		finishMotion();
		return;
	}
	const double aim = std::atan2(dy, dx);
	const double hErr = normalizeAngleRad(aim - headingRad());
	const double maxS = static_cast<double>(movePointParams_.maxSpeed);
	const double minS = static_cast<double>(movePointParams_.minSpeed);
	double forward = distPid_.update(dist, dt) * std::cos(hErr);
	double turn = headingPid_.update(hErr, dt);
	forward = clampd(forward, -maxS, maxS);
	turn = clampd(turn, -maxS, maxS);
	if (std::abs(forward) < minS && dist > kPosTolIn) {
		forward = (forward >= 0 ? 1.0 : -1.0) * minS;
	}
	int l = clampVolt(static_cast<int>(std::lround(forward - turn)));
	int r = clampVolt(static_cast<int>(std::lround(forward + turn)));
	applyTank(l, r);
}

inline void drivetrain::Chassis::stepMoveToPose(double dt) {
	if (movePosePhase_ == 0) {
		const double dx = targetX_ - x_;
		const double dy = targetY_ - y_;
		const double dist = std::hypot(dx, dy);
		if (dist < kPosTolIn + 0.5) {
			movePosePhase_ = 1;
			distPid_.reset();
			headingPid_.reset();
			turnPid_.reset();
			return;
		}
		const double aim = std::atan2(dy, dx);
		const double hErr = normalizeAngleRad(aim - headingRad());
		const double maxS = static_cast<double>(movePoseParams_.maxSpeed);
		const double minS = static_cast<double>(movePoseParams_.minSpeed);
		double forward = distPid_.update(dist, dt) * std::cos(hErr);
		double turn = headingPid_.update(hErr, dt);
		forward = clampd(forward, -maxS, maxS);
		turn = clampd(turn, -maxS, maxS);
		if (std::abs(forward) < minS && dist > kPosTolIn) {
			forward = (forward >= 0 ? 1.0 : -1.0) * minS;
		}
		applyTank(clampVolt(static_cast<int>(std::lround(forward - turn))),
		          clampVolt(static_cast<int>(std::lround(forward + turn))));
		return;
	}
	const double err = normalizeAngleRad(targetTheta_ - headingRad());
	if (std::abs(err) < kAngleTolRad) {
		finishMotion();
		return;
	}
	const double maxT = static_cast<double>(movePoseParams_.turnMaxSpeed);
	double turn = turnPid_.update(err, dt);
	turn = clampd(turn, -maxT, maxT);
	applyTank(clampVolt(static_cast<int>(std::lround(-turn))), clampVolt(static_cast<int>(std::lround(turn))));
}

inline void drivetrain::Chassis::stepTurnToHeading(double dt) {
	const double err = normalizeAngleRad(targetTheta_ - headingRad());
	if (std::abs(err) < kAngleTolRad) {
		finishMotion();
		return;
	}
	const double maxS = static_cast<double>(turnHeadingParams_.maxSpeed);
	double turn = turnPid_.update(err, dt);
	turn = clampd(turn, -maxS, maxS);
	if (std::abs(turn) < turnHeadingParams_.minSpeed) {
		turn = (err >= 0 ? 1.0 : -1.0) * static_cast<double>(turnHeadingParams_.minSpeed);
	}
	applyTank(clampVolt(static_cast<int>(std::lround(-turn))), clampVolt(static_cast<int>(std::lround(turn))));
}

inline void drivetrain::Chassis::stepTurnToPoint(double dt) {
	const double aim = std::atan2(targetY_ - y_, targetX_ - x_);
	const double err = normalizeAngleRad(aim - headingRad());
	if (std::abs(err) < kAngleTolRad) {
		finishMotion();
		return;
	}
	const double maxS = static_cast<double>(turnPointParams_.maxSpeed);
	double turn = turnPid_.update(err, dt);
	turn = clampd(turn, -maxS, maxS);
	if (std::abs(turn) < turnPointParams_.minSpeed) {
		turn = (err >= 0 ? 1.0 : -1.0) * static_cast<double>(turnPointParams_.minSpeed);
	}
	applyTank(clampVolt(static_cast<int>(std::lround(-turn))), clampVolt(static_cast<int>(std::lround(turn))));
}

inline void drivetrain::Chassis::stepSwingHeading(double dt) {
	const double err = normalizeAngleRad(targetTheta_ - headingRad());
	if (std::abs(err) < kAngleTolRad) {
		finishMotion();
		return;
	}
	const double maxS = static_cast<double>(swingHeadingParams_.maxSpeed);
	double turn = turnPid_.update(err, dt);
	turn = clampd(turn, -maxS, maxS);
	if (std::abs(turn) < swingHeadingParams_.minSpeed) {
		turn = (err >= 0 ? 1.0 : -1.0) * static_cast<double>(swingHeadingParams_.minSpeed);
	}
	const int t = clampVolt(static_cast<int>(std::lround(turn)));
	if (swingLocked_ == DriveSide::Left) {
		applyTank(0, t);
	} else {
		applyTank(t, 0);
	}
}

inline void drivetrain::Chassis::stepSwingPoint(double dt) {
	const double aim = std::atan2(targetY_ - y_, targetX_ - x_);
	const double err = normalizeAngleRad(aim - headingRad());
	if (std::abs(err) < kAngleTolRad) {
		finishMotion();
		return;
	}
	const double maxS = static_cast<double>(swingPointParams_.maxSpeed);
	double turn = turnPid_.update(err, dt);
	turn = clampd(turn, -maxS, maxS);
	if (std::abs(turn) < swingPointParams_.minSpeed) {
		turn = (err >= 0 ? 1.0 : -1.0) * static_cast<double>(swingPointParams_.minSpeed);
	}
	const int t = clampVolt(static_cast<int>(std::lround(turn)));
	if (swingLocked_ == DriveSide::Left) {
		applyTank(0, t);
	} else {
		applyTank(t, 0);
	}
}

inline void drivetrain::Chassis::stepFollowPath(double dt) {
	(void)dt;
	if (followPath_.size() < 2) {
		finishMotion();
		return;
	}
	const SegClosest close = closestOnPolyline(followPath_, x_, y_, followPathIndex_);
	followPathIndex_ = close.segIndex;
	std::size_t outSeg = 0;
	double outT = 0;
	double lx = 0;
	double ly = 0;
	advanceAlongPath(followPath_, close.segIndex, close.t, followLookahead_, outSeg, outT, lx, ly);
	const double dx = lx - x_;
	const double dy = ly - y_;
	const double Ld = std::max(std::hypot(dx, dy), 1.0);
	double alpha = normalizeAngleRad(std::atan2(dy, dx) - headingRad());
	if (!followForwards_) {
		alpha = normalizeAngleRad(alpha + kPi);
	}
	const double kappa = 2.0 * std::sin(alpha) / Ld;
	const double maxS = static_cast<double>(followSpeedParams_.maxSpeed);
	double v = maxS * 0.85;
	const double track = cfg_.trackWidthIn;
	const double w = v * kappa * (track / 2.0) * 0.18;
	double left = v - w;
	double right = v + w;
	if (!followForwards_) {
		left = -left;
		right = -right;
	}
	left = clampd(left, -maxS, maxS);
	right = clampd(right, -maxS, maxS);
	applyTank(clampVolt(static_cast<int>(std::lround(left))), clampVolt(static_cast<int>(std::lround(right))));
	const double fx = followPath_.back().x;
	const double fy = followPath_.back().y;
	if (std::hypot(fx - x_, fy - y_) < 2.5 && std::abs(alpha) < degToRad(12.0)) {
		finishMotion();
	}
}
