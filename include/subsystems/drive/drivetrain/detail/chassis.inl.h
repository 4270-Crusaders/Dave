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
	  leftMotors_(cfg_.leftMotorPorts, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees),
	  rightMotors_(cfg_.rightMotorPorts, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees),
	  imus_([&]() {
		  // Default: use legacy single IMU if user didn't supply a vector.
		  std::vector<pros::Imu> out;
		  const auto& ports = cfg_.imuPorts.empty() ? std::vector<std::uint8_t>{cfg_.imuPort} : cfg_.imuPorts;
		  out.reserve(ports.size());
		  for (std::uint8_t p : ports) {
			  out.emplace_back(p);
		  }
		  return out;
	  }()),
	  verticalRots_([&]() {
		  // Default: use legacy single vertical rotation if user didn't supply a vector.
		  std::vector<pros::Rotation> out;
		  if (cfg_.verticalWheels.empty()) {
			  out.emplace_back(cfg_.verticalRotationPort);
			  return out;
		  }
		  out.reserve(cfg_.verticalWheels.size());
		  for (const auto& w : cfg_.verticalWheels) {
			  out.emplace_back(w.port);
		  }
		  return out;
	  }()),
	  horizontalRots_([&]() {
		  // Default: use legacy single horizontal rotation if user didn't supply a vector.
		  std::vector<pros::Rotation> out;
		  if (cfg_.horizontalWheels.empty()) {
			  out.emplace_back(cfg_.horizontalRotationPort);
			  return out;
		  }
		  out.reserve(cfg_.horizontalWheels.size());
		  for (const auto& w : cfg_.horizontalWheels) {
			  out.emplace_back(w.port);
		  }
		  return out;
	  }()) {
	// LemLib-style odometry fusion setup (multi-IMU + arbitrary wheels).

	// IME fallback always available (used when vertical wheels list is empty in fusion).
	ime_.left = &leftMotors_;
	ime_.right = &rightMotors_;
	ime_.inchesPerMotorDeg = (kPi * cfg_.driveWheelDiameterIn) / 360.0; // 360deg = one shaft rev

	// Heading sources: feed raw IMU headings (no user offset). Fusion will circular-mean them.
	std::vector<drive_odom::OdomFusion::HeadingSource> headings;
	headings.reserve(imus_.size());
	for (auto& imu : imus_) {
		headings.emplace_back([&imu]() { return degToRad(imu.get_rotation()); });
	}
	odom_.setHeadingSources(std::move(headings));

	// Wheel sources (rotation sensors).
	std::vector<drive_odom::OdomFusion::WheelSource> vws;
	std::vector<drive_odom::OdomFusion::WheelSource> hws;

	if (cfg_.verticalWheels.empty()) {
		// Default legacy: single wheel using `trackingWheelDiameterIn`, offset 0.
		legacyVertWheel_.rot = verticalRots_.empty() ? nullptr : &verticalRots_[0];
		legacyVertWheel_.inchesPerCentiDeg = (kPi * cfg_.trackingWheelDiameterIn) / 36000.0;
		legacyVertWheel_.offsetIn = 0.0;
		vws.push_back({[this]() { return legacyVertWheel_.deltaIn(); },
		               [this]() { return legacyVertWheel_.totalIn(); },
		               0.0});
	} else {
		verticalWheelAdapters_.clear();
		verticalWheelAdapters_.reserve(cfg_.verticalWheels.size());
		for (std::size_t i = 0; i < cfg_.verticalWheels.size(); ++i) {
			const auto& wc = cfg_.verticalWheels[i];
			RotationWheelAdapter a{};
			a.rot = &verticalRots_[i];
			a.inchesPerCentiDeg = (kPi * wc.diameterIn * wc.ratio) / 36000.0;
			a.offsetIn = wc.offsetIn;
			verticalWheelAdapters_.push_back(a);
			vws.push_back({[this, i]() { return verticalWheelAdapters_[i].deltaIn(); },
			               [this, i]() { return verticalWheelAdapters_[i].totalIn(); },
			               wc.offsetIn});
		}
	}

	if (cfg_.horizontalWheels.empty()) {
		legacyHorizWheel_.rot = horizontalRots_.empty() ? nullptr : &horizontalRots_[0];
		legacyHorizWheel_.inchesPerCentiDeg = (kPi * cfg_.trackingWheelDiameterIn) / 36000.0;
		legacyHorizWheel_.offsetIn = 0.0;
		hws.push_back({[this]() { return legacyHorizWheel_.deltaIn(); },
		               [this]() { return legacyHorizWheel_.totalIn(); },
		               0.0});
	} else {
		horizontalWheelAdapters_.clear();
		horizontalWheelAdapters_.reserve(cfg_.horizontalWheels.size());
		for (std::size_t i = 0; i < cfg_.horizontalWheels.size(); ++i) {
			const auto& wc = cfg_.horizontalWheels[i];
			RotationWheelAdapter a{};
			a.rot = &horizontalRots_[i];
			a.inchesPerCentiDeg = (kPi * wc.diameterIn * wc.ratio) / 36000.0;
			a.offsetIn = wc.offsetIn;
			horizontalWheelAdapters_.push_back(a);
			hws.push_back({[this, i]() { return horizontalWheelAdapters_[i].deltaIn(); },
			               [this, i]() { return horizontalWheelAdapters_[i].totalIn(); },
			               wc.offsetIn});
		}
	}

	odom_.setVerticalWheels(std::move(vws));
	odom_.setHorizontalWheels(std::move(hws));
	odom_.setImeDriveSource(drive_odom::OdomFusion::MotorImuDriveSource{
		[this]() { return ime_.leftDeltaIn(); },
		[this]() { return ime_.rightDeltaIn(); },
	});
}

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

inline double drivetrain::Chassis::headingRad() const {
	const double raw = imus_.empty() ? odom_.getPose().theta : degToRad(imus_[0].get_rotation());
	return raw + headingOffsetRad_;
}

inline void drivetrain::Chassis::updateOdometry() {
	if (firstOdomSample_) {
		for (auto& w : verticalWheelAdapters_) {
			w.reset();
		}
		for (auto& w : horizontalWheelAdapters_) {
			w.reset();
		}
		legacyVertWheel_.reset();
		legacyHorizWheel_.reset();
		ime_.reset();
		firstOdomSample_ = false;
		odom_.setPose({x_, y_, headingRad()});
		return;
	}
	odom_.update();
	const drivetrain::Pose p = odom_.getPose();
	x_ = p.x;
	y_ = p.y;
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
	const double raw = imus_.empty() ? thetaRad : degToRad(imus_[0].get_rotation());
	headingOffsetRad_ = thetaRad - raw;
	odom_.setPose({x_, y_, thetaRad});
}

inline void drivetrain::Chassis::resetLocalPosition() {
	x_ = 0;
	y_ = 0;
	odom_.setPose({0.0, 0.0, headingRad()});
}

inline void drivetrain::Chassis::calibrateImu() {
	for (auto& imu : imus_) {
		imu.reset(true);
	}
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
	movePoseBoomerangPrevLat_ = 0.0;
	movePoseBoomerangPrevAng_ = 0.0;
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
	case Motion::MoveToPoseBoomerang:
		stepMoveToPoseBoomerang(dt);
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

inline void drivetrain::Chassis::moveToPoseBoomerang(double x, double y, double theta, int timeoutMs,
                                                     MoveToPoseBoomerangParams params, bool async) {
	targetX_ = x;
	targetY_ = y;
	targetTheta_ = degToRad(theta);
	movePoseBoomerangParams_ = params;
	movePoseBoomerangPrevLat_ = 0.0;
	movePoseBoomerangPrevAng_ = 0.0;
	startMotion(Motion::MoveToPoseBoomerang, timeoutMs);
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

inline void drivetrain::Chassis::stepMoveToPoseBoomerang(double dt) {
	// Port of LemLib moveToPose core idea (carrot point + combined lateral/angular control).
	// Outputs are in PROS motor units [-127,127].
	const double dx = targetX_ - x_;
	const double dy = targetY_ - y_;
	const double dist = std::hypot(dx, dy);
	if (dist < movePoseBoomerangParams_.settleDistIn &&
	    std::abs(normalizeAngleRad(targetTheta_ - headingRad())) < degToRad(movePoseBoomerangParams_.settleAngleDeg)) {
		finishMotion();
		return;
	}

	// Motion chaining: if user requests early exit and we've crossed the target plane, finish.
	if (movePoseBoomerangParams_.earlyExitRangeIn > 0.0) {
		const double tx = x_ - targetX_;
		const double ty = y_ - targetY_;
		const double plane = tx * std::cos(targetTheta_) + ty * std::sin(targetTheta_);
		if (plane > static_cast<double>(movePoseBoomerangParams_.earlyExitRangeIn)) {
			finishMotion();
			return;
		}
	}

	// Carrot point: when far, offset behind the target along its heading.
	const double close = dist < movePoseBoomerangParams_.closeRangeIn ? 1.0 : 0.0;
	const double lead = movePoseBoomerangParams_.lead;
	const double carrotX = close > 0.5 ? targetX_ : (targetX_ - std::cos(targetTheta_) * (lead * dist));
	const double carrotY = close > 0.5 ? targetY_ : (targetY_ - std::sin(targetTheta_) * (lead * dist));

	// Lateral error: distance * cos(angle error between heading and direction-to-carrot).
	const double aim = std::atan2(carrotY - y_, carrotX - x_);
	const double hErr = normalizeAngleRad(aim - headingRad());
	double lateralErr = dist;
	const double scalar = std::cos(hErr);
	lateralErr *= (close > 0.5) ? scalar : (scalar >= 0 ? 1.0 : -1.0);

	// Angular error: when far, face carrot; when close, face final theta.
	const double desiredHeading = (close > 0.5) ? targetTheta_ : aim;
	const double angErr = normalizeAngleRad(desiredHeading - headingRad());

	// Controller outputs.
	const double maxLat = static_cast<double>(movePoseBoomerangParams_.maxLateralSpeed);
	const double minLat = static_cast<double>(movePoseBoomerangParams_.minLateralSpeed);
	const double maxAng = static_cast<double>(movePoseBoomerangParams_.maxAngularSpeed);

	double angularOut = turnPid_.update(angErr, dt);
	angularOut = clampd(angularOut, -maxAng, maxAng);

	double lateralOut = distPid_.update(lateralErr, dt);
	lateralOut = clampd(lateralOut, -maxLat, maxLat);

	// Slew (only while not close).
	if (close < 0.5) {
		lateralOut = drive_control::slew(lateralOut, movePoseBoomerangPrevLat_, movePoseBoomerangParams_.lateralSlew,
		                                 dt);
		angularOut = drive_control::slew(angularOut, movePoseBoomerangPrevAng_, movePoseBoomerangParams_.angularSlew,
		                                 dt);
	}
	movePoseBoomerangPrevLat_ = lateralOut;
	movePoseBoomerangPrevAng_ = angularOut;

	// Prevent moving in the wrong direction while far (LemLib behavior).
	if (close < 0.5) {
		if (movePoseBoomerangParams_.reversed) {
			lateralOut = std::min(lateralOut, 0.0);
		} else {
			lateralOut = std::max(lateralOut, 0.0);
		}
	}

	// Minimum speed (only while not close).
	if (close < 0.5) {
		lateralOut = drive_control::constrainPower(lateralOut, maxLat, minLat);
	}

	// Drift / slip cap (optional).
	if (movePoseBoomerangParams_.driftCompensation > 0.0) {
		// Radius from signed tangent arc curvature (Pilons / LemLib).
		const double tx = carrotX - x_;
		const double ty = carrotY - y_;
		const double th = headingRad();
		const double side = (std::sin(th) * tx - std::cos(th) * ty) >= 0.0 ? 1.0 : -1.0;
		const double a = -std::tan(th);
		const double c = std::tan(th) * x_ - y_;
		const double x = std::abs(a * carrotX + carrotY + c) / std::sqrt(a * a + 1.0);
		const double d = std::hypot(tx, ty);
		const double curvature = (d > 1e-6) ? (side * ((2.0 * x) / (d * d))) : 0.0;
		const double radius = (std::abs(curvature) > 1e-9) ? (1.0 / std::abs(curvature)) : 1e9;
		const double maxSlip = std::sqrt(std::abs(movePoseBoomerangParams_.driftCompensation) * radius);
		lateralOut = clampd(lateralOut, -maxSlip, maxSlip);
	}

	// Prioritize angular movement over lateral movement (LemLib overthrow behavior).
	{
		const double overturn = std::abs(angularOut) + std::abs(lateralOut) - maxLat;
		if (overturn > 0.0) {
			lateralOut -= (lateralOut > 0.0 ? overturn : -overturn);
		}
	}

	// Desaturate to keep within [-127,127] while preserving mix.
	const auto out = drive_control::desaturate(lateralOut / 127.0, angularOut / 127.0);
	applyTank(clampVolt(static_cast<int>(std::lround(out.left * 127.0))),
	          clampVolt(static_cast<int>(std::lround(out.right * 127.0))));
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
