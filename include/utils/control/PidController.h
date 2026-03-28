#pragma once

#include <algorithm>
#include <cmath>

namespace control {

/**
 * Generic position- or velocity-loop PID (parallel form).
 * Units are caller-defined: e.g. error in degrees, output in millivolts.
 */
class PidController {
public:
	PidController(double kp, double ki, double kd, double integral_windup = 10000.0,
	                double output_min = -1e100, double output_max = 1e100)
		: kp_(kp),
		  ki_(ki),
		  kd_(kd),
		  windup_(integral_windup),
		  out_min_(output_min),
		  out_max_(output_max) {}

	void setGains(double kp, double ki, double kd) {
		kp_ = kp;
		ki_ = ki;
		kd_ = kd;
	}

	void setOutputLimits(double min_out, double max_out) {
		out_min_ = min_out;
		out_max_ = max_out;
	}

	void reset() {
		integral_ = 0;
		prev_err_ = 0;
		first_ = true;
	}

	/** PI + D on error; integral clamped to ±windup. */
	double update(double error, double dt_sec) {
		if (dt_sec <= 0.0 || !std::isfinite(dt_sec)) {
			return 0.0;
		}
		integral_ = std::clamp(integral_ + error * dt_sec, -windup_, windup_);
		const double deriv = first_ ? 0.0 : (error - prev_err_) / dt_sec;
		first_ = false;
		prev_err_ = error;
		const double u = kp_ * error + ki_ * integral_ + kd_ * deriv;
		return std::clamp(u, out_min_, out_max_);
	}

	double kp() const { return kp_; }
	double ki() const { return ki_; }
	double kd() const { return kd_; }

private:
	double kp_, ki_, kd_;
	double windup_;
	double out_min_;
	double out_max_;
	double integral_ = 0;
	double prev_err_ = 0;
	bool first_ = true;
};

} // namespace control
