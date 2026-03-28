#pragma once

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace drive_control {

class ExitCondition {
public:
	ExitCondition(double range, std::uint32_t settle_ms) : range_(range), settleMs_(settle_ms) {}

	bool update(double input) {
		if (!std::isfinite(input)) {
			reset();
			return false;
		}
		const std::uint32_t now = pros::millis();
		if (std::abs(input) >= range_) {
			startMs_.reset();
			return false;
		}
		if (!startMs_.has_value()) {
			startMs_ = now;
			return false;
		}
		return static_cast<std::int32_t>(now - *startMs_) >= static_cast<std::int32_t>(settleMs_);
	}

	void reset() { startMs_.reset(); }

private:
	double range_ = 0.0;
	std::uint32_t settleMs_ = 0;
	std::optional<std::uint32_t> startMs_{};
};

class ExitConditionGroup {
public:
	explicit ExitConditionGroup(std::vector<ExitCondition> conditions) : conditions_(std::move(conditions)) {}

	bool update(double input) {
		for (auto& c : conditions_) {
			if (c.update(input)) {
				return true;
			}
		}
		return false;
	}

	void reset() {
		for (auto& c : conditions_) {
			c.reset();
		}
	}

private:
	std::vector<ExitCondition> conditions_;
};

enum class SlewDirection { Increasing, Decreasing, All };

inline double slew(double target, double current, double max_change_per_sec, double dt_sec,
                   SlewDirection dir = SlewDirection::All) {
	if (max_change_per_sec == 0.0 || dt_sec <= 0.0 || !std::isfinite(dt_sec)) {
		return target;
	}
	const double change = target - current;
	if (dir == SlewDirection::Increasing && change < 0.0) {
		return target;
	}
	if (dir == SlewDirection::Decreasing && change > 0.0) {
		return target;
	}
	const double max_change = std::abs(max_change_per_sec * dt_sec);
	if (std::abs(change) <= max_change) {
		return target;
	}
	return current + (change > 0.0 ? max_change : -max_change);
}

inline double constrainPower(double power, double max_abs, double min_abs) {
	if (!std::isfinite(power)) {
		return 0.0;
	}
	if (min_abs > 0.0 && std::abs(power) < min_abs) {
		power = (power >= 0 ? 1.0 : -1.0) * min_abs;
	}
	return std::clamp(power, -max_abs, max_abs);
}

struct DriveOutputs {
	double left = 0.0;
	double right = 0.0;
};

/** LemLib-style desaturation: keep `left/right` within [-1,1] by scaling when |L|+|R|>1. */
inline DriveOutputs desaturate(double lateral, double angular) {
	const double left = lateral - angular;
	const double right = lateral + angular;
	const double sum = std::abs(left) + std::abs(right);
	if (sum <= 1.0) {
		return {left, right};
	}
	return {left / sum, right / sum};
}

} // namespace drive_control

