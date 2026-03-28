#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace drive_driver {

struct AxisShape {
	/** Deadband in controller units [0,127]. */
	std::int32_t deadband = 6;
	/** Expo amount in [0,1]. 0 = linear, 1 = strong curve. */
	double expo = 0.25;
	/** Minimum output once outside deadband (controller units). */
	std::int32_t minOutput = 0;
};

inline double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

inline std::int32_t applyDeadband(std::int32_t v, std::int32_t deadband) {
	if (std::abs(v) <= deadband) {
		return 0;
	}
	// Remap remaining range back to full scale to keep maximum reachable.
	const int sign = (v >= 0) ? 1 : -1;
	const double mag = static_cast<double>(std::abs(v) - deadband) / static_cast<double>(127 - deadband);
	return sign * static_cast<std::int32_t>(std::lround(mag * 127.0));
}

inline std::int32_t applyExpo(std::int32_t v, double expo) {
	expo = clamp01(expo);
	const double x = static_cast<double>(v) / 127.0;
	const double y = (1.0 - expo) * x + expo * x * x * x;
	return static_cast<std::int32_t>(std::lround(y * 127.0));
}

inline std::int32_t applyMinOutput(std::int32_t v, std::int32_t min_out) {
	if (v == 0) {
		return 0;
	}
	const int sign = (v >= 0) ? 1 : -1;
	const int mag = std::abs(v);
	if (mag < min_out) {
		return sign * min_out;
	}
	return v;
}

inline std::int32_t shapeAxis(std::int32_t raw, const AxisShape& s) {
	raw = std::clamp(raw, -127, 127);
	std::int32_t v = applyDeadband(raw, s.deadband);
	v = applyExpo(v, s.expo);
	v = applyMinOutput(v, s.minOutput);
	return std::clamp(v, -127, 127);
}

struct ArcadeConfig {
	AxisShape throttle{};
	AxisShape turn{};
	/** When turn is large, reduce throttle. [0,1]. */
	double turnSteerPriority = 0.2;
};

struct ArcadeOut {
	std::int32_t throttle = 0;
	std::int32_t turn = 0;
};

inline ArcadeOut shapeArcade(std::int32_t raw_throttle, std::int32_t raw_turn, const ArcadeConfig& cfg) {
	std::int32_t t = shapeAxis(raw_throttle, cfg.throttle);
	std::int32_t r = shapeAxis(raw_turn, cfg.turn);
	const double pr = clamp01(cfg.turnSteerPriority);
	const double turn_frac = static_cast<double>(std::abs(r)) / 127.0;
	const double scale = 1.0 - pr * turn_frac;
	t = static_cast<std::int32_t>(std::lround(static_cast<double>(t) * scale));
	return {std::clamp(t, -127, 127), r};
}

} // namespace drive_driver

