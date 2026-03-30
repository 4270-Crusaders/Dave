#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace drive_driver {

struct AxisShape {
	std::int32_t deadband = 6;
	double expo = 0.25;
	std::int32_t minOutput = 0;
};

inline double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

inline std::int32_t applyDeadband(std::int32_t v, std::int32_t deadband) {
	if (std::abs(v) <= deadband) {
		return 0;
	}
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
	const std::int32_t mag = std::abs(v);
	return sign * std::max(mag, min_out);
}

struct ArcadeConfig {
	AxisShape throttle{};
	AxisShape turn{};
	double turnSteerPriority = 0.2;
};

struct ArcadeOut {
	double throttle = 0;
	double turn = 0;
};

inline ArcadeOut shapeArcade(std::int32_t forward_raw, std::int32_t steer_raw, const ArcadeConfig& cfg) {
	std::int32_t f = applyDeadband(forward_raw, cfg.throttle.deadband);
	f = applyExpo(f, cfg.throttle.expo);
	f = applyMinOutput(f, cfg.throttle.minOutput);

	std::int32_t t = applyDeadband(steer_raw, cfg.turn.deadband);
	t = applyExpo(t, cfg.turn.expo);
	t = applyMinOutput(t, cfg.turn.minOutput);

	const double fp = static_cast<double>(std::abs(f)) / 127.0;
	const double mix = cfg.turnSteerPriority + (1.0 - cfg.turnSteerPriority) * fp;
	const double turn_scaled = static_cast<double>(t) * mix;

	return ArcadeOut{static_cast<double>(f), turn_scaled};
}

} // namespace drive_driver
