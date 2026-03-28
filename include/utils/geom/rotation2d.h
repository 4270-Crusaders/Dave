#pragma once

#include "utils/geom/math.h"

namespace geom {

/** 2D rotation in the plane; angle is CCW positive, stored in radians (WPILib-style). */
class Rotation2d {
public:
	constexpr Rotation2d() = default;
	explicit constexpr Rotation2d(double radians) : m_rad(radians) {}

	static Rotation2d fromRadians(double r) { return Rotation2d(r); }
	static Rotation2d fromDegrees(double deg) { return Rotation2d(degToRad(deg)); }

	double radians() const { return m_rad; }
	double degrees() const { return radToDeg(m_rad); }
	double cos() const { return std::cos(m_rad); }
	double sin() const { return std::sin(m_rad); }

	Rotation2d operator+(const Rotation2d& o) const { return Rotation2d(m_rad + o.m_rad); }
	Rotation2d operator-(const Rotation2d& o) const { return Rotation2d(m_rad - o.m_rad); }
	Rotation2d operator-() const { return Rotation2d(-m_rad); }
	Rotation2d operator*(double s) const { return Rotation2d(m_rad * s); }

	Rotation2d& operator+=(const Rotation2d& o) {
		m_rad += o.m_rad;
		return *this;
	}

	bool operator==(const Rotation2d& o) const { return m_rad == o.m_rad; }

private:
	double m_rad = 0;
};

} // namespace geom
