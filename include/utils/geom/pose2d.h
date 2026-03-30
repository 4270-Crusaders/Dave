#pragma once

#include "utils/geom/math.h"
#include "utils/geom/rotation2d.h"
#include "utils/geom/transform2d.h"
#include "utils/geom/translation2d.h"
#include "utils/geom/twist2d.h"

namespace geom {

/**
 * Pose in 2D: position + heading (WPILib-style). Default units: inches + radians for VEX odometry.
 */
class Pose2d {
public:
	Pose2d() = default;
	Pose2d(Translation2d translation, Rotation2d rotation)
		: m_t(std::move(translation)),
		  m_r(std::move(rotation)) {}

	static Pose2d fromXYTheta(double x, double y, double theta_rad) {
		return {{x, y}, Rotation2d::fromRadians(theta_rad)};
	}

	const Translation2d& translation() const { return m_t; }
	const Rotation2d& rotation() const { return m_r; }
	double x() const { return m_t.x(); }
	double y() const { return m_t.y(); }
	double theta() const { return m_r.radians(); }

	/** Transform robot-relative point into field frame. */
	Translation2d operator*(const Translation2d& robotRel) const {
		return robotRel.rotateBy(m_r) + m_t;
	}

	Pose2d operator+(const Transform2d& o) const {
		return {m_t + o.translation().rotateBy(m_r), m_r + o.rotation()};
	}

	Transform2d operator-(const Pose2d& other) const {
		const Pose2d rel = relativeTo(other);
		return {rel.translation(), rel.rotation()};
	}

	/** This pose expressed relative to `origin` (inverse compose). */
	Pose2d relativeTo(const Pose2d& origin) const {
		const Translation2d t = (m_t - origin.m_t).rotateBy(-origin.m_r);
		return {t, m_r - origin.m_r};
	}

	Twist2d log(const Pose2d& end) const {
		const Pose2d d = end.relativeTo(*this);
		return {d.x(), d.y(), d.theta()};
	}

	bool operator==(const Pose2d& o) const { return m_t == o.m_t && m_r == o.m_r; }

private:
	Translation2d m_t{};
	Rotation2d m_r{};
};

/** Flat storage (x, y, theta rad); same layout as `lemlib::Pose` for quick interop. */
struct FlatPose2d {
	double x = 0;
	double y = 0;
	double theta = 0;
};

inline FlatPose2d toFlat(const Pose2d& p) { return {p.x(), p.y(), p.theta()}; }

inline Pose2d fromFlat(const FlatPose2d& f) { return Pose2d::fromXYTheta(f.x, f.y, f.theta); }

} // namespace geom
