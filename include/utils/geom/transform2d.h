#pragma once

#include "utils/geom/rotation2d.h"
#include "utils/geom/translation2d.h"

namespace geom {

/**
 * Rigid transform in 2D: first rotate by m_rotation, then translate by m_translation
 * applied in the rotated frame (matches common FRC Transform2d usage for odometry deltas).
 */
class Transform2d {
public:
	Transform2d() = default;
	Transform2d(Translation2d translation, Rotation2d rotation)
		: m_translation(std::move(translation)),
		  m_rotation(std::move(rotation)) {}

	const Translation2d& translation() const { return m_translation; }
	const Rotation2d& rotation() const { return m_rotation; }

	/** Apply transform to a point (vector from origin). */
	Translation2d operator*(const Translation2d& p) const {
		return p.rotateBy(m_rotation) + m_translation;
	}

	/** Compose: this * other (apply other first, then this). */
	Transform2d operator*(const Transform2d& o) const {
		return {m_translation + o.translation().rotateBy(m_rotation), m_rotation + o.rotation()};
	}

	Transform2d inverse() const {
		const Rotation2d invRot = -m_rotation;
		return {(-m_translation).rotateBy(invRot), invRot};
	}

private:
	Translation2d m_translation{};
	Rotation2d m_rotation{};
};

} // namespace geom
