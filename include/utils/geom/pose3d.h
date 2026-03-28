#pragma once

#include "utils/geom/rotation3d.h"
#include "utils/geom/transform3d.h"
#include "utils/geom/translation3d.h"

namespace geom {

class Pose3d {
public:
	Pose3d() = default;
	Pose3d(Translation3d t, Rotation3d r) : m_t(std::move(t)),
	                                         m_r(std::move(r)) {}

	const Translation3d& translation() const { return m_t; }
	const Rotation3d& rotation() const { return m_r; }

	Translation3d operator*(const Translation3d& body) const { return m_r.rotate(body) + m_t; }

	Pose3d operator+(const Transform3d& o) const { return {m_t + m_r.rotate(o.translation()), m_r * o.rotation()}; }

	Pose3d relativeTo(const Pose3d& o) const {
		const Translation3d t = o.rotation().inverse().rotate(m_t - o.m_t);
		return {t, o.rotation().inverse() * m_r};
	}

private:
	Translation3d m_t{};
	Rotation3d m_r{};
};

} // namespace geom
