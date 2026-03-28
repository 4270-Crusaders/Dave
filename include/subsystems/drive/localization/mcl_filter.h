#pragma once

#include "api.h"
#include "subsystems/drive/drivetrain/math.h"
#include "subsystems/drive/drivetrain/types.h"
#include "subsystems/drive/localization/axis_aligned_raycast.h"
#include "subsystems/drive/DriveConstants.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace localization {

struct Particle {
	double x = 0;
	double y = 0;
	double theta = 0;
	double weight = 1.0;
};

namespace mcl_detail {
inline constexpr double kPi = 3.14159265358979323846;

inline double wrapPi(double a) {
	while (a <= -kPi) {
		a += 2.0 * kPi;
	}
	while (a > kPi) {
		a -= 2.0 * kPi;
	}
	return a;
}

inline double mmToIn(std::int32_t mm) {
	if (mm <= 0 || mm >= drive_constants::kDistanceInvalidMm) {
		return -1.0;
	}
	return static_cast<double>(mm) / 25.4;
}

/** Log Gaussian N(z | predicted, sigma) up to an additive constant: -0.5 * ((z-predicted)/sigma)^2 */
inline double distanceLogLikelihood(double predicted_in, double measured_in, double sigma_in, double outlier_max_in,
                                    double outlier_log_penalty) {
	const double diff = predicted_in - measured_in;
	if (std::abs(diff) > outlier_max_in) {
		return -outlier_log_penalty;
	}
	const double inv2s2 = 1.0 / (2.0 * sigma_in * sigma_in);
	return -diff * diff * inv2s2;
}

inline void sensorWorldPosition(const Particle& p, const drive_constants::DistanceSensorMount& mount, double& wx, double& wy) {
	wx = p.x + mount.offset_x_in * std::cos(p.theta) - mount.offset_y_in * std::sin(p.theta);
	wy = p.y + mount.offset_x_in * std::sin(p.theta) + mount.offset_y_in * std::cos(p.theta);
}
} // namespace mcl_detail

/**
 * Monte Carlo localization (particle filter) for a rectangular field using VEX Distance sensors.
 *
 * Design notes (see also [Aadish Verma on MCL](https://www.aadishv.dev/mcl), [resampling (SUR)](https://www.aadishv.dev/mcl-2x)):
 * - Predict: apply odometry delta + Gaussian process noise per particle (accounts for slip / integration error).
 * - Update: multiply likelihoods across sensors in one pass using log-weights and a shared max for numerical stability.
 * - Outliers: large |predicted − measured| are down-weighted sharply to limit “teleport” updates from obstacles.
 * - Resample: stochastic universal resampling (SUR) in O(N) with a linear CDF walk.
 * - Estimate: weighted mean for x,y; [circular mean](https://www.aadishv.dev/mcl) for θ via atan2(Σ w sin θ, Σ w cos θ).
 */
class MclFilter {
public:
	explicit MclFilter(double min_x = drive_constants::kFieldMinX, double min_y = drive_constants::kFieldMinY,
	                   double max_x = drive_constants::kFieldMaxX, double max_y = drive_constants::kFieldMaxY)
		: min_x_(min_x),
		  min_y_(min_y),
		  max_x_(max_x),
		  max_y_(max_y),
		  rng_(static_cast<std::mt19937::result_type>(pros::millis())) {
		particles_.resize(drive_constants::kMclParticleCount);
		particles_next_.resize(drive_constants::kMclParticleCount);
		log_w_scratch_.resize(drive_constants::kMclParticleCount);
		cdf_scratch_.resize(drive_constants::kMclParticleCount);
	}

	void setFieldBounds(double min_x, double min_y, double max_x, double max_y) {
		min_x_ = min_x;
		min_y_ = min_y;
		max_x_ = max_x;
		max_y_ = max_y;
	}

	void resetUniform() {
		std::uniform_real_distribution<double> ux(min_x_, max_x_);
		std::uniform_real_distribution<double> uy(min_y_, max_y_);
		std::uniform_real_distribution<double> ut(-mcl_detail::kPi, mcl_detail::kPi);
		const double w = 1.0 / static_cast<double>(particles_.size());
		for (auto& p : particles_) {
			p.x = ux(rng_);
			p.y = uy(rng_);
			p.theta = ut(rng_);
			p.weight = w;
		}
	}

	void resetAround(double x, double y, double theta_rad, double pos_spread_in, double theta_spread_rad) {
		std::normal_distribution<double> nx(x, pos_spread_in);
		std::normal_distribution<double> ny(y, pos_spread_in);
		std::normal_distribution<double> nt(theta_rad, theta_spread_rad);
		const double w = 1.0 / static_cast<double>(particles_.size());
		for (auto& p : particles_) {
			p.x = drivetrain::clampd(nx(rng_), min_x_ + 1.0, max_x_ - 1.0);
			p.y = drivetrain::clampd(ny(rng_), min_y_ + 1.0, max_y_ - 1.0);
			p.theta = mcl_detail::wrapPi(nt(rng_));
			p.weight = w;
		}
	}

	void predictFromOdom(const drivetrain::Pose& prev_pose, const drivetrain::Pose& new_pose) {
		const double dx = new_pose.x - prev_pose.x;
		const double dy = new_pose.y - prev_pose.y;
		const double dtheta = mcl_detail::wrapPi(new_pose.theta - prev_pose.theta);
		for (auto& p : particles_) {
			p.x += dx + drawGaussian(rng_, drive_constants::kMclSigmaX);
			p.y += dy + drawGaussian(rng_, drive_constants::kMclSigmaY);
			p.theta = mcl_detail::wrapPi(p.theta + dtheta + drawGaussian(rng_, drive_constants::kMclSigmaTheta));
			p.x = drivetrain::clampd(p.x, min_x_ + 0.5, max_x_ - 0.5);
			p.y = drivetrain::clampd(p.y, min_y_ + 0.5, max_y_ - 0.5);
		}
	}

	bool updateDistance(std::int32_t dist_mm, const drive_constants::DistanceSensorMount& mount) {
		const double z = mcl_detail::mmToIn(dist_mm);
		if (z < 0.0) {
			return false;
		}
		const double sig = drive_constants::kMclSigmaMeasureIn;
		for (auto& p : particles_) {
			double wx = 0;
			double wy = 0;
			mcl_detail::sensorWorldPosition(p, mount, wx, wy);
			const double ray = mcl_detail::wrapPi(p.theta + mount.bearing_rad);
			const double predicted =
				raycastAxisAlignedRectangle(wx, wy, ray, min_x_, min_y_, max_x_, max_y_);
			const double ll = mcl_detail::distanceLogLikelihood(predicted, z, sig, drive_constants::kMclOutlierMaxIn,
			                                                     drive_constants::kMclOutlierLogPenalty);
			p.weight *= std::exp(ll);
		}
		normalizeWeights();
		return true;
	}

	/** Batched measurement update: all sensors in one pass, log-domain for stability (recommended each frame). */
	void updateAllDistances(const std::int32_t* dist_mm, std::size_t count) {
		const std::size_t nmount = drive_constants::kMclDistanceMounts.size();
		const std::size_t nsens = std::min(count, nmount);
		if (nsens == 0 || particles_.empty()) {
			return;
		}

		std::array<double, drive_constants::kMclDistanceMounts.size()> z_in{};
		z_in.fill(-1.0);
		for (std::size_t s = 0; s < nsens; ++s) {
			z_in[s] = mcl_detail::mmToIn(dist_mm[s]);
		}

		const double sig = drive_constants::kMclSigmaMeasureIn;
		double max_log = -1e300;

		for (std::size_t i = 0; i < particles_.size(); ++i) {
			const Particle& p = particles_[i];
			double lw = std::log(std::max(p.weight, 1e-300));
			for (std::size_t s = 0; s < nsens; ++s) {
				if (z_in[s] < 0.0) {
					continue;
				}
				double wx = 0;
				double wy = 0;
				mcl_detail::sensorWorldPosition(p, drive_constants::kMclDistanceMounts[s], wx, wy);
				const double ray = mcl_detail::wrapPi(p.theta + drive_constants::kMclDistanceMounts[s].bearing_rad);
				const double pred =
					raycastAxisAlignedRectangle(wx, wy, ray, min_x_, min_y_, max_x_, max_y_);
				lw += mcl_detail::distanceLogLikelihood(pred, z_in[s], sig, drive_constants::kMclOutlierMaxIn,
				                                         drive_constants::kMclOutlierLogPenalty);
			}
			log_w_scratch_[i] = lw;
			max_log = std::max(max_log, lw);
		}

		for (std::size_t i = 0; i < particles_.size(); ++i) {
			particles_[i].weight = std::exp(log_w_scratch_[i] - max_log);
		}
		normalizeWeights();
	}

	double effectiveSampleSize() const {
		double sumsq = 0.0;
		for (const auto& p : particles_) {
			sumsq += p.weight * p.weight;
		}
		return sumsq > 1e-100 ? 1.0 / sumsq : 0.0;
	}

	void resampleIfNeeded() {
		const std::size_t n = particles_.size();
		if (n == 0) {
			return;
		}
		const double ess = effectiveSampleSize();
		const double thresh = drive_constants::kMclResampleEssFrac * static_cast<double>(n);
		if (ess >= thresh) {
			return;
		}

		double* cdf = cdf_scratch_.data();
		cdf[0] = particles_[0].weight;
		for (std::size_t i = 1; i < n; ++i) {
			cdf[i] = cdf[i - 1] + particles_[i].weight;
		}

		std::uniform_real_distribution<double> uni(0.0, 1.0 / static_cast<double>(n));
		const double r0 = uni(rng_);

		particles_next_.resize(n);
		std::size_t j = 0;
		for (std::size_t i = 0; i < n; ++i) {
			const double u = r0 + static_cast<double>(i) / static_cast<double>(n);
			while (j + 1 < n && cdf[j] < u) {
				++j;
			}
			particles_next_[i] = particles_[j];
		}

		const double w = 1.0 / static_cast<double>(n);
		const double pj = drive_constants::kMclResamplePosJitterIn;
		const double tj = drive_constants::kMclResampleThetaJitterRad;
		if (pj > 1e-12 || tj > 1e-12) {
			std::normal_distribution<double> noise_x(0.0, pj);
			std::normal_distribution<double> noise_y(0.0, pj);
			std::normal_distribution<double> noise_t(0.0, tj);
			for (auto& p : particles_next_) {
				if (pj > 1e-12) {
					p.x = drivetrain::clampd(p.x + noise_x(rng_), min_x_ + 0.5, max_x_ - 0.5);
					p.y = drivetrain::clampd(p.y + noise_y(rng_), min_y_ + 0.5, max_y_ - 0.5);
				}
				if (tj > 1e-12) {
					p.theta = mcl_detail::wrapPi(p.theta + noise_t(rng_));
				}
				p.weight = w;
			}
		} else {
			for (auto& p : particles_next_) {
				p.weight = w;
			}
		}

		particles_.swap(particles_next_);
	}

	drivetrain::Pose estimateMean() const {
		double sx = 0, sy = 0, sc = 0, ss = 0;
		for (const auto& p : particles_) {
			sx += p.weight * p.x;
			sy += p.weight * p.y;
			sc += p.weight * std::cos(p.theta);
			ss += p.weight * std::sin(p.theta);
		}
		return {sx, sy, std::atan2(ss, sc)};
	}

	const std::vector<Particle>& particles() const { return particles_; }

private:
	static double drawGaussian(std::mt19937& gen, double sigma) {
		std::normal_distribution<double> d(0.0, sigma);
		return d(gen);
	}

	void normalizeWeights() {
		double sum = 0.0;
		for (const auto& p : particles_) {
			sum += p.weight;
		}
		if (sum <= 1e-100) {
			const double u = 1.0 / static_cast<double>(particles_.size());
			for (auto& p : particles_) {
				p.weight = u;
			}
			return;
		}
		for (auto& p : particles_) {
			p.weight /= sum;
		}
	}

	double min_x_;
	double min_y_;
	double max_x_;
	double max_y_;
	std::vector<Particle> particles_;
	std::vector<Particle> particles_next_;
	std::vector<double> log_w_scratch_;
	std::vector<double> cdf_scratch_;
	std::mt19937 rng_;
};

} // namespace localization
