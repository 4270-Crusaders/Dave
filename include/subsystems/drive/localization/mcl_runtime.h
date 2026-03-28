#pragma once

#include "subsystems/drive/drivetrain/types.h"

class Drive;

/** After drive IMU cal when drive_constants::kEnableMcl is true. */
void localization_init_mcl(Drive* drive);

/** Called from Drive::periodic after odometry. */
void localization_tick_mcl(Drive* drive);

/** Weighted mean particle pose (theta radians). */
drivetrain::Pose localization_get_mcl_estimate();
