#pragma once

#include "utils/control/PidController.h"

namespace drivetrain {

/** Chassis motion PIDs — same implementation as `control::PidController`. */
using Pid = control::PidController;

} // namespace drivetrain
