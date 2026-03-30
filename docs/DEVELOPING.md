# Developing with this template

This project is a **PROS 3** C++ template for VEX V5 robots. It is organized around **command-based programming** (like WPILib on FRC): you register **subsystems**, schedule **commands**, and let a **scheduler** run them on a fixed loop. The drivetrain is **[LemLib](https://github.com/LemLib/LemLib)** behind a thin **`Drive`** wrapper (`include/subsystems/drive/Drive.h`).

Use this document in order, or jump to the section that matches your level.

---

## Table of contents

1. [Concepts (everyone should read this)](#1-concepts-everyone-should-read-this)
2. [Absolute beginners: environment and first run](#2-absolute-beginners-environment-and-first-run)
3. [Beginners: where to edit the robot](#3-beginners-where-to-edit-the-robot)
4. [Intermediate: commands, triggers, and new subsystems](#4-intermediate-commands-triggers-and-new-subsystems)
5. [Intermediate: autonomous routines](#5-intermediate-autonomous-routines)
6. [Advanced: drivetrain, geometry, and localization](#6-advanced-drivetrain-geometry-and-localization)
7. [Project layout reference](#7-project-layout-reference)
8. [Troubleshooting and glossary](#8-troubleshooting-and-glossary)

---

## 1. Concepts (everyone should read this)

### PROS

[PROS](https://pros.cs.purdue.edu/) is the toolchain (editor integration, build, upload) and API for talking to the V5 brain, motors, sensors, and controller.

### Competition modes

| Mode | Function | Typical use |
|------|-----------|-------------|
| `initialize()` | Runs once at program start | Create subsystems, register scheduler, calibrate IMU |
| `disabled()` | While robot disabled | Optional cleanup or LCD |
| `competition_initialize()` | After init, before match | Selector, last-minute setup |
| `autonomous()` | Auton period | Run scripted commands / paths |
| `opcontrol()` | Driver control | Often empty if everything is command + triggers |

This template runs **`CommandScheduler::run()`** from a dedicated task on a **10 ms** loop (`src/main.cpp`). Your subsystems’ `periodic()` methods and active commands are driven from there.

### Command-based programming

- **Subsystem** — A robot mechanism (or logical group) with a `periodic()` hook. Example: `Drive`, `ExampleVelocityRoller`.
- **Command** — A unit of behavior with a lifecycle: initialize, execute, end, and “is finished?” Scheduling commands instead of putting everything in `opcontrol()` keeps code composable and testable.
- **Default command** — When nothing else requires a subsystem, the scheduler runs this command. Here, the drive default is **arcade** from the primary controller.
- **Trigger** — Binds controller buttons to commands (e.g. “while held, run intake”).

The command framework lives in **`include/utils/command/`** (header-only). `main.h` pulls in **`utils/command/includes.h`**.

### This template’s stack (folders)

- **`include/subsystems/`** — One folder per mechanism. **`drive/`** holds **Drive**, **DriveConstants**, **math.h**, and **driver/** (teleop shaping). Examples: **`example_roller/`**, **`example_position_arm/`**, **`example_pneumatic/`** each with `*Subsystem*.h` + `*Constants.h`. Root **`subsystems.h`** lists includes.
- **`include/commands/`** — FRC-style command factories, e.g. **`DriveCommands.h`** (`DriveCommands::moveToPointCommand`, default teleop commands, …).
- **`RobotConstants.h`** — Project-wide toggles (e.g. `robot::kEnableExampleSubsystems`).
- **`include/utils/`** — **`geom/`**, **`command/`** (scheduler + command types), **`control/`**; **`utils.h`** pulls geom + drivetrain math shims.
- **`include/units/`** — [LemLib units](https://github.com/LemLib/units) (`units/units.hpp`).

### What to edit day to day

1. **`include/subsystems/<mechanism>/`** — Subsystem class + constants for that mechanism.
2. **`include/commands/`** — Command groups/factories for auton and teleop (mirror WPILib / FRC team layout).

Vendor/kernel trees (`pros/`, `liblvgl/`, `robodash/`) are not team code.

---

## 2. Absolute beginners: environment and first run

### Prerequisites

- Install **PROS** (CLI and editor extension as you prefer).
- Basic **C++** helps: classes, headers (`#include`), and `namespace`.

Official docs: [PROS Getting Started](https://pros.cs.purdue.edu/v5/getting-started/index.html).

### Build and upload

1. Open the project folder in your PROS environment.
2. Build the project (PROS: build / `pros make`).
3. Connect the V5 brain via USB and upload.

If the build fails, read the compiler error: it usually names the file and line (missing include, typo, wrong port type).

### Safety

- **Lift the robot** or use wheel chocks when testing drivetrain code.
- Verify **motor ports and reversed ports** (`-` in PROS means reversed) in hardware and in **`include/subsystems/drive/DriveConstants.h`** (and IMU / tracking wheel ports there).

---

## 3. Beginners: where to edit the robot

### Step 1: constants files

- **`include/RobotConstants.h`** — `robot::kEnableExampleSubsystems` and other project-wide flags.
- **`include/subsystems/drive/DriveConstants.h`** — LemLib chassis hardware (ports, wheel size, tracking offsets) and PID / teleop tuning.
- **Per-example constants** — e.g. `example_roller_constants::` in `subsystems/example_roller/ExampleVelocityRollerConstants.h`.

### Step 2: `src/main.cpp`

- **`initialize()`** — Subsystems are constructed and registered here. The drive is always created; examples are behind `if constexpr (robot::kEnableExampleSubsystems)`.
- **Default command** — `CommandScheduler::registerSubsystem(chassis, DriveCommands::arcadeDefaultCommand(chassis, &primary));` keeps arcade drive running in teleop unless another command takes over the drive (if you add that later).
- **`opcontrol()`** — Empty on purpose; teleop is driven by **default commands** and **triggers** you add in `initialize()`.

### Step 3: Try a tiny change

In `initialize()`, after subsystems are registered, you can experiment with a **trigger** (see [Intermediate](#4-intermediate-commands-triggers-and-new-subsystems)) or temporarily change the default drive command to **tank** using `DriveCommands::tankDefaultCommand(chassis, &primary)`.

---

## 4. Intermediate: commands, triggers, and new subsystems

### Included patterns

From `utils/command/includes.h` you get types such as:

- `RunCommand`, `InstantCommand`, `FunctionalCommand`
- `ParallelCommandGroup`, `Sequence`
- `Trigger` (via `CommandController`)

`src/main.cpp` already has `CommandController primary(pros::E_CONTROLLER_MASTER);`.

### Example: button runs a command

Conceptually:

1. Get a trigger from `primary.getTrigger(DIGITAL_R1)` (button names come from PROS simple names in `main.h`).
2. Call `onTrue` / `onFalse` with a `Command*` (often `new InstantCommand(...)` or a subsystem helper that returns a `Command*`).

Study **`Drive`** in `include/subsystems/drive/Drive.h` (behavior) and **`include/commands/DriveCommands.h`** (command factories for auton / default teleop).

### Adding your own subsystem (recommended workflow)

1. **Create a folder** `include/subsystems/<name>/` with **`<Name>.h`** (subsystem class) and **`<Name>Constants.h`** (ports, PID, etc.).
2. **Subclass `Subsystem`**, implement `periodic()` (even if empty at first).
3. **Add commands** in **`include/commands/<Name>Commands.h`** (FRC style) or small `RunCommand` helpers on the class for simple cases.
4. **`#include` your subsystem header** from `include/subsystems/subsystems.h` so `main.h` pulls it in.
5. In **`src/main.cpp`**: construct and `registerSubsystem(..., defaultCommand)`.

The **example_*** folders under `include/subsystems/` are short, copy-friendly references.

### Adding your own commands

Use **`include/commands/`** for factory functions and command groups (see **`DriveCommands.h`**). Base types come from **`utils/command/`**.

---

## 5. Intermediate: autonomous routines

### Entry point

- Implement **`autonomous()`** in **`src/main.cpp`** (or add a separate `.cpp` and call it from `autonomous()`).
- Use **`DriveCommands::`** factories (e.g. `moveToPoseCommand(chassis, ...)`) with **`Sequence`** / **`ParallelCommandGroup`** from **`include/utils/command/`**.

### Using the drive in auton

`Drive` is available as the global **`chassis`** pointer from `main.cpp`.

Typical flow:

1. **`chassis->setPose(x, y, theta)`** — Seed pose (inches and degrees by default; overloads exist).
2. **Motion calls** — e.g. `moveToPoint`, `moveToPose`, `moveToPoseBoomerang`, `turnToHeading`, or **`follow`** with a path **asset** (see below).
3. **`waitUntilDone()`** — Block until the current motion finishes (when using async motion APIs carefully).
4. **Command-based auton** — Schedule `DriveCommands::moveToPoseCommand(chassis, ...)` and compose with `Sequence` / `ParallelCommandGroup` for structured routines.

**Timeouts** are in **milliseconds** (PROS convention). Full motion API: **`lemlib::Chassis`** via **`chassis->chassis()`** or the **`Drive`** shortcuts in `include/subsystems/drive/Drive.h`.

### Paths (LemLib pure pursuit)

LemLib follows **static path files**, not `std::vector` waypoints. Put a path text file in **`static/`**, declare it with **`ASSET(myPath_txt)`** (dots → underscores), then call **`chassis->follow(myPath_txt, lookaheadIn, timeoutMs)`**. See [LemLib path following](https://lemlib.readthedocs.io/en/stable/tutorials/7_pure_pursuit.html).

---

## 6. Advanced: LemLib, geometry, localization

### LemLib in this project

- **Sources**: `include/lemlib/`, `src/lemlib/`, bundled **`include/fmt/`**.
- **Configuration**: `include/subsystems/drive/DriveConstants.h` (ports, wheel/track dimensions, tracking wheel offsets, linear/angular controller gains, Expo curve inputs).
- **Wrapper**: `Drive` constructs `pros::MotorGroup`, IMU, rotation sensors, `lemlib::TrackingWheel`, `lemlib::Drivetrain`, `lemlib::OdomSensors`, and `lemlib::Chassis` in `src/subsystems/drive/Drive.cpp`.

`Drive::periodic()` is a no-op; LemLib runs its own background tasks after **`chassis.calibrate()`**.

Auton/teleop **commands** live in **`include/commands/DriveCommands.h`**.

### `utils/geom/`

Header-only 2D/3D math types (`Pose2d`, `Rotation2d`, `Transform2d`, etc.). Use these for readable geometry instead of raw `x,y,theta` when modeling mechanisms or vision offsets.

Umbrella include: **`utils/geom/geom.h`**.

### Localization beyond wheel + IMU odometry

The previous template’s **Monte Carlo localization (MCL)** stack was removed when switching to LemLib. For particle-filter style localization on VEX, see community projects such as [u-k-g/monte-carlo-localization](https://github.com/u-k-g/monte-carlo-localization) and integrate against **`chassis->getPose()`** as your odometry prior.

---

## 7. Project layout reference

| Path | Role |
|------|------|
| `src/main.cpp` | Competition entrypoints, scheduler, `autonomous()`, subsystem registration |
| `include/main.h` | PROS, `RobotConstants.h`, `subsystems/subsystems.h`, `commands/DriveCommands.h`, utils |
| `include/subsystems/` | One folder per mechanism; **`drive/`** = LemLib wrapper + constants; **`subsystems.h`** umbrella |
| `include/lemlib/`, `src/lemlib/` | LemLib library (vendored) |
| `include/commands/` | **`DriveCommands.h`** and future `*Commands.h` files |
| `include/RobotConstants.h` | Project-wide feature flags |
| `include/utils/command/` | Command scheduler + command types (header-only) |
| `include/utils/geom/` | Geometry types |
| `include/utils/control/` | Shared control helpers (e.g. PID) |
| `include/utils/utils.h` | Shortcut: geom + drivetrain math |
| `include/units/` | LemLib QUnits |
| `project.pros` | PROS project metadata |

---

## 8. Troubleshooting and glossary

### Common issues

| Symptom | Things to check |
|---------|------------------|
| Build errors after clone | PROS kernel version, run `pros conduct fetch` / apply template if prompted |
| Robot drives wrong direction | Reversed ports in `ChassisConfig`, joystick axis sign |
| Odometry drifts | Wheel diameter, track width, IMU calibration, wheel slippage |
| MCL diverges | Field bounds, sensor mounts/bearings, `kMclSigmaMeasureIn` too low, tighten `kMclOutlierMaxIn`, enable slight resample jitter |
| Scheduler “does nothing” | Ensure `pros::Task commandSchedulerTask(update_loop);` runs and `CommandScheduler::registerSubsystem` was called |

### Glossary

| Term | Meaning |
|------|---------|
| **PROS** | VEX V5 C/C++ development environment and API |
| **Subsystem** | Registered robot module with `periodic()` |
| **Command** | Scheduled behavior with start/end/finish semantics |
| **Default command** | Runs when no other command needs that subsystem |
| **MCL** | Monte Carlo localization (particle filter) |
| **Odometry** | Estimating pose from encoders + heading |
| **Lookahead** | Pure-pursuit style path follower parameter (distance along path the robot “chases”) |

---

## Further reading

- [PROS documentation](https://pros.cs.purdue.edu/v5/index.html)
- [WPILib command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) (concepts transfer well)
- Example of minimal subsystem sources in another open project: [Echo `src/subsystems`](https://github.com/alexDickhans/echo/tree/worlds-day-1/src/subsystems) (this template keeps more logic in headers like Echo’s include-only style for teaching)

If you improve this template for your team, consider appending a short **“Team overrides”** section at the bottom of this file (alliance color, auton selector, calibration checklist) so new programmers know where your custom conventions live.
