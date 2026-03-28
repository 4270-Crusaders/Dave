# Command Framework Reference

This document explains **CommandScheduler**, **Trigger**, **EventLoop**, and all **command types** in the **`include/utils/command/`** folder (vendored libcommand), plus **sequences** and **parallel groups**. Put **your** command classes under **`include/utils/commands/`**.

---

## 1. CommandScheduler

The scheduler is a **singleton** that runs every frame (e.g. from `update_loop()` at ~10 ms) and:

1. **Runs subsystem `periodic()`** for every registered subsystem.
2. **Polls event loops** (general + teleop when in opcontrol) so Triggers run.
3. **Runs all scheduled commands**: calls `execute()` on each; if `isFinished()` is true, calls `end(false)` and removes the command, freeing its **requirements** (subsystems).
4. **Re-schedules default commands** for any subsystem that has no command currently using it.

### Key concepts

- **Requirements**: Each command returns `getRequirements()` — a list of subsystems it uses. The scheduler ensures **only one command per subsystem at a time**. Scheduling a new command that needs a subsystem either:
  - **CancelRunning** (default): cancels the current command using that subsystem and runs the new one.
  - **CancelIncoming**: the new command is not scheduled (current keeps running).
- **Default command**: When you `registerSubsystem(subsystem, defaultCommand)`, that command is run whenever that subsystem is not in use by any other command (e.g. arcade drive in teleop).
- **Two event loops**:
  - **Teleop event loop**: polled only when **not** autonomous and **not** disabled (so controller triggers only run in opcontrol).
  - **General event loop**: polled whenever the scheduler runs (e.g. for non-controller triggers).

---

## 2. EventLoop

- **Purpose**: Holds a list of **bindings** (functions) that run **every frame** when the scheduler calls `poll()`.
- **Used by**: Triggers. When you call `primary.getTrigger(DIGITAL_L2)->onTrue(...)`, that trigger binds a lambda to the **teleop** event loop. Each poll, the lambda checks the condition and schedules/cancels commands as needed.
- **Where it runs**:
  - `CommandScheduler::getTeleopEventLoop()` — used by `CommandController::getTrigger()`; only polled in opcontrol.
  - `CommandScheduler::getEventLoop()` — general loop; polled every run.

---

## 3. Trigger

A **Trigger** wraps a **condition** (a `std::function<bool()>`) and an **EventLoop**. When that loop is polled, the trigger checks the condition and reacts to **changes** (or level) by scheduling or canceling commands.

| Method | When it runs | Behavior |
|--------|----------------|----------|
| **onTrue(command)** | Condition goes false → true | Schedules `command` once (e.g. button press). |
| **onFalse(command)** | Condition goes true → false | Schedules `command` once (e.g. button release). |
| **onChange(command)** | Condition changes (either direction) | Schedules `command` once per edge. |
| **whileTrue(command)** | Condition becomes true | Schedules `command`; when condition becomes false, **cancels** it. Good for “hold button = run command”. |
| **whileFalse(command)** | Condition becomes false | Schedules `command`; when condition becomes true, **cancels** it. |
| **toggleOnTrue(command)** | Condition goes false → true | If `command` is scheduled, cancel it; else schedule it. Toggle on button press. |
| **toggleOnFalse(command)** | Condition goes true → false | Same toggle behavior on release. |

**Typical use**: `primary.getTrigger(DIGITAL_L2)->whileTrue(intake->pctCommand(1.0))` — get a trigger from the controller (uses teleop event loop), run intake while the button is held.

---

## 4. Command (base class)

Every command has:

- **initialize()** — called once when the command is scheduled.
- **execute()** — called every frame while the command is running.
- **isFinished()** — when true, the command ends and `end(false)` is called.
- **end(interrupted)** — called when the command stops; `interrupted == true` if it was cancelled by another command.
- **getRequirements()** — list of subsystems this command uses (scheduler uses this for conflict resolution).
- **getCancelBehavior()** — `CancelRunning` (default) or `CancelIncoming`.

Helpers on `Command*`:

- `command->schedule()` / `command->cancel()` / `command->scheduled()`
- `command->andThen(other)` → **SequentialCommandGroup**
- `command->withTimeout(duration)` → **ParallelRaceGroup** with **WaitCommand**
- `command->until(isFinish)` → **ParallelRaceGroup** with **WaitUntilCommand**
- `command->with(other)` → **ParallelCommandGroup**
- `command->race(other)` → **ParallelRaceGroup**
- `command->repeatedly()` → **RepeatCommand**
- `command->asProxy()` → **ProxyCommand** (use sparingly)

---

## 5. Command types (in `utils/command/`)

### 5.1 FunctionalCommand

- **Role**: Build a command from four lambdas: `onInit`, `onExecute`, `onEnd`, `isFinish`, plus requirements.
- **Use when**: You want a one-off command without creating a new class.

### 5.2 RunCommand

- **Extends**: FunctionalCommand.
- **Role**: “Run one function every frame, never finish.”  
  `onInit` and `onEnd` are no-ops; `isFinish` always returns false.
- **Use when**: Default commands (e.g. arcade drive), or any “while this command is active, do X every frame.”

### 5.3 InstantCommand

- **Extends**: FunctionalCommand.
- **Role**: Run `onInit` once, then finish immediately (no execute loop).
- **Use when**: One-shot actions (e.g. fire solenoid, set a state).

### 5.4 WaitCommand

- **Role**: Command with **no requirements** that does nothing and finishes after a **duration** (e.g. `WaitCommand(2_s)`).
- **Use when**: Delays in sequences, or with `withTimeout()` to limit how long another command can run.

### 5.5 WaitUntilCommand

- **Extends**: FunctionalCommand (no requirements).
- **Role**: Does nothing; finishes when a condition `is_finish()` returns true.
- **Use when**: “Run until sensor/condition” (e.g. `command->until([] { return sensor.get(); })`).

### 5.6 ScheduleCommand

- **Extends**: InstantCommand.
- **Role**: On init, **schedules** another command. No requirements.
- **Use when**: You need to start another command from inside a sequence or conditional without holding its requirements (e.g. “do A, then schedule B” so B can use subsystems after A releases them).

### 5.7 RepeatCommand

- **Role**: Wraps one command and **restarts it** every time it finishes (so it never “finishes” from the outside).
- **Use when**: “Run this command over and over until interrupted.”

### 5.8 ConditionalCommand

- **Role**: Picks one of two commands at **initialize()** based on a condition; runs only that one until it finishes.
- **Use when**: “If condition then run A else run B.”

### 5.9 ProxyCommand

- **Role**: Has **no requirements** but schedules another command (from a supplier or a fixed pointer) and is “finished” when that command is no longer scheduled. Used so a **SequentialCommandGroup** can “wait” for a command that runs in the scheduler without the sequence holding the subsystems.
- **Use when**: Only when you must free a subsystem before/after a step in a sequence and have no cleaner design. Use sparingly.

---

## 6. Sequences and parallel groups

### 6.1 SequentialCommandGroup

- **Class**: `SequentialCommandGroup`.
- **Behavior**: Runs a list of commands **one after another**. When the current command’s `isFinished()` is true, it ends that command and starts the next; the group finishes when the last command finishes.
- **Creation**:
  - `new SequentialCommandGroup({cmd1, cmd2, cmd3})`
  - Or: `cmd1->andThen(cmd2)` (then chain or use with more commands).
- **Requirements**: Union of all sub-commands’ requirements (so the group “holds” all of them until the whole sequence ends).

### 6.2 ParallelCommandGroup

- **Class**: `ParallelCommandGroup`.
- **Behavior**: Runs **all** commands **at the same time**. Finishes when **every** command has finished. No two sub-commands may share a requirement (assert).
- **Creation**:
  - `new ParallelCommandGroup({cmd1, cmd2, cmd3})`
  - Or: `cmd1->with(cmd2)`.
- **Use when**: “Do A and B and C together” (e.g. intake + lever + match loader in one binding).

### 6.3 ParallelRaceGroup

- **Class**: `ParallelRaceGroup`.
- **Behavior**: Runs all commands at once; finishes as soon as **any** one command finishes; then ends the others (with `end(true)`). No two sub-commands may share a requirement.
- **Creation**:
  - `new ParallelRaceGroup({cmd1, cmd2})`
  - Or: `cmd1->race(cmd2)`.
- **Use when**:
  - **Time limit**: `command->withTimeout(3_s)` → race between `command` and `WaitCommand(3_s)`; when the timer wins, the other command is cancelled.
  - **Until condition**: `command->until([] { return condition(); })` → race between `command` and `WaitUntilCommand`; when the condition becomes true, the main command is cancelled.

---

## 7. Quick reference

| Goal | Use |
|-----|-----|
| Run one function every frame (e.g. default drive) | **RunCommand** |
| Do one action once | **InstantCommand** |
| Wait for time | **WaitCommand** or `command->withTimeout(duration)` |
| Wait for condition | **WaitUntilCommand** or `command->until(condition)` |
| Do A then B then C | **SequentialCommandGroup** or `cmdA->andThen(cmdB)` |
| Do A and B and C together | **ParallelCommandGroup** or `cmdA->with(cmdB)` |
| Do A or B, first to finish wins | **ParallelRaceGroup** or `cmdA->race(cmdB)` |
| Run command in a loop until interrupted | **RepeatCommand** or `command->repeatedly()` |
| If condition then A else B | **ConditionalCommand** |
| Schedule another command from inside a command | **ScheduleCommand** |
| Button press → run once | Trigger **onTrue** / **onFalse** |
| Button held → run; release → stop | Trigger **whileTrue** / **whileFalse** |
| Button press → toggle command | Trigger **toggleOnTrue** / **toggleOnFalse** |

---

## 8. How your main.cpp fits in

- **initialize()**: Registers subsystems with default commands (e.g. `drive->arcadeCommand(&primary)`). Triggers are set up with `primary.getTrigger(...)->onTrue(...)` etc.; those bind to the **teleop** event loop.
- **update_loop()**: Calls `CommandScheduler::run()` every 10 ms. That runs subsystem periodics, polls event loops (teleop only when in opcontrol), runs scheduled commands, and re-schedules default commands for idle subsystems.
- **opcontrol()**: Can be a simple delay loop; drive and other behaviors run via the scheduler and triggers, so you don’t need to call `chassis.arcade()` or intake logic directly there.

This gives you a single place (scheduler + triggers) where all command-based behavior is coordinated and subsystem conflicts are handled.
