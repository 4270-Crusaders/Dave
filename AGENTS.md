# AGENTS.md

## Cursor Cloud specific instructions

This is a **VEX V5 robotics firmware** project (PROS 3, C++23, cross-compiled for ARM Cortex-A9). There are no web services, databases, or containers.

### Toolchain

- **ARM GNU Toolchain 14.3.rel1** is installed at `/opt/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/`.
- **PROS CLI** (`pros`) is installed via pip at `~/.local/bin/pros`.
- Both are on PATH via `~/.bashrc`. If a new shell doesn't find `arm-none-eabi-gcc` or `pros`, run `source ~/.bashrc`.

### Build

```
make            # Build firmware (hot.package.bin + cold.package.bin)
make clean      # Remove build artifacts
```

Output goes to `bin/`. The project cannot be *run* locally — it targets physical VEX V5 Brain hardware. A successful build is the primary verification.

### Lint / Test

There are no automated tests or linters configured. The build itself (`make`) is the main correctness check. `pros make` is equivalent to `make`.

### Drivetrain (LemLib)

Motion and odometry use **[LemLib](https://github.com/LemLib/LemLib)** (`include/lemlib/`, `src/lemlib/`). Tune ports and tracking offsets in `include/subsystems/drive/DriveConstants.h`; wrapper API is `Drive` in `include/subsystems/drive/Drive.h`.

### Units (LemLib)

Headers from [LemLib/units](https://github.com/LemLib/units) live under `include/units/` (e.g. `units/units.hpp`). Use `pros c fetch` / apply the matching **liblvgl@8.3.8** template so `firmware/liblvgl.a` exists next to the vendored `include/liblvgl` tree ([purduesigbots/liblvgl](https://github.com/purduesigbots/liblvgl)). LemLib also vendors **`{fmt}`** under `include/fmt/` for logging/formatting inside the library.
