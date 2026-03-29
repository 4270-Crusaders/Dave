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

### Key gotcha: `chassis.inl.h` namespace

`include/subsystems/drive/drivetrain/detail/chassis.inl.h` is included **outside** `namespace drivetrain` in `chassis.h`. The inline member definitions use fully-qualified `drivetrain::Chassis::` names. Moving the include back inside the namespace will break the build because the anonymous helper namespace and standard library headers conflict with the `drivetrain` namespace.

### Missing `include/units/` symlink

The project expects `include/units/units.hpp` but the file lives at `include/utils/units/units.hpp`. A symlink `include/units -> utils/units` is needed. The update script creates it if absent.
