<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

`bb_estimator_ahrs` ships three 6-DOF IMU fusion algorithms — Madgwick,
Mahony, and Complementary — as `BB.Estimator` implementations for the
[Beam Bots](https://github.com/beam-bots/bb) framework. Ported from Gus
Workman's [`gworkman/ahrs`](https://github.com/gworkman/ahrs).

## Architecture

```
lib/bb/estimator/ahrs/
├── madgwick.ex         # BB.Estimator, gradient-descent filter
├── mahony.ex           # BB.Estimator, PI controller filter
├── complementary.ex    # BB.Estimator, high-pass gyro + low-pass tilt
├── quaternion.ex       # Scalar (w,x,y,z) quaternion, INTERNAL
└── math.ex             # Euler / tilt / vector-rotation helpers
```

Each algorithm module:

- `use BB.Estimator` with an `options_schema` for its tuning parameters.
- Consumes `BB.Message.Sensor.Imu` on `handle_input/2`, decomposes
  `BB.Math.Vec3` inputs into scalar tuples, runs a pure `step/4`
  function, and emits a new `Imu` message with the fused orientation
  on `:out`.
- Tracks `dt` from `message.monotonic_time` — the wall clock is never
  read.

The internal `BB.Estimator.Ahrs.Quaternion` is a scalar 4-float struct, deliberately
kept separate from `BB.Math.Quaternion` (Nx-backed) — Nx dispatch
overhead is prohibitive at AHRS update rates of 100–1000 Hz.
Conversion to `BB.Math.Quaternion` happens only at the message
boundary via `BB.Estimator.Ahrs.Quaternion.to_bb/1`.

## Build and Test Commands

```bash
mix check --no-retry    # All checks (compile, test, format, credo, dialyzer, reuse)
mix test                # Run tests
mix test path/to/test.exs:42  # Single test at line
mix format              # Format
mix credo --strict      # Linting
```

`ex_check` is the canonical entry point — always prefer `mix check --no-retry`
over running individual tools.

For cross-repo development against a local `bb` checkout:

```bash
BB_VERSION=local mix deps.get
BB_VERSION=local mix test
```

## Key Patterns

### Units

All inputs and outputs are SI: rad/s for angular velocity, m/s² for
linear acceleration, radians for orientation. **Unit conversion is the
sensor driver's responsibility**, not the AHRS algorithm's. The
upstream Gus library accepted multiple unit types and converted; that
machinery has been removed.

### `accel_threshold`

Each algorithm accepts `:accel_threshold` (default `0.1`) as a
fractional deviation from 1 g. When `|accel| / g` falls outside
`[1 - threshold, 1 + threshold]`, the correction term is suppressed for
that step and the filter falls back to gyro-only integration. Standard
mitigation for sustained linear acceleration.

### Pure `step/4` for testing

Each algorithm exposes a public `step/4` that takes the algorithm
state, `{gx, gy, gz}` rad/s, `{ax, ay, az}` m/s², and `dt` seconds, and
returns the updated state. Use this directly in tests rather than
mocking the BB.Estimator envelope.

## Dependencies

- `bb` — core framework (provides `BB.Estimator`, `BB.Math.*`,
  `BB.Message.Sensor.Imu`). Switch via `BB_VERSION=local|main|<version>`.

## Origins and Licensing

Mixed licence: Gus Workman's original code is MIT, the
`BB.Estimator` wrappers and project scaffolding are Apache-2.0. See
per-file SPDX headers and the `LICENSES/` directory. REUSE-compliant.

## When Making Changes

1. Keep the algorithm internals scalar-float. Don't drop into Nx for
   per-step math.
2. Don't add unit conversion to the algorithm modules — push it into
   the publishing sensor.
3. Preserve `accel_threshold` semantics — many real-world traces rely
   on its rejection behaviour.
4. Run `mix check --no-retry` before pushing.
