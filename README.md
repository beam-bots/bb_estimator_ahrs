<!--
SPDX-FileCopyrightText: 2026 Gus Workman
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# bb_ahrs

![Build Status](https://github.com/beam-bots/bb_ahrs/actions/workflows/ci.yml/badge.svg)
[![License: Apache-2.0 / MIT](https://img.shields.io/badge/License-Apache--2.0%20%2F%20MIT-green.svg)](#origins)

AHRS (Attitude and Heading Reference System) filters for the
[Beam Bots](https://github.com/beam-bots/bb) framework.

This is version `0.1.0` of `bb_ahrs`.

Three 6-DOF IMU fusion algorithms, each implemented as a `BB.Estimator`:

- **`BB.Ahrs.Madgwick`** — gradient-descent filter. Standard parameter
  is `:beta` (default `0.1`).
- **`BB.Ahrs.Mahony`** — PI-controller filter with optional gyro bias
  estimation. Parameters `:kp` / `:ki`.
- **`BB.Ahrs.Complementary`** — high-pass gyro + low-pass tilt blend.
  Parameter `:alpha` or `:time_constant`.

All three consume `BB.Message.Sensor.Imu` payloads and republish them
with `:orientation` replaced by the fused estimate. Inputs and outputs
are in SI units (rad/s, m/s², radians) — unit conversion is the
publishing sensor's responsibility.

## Usage

Nest an estimator inside any sensor that publishes `BB.Message.Sensor.Imu`:

```elixir
defmodule MyRobot do
  use BB

  topology do
    link :base_link do
      sensor :imu, BB.Sensor.SomeImu, bus: "i2c-1", address: 0x68 do
        estimator :orientation, {BB.Ahrs.Madgwick, beta: 0.1}
      end
    end
  end
end
```

Subscribers to `[:sensor, :base_link, :imu, :orientation]` receive the
fused IMU message; subscribers to `[:sensor, :base_link, :imu]` still
receive the raw sensor output.

Swap algorithms with a one-line change:

```elixir
estimator :orientation, {BB.Ahrs.Mahony, kp: 2.0, ki: 0.005}
# or
estimator :orientation, {BB.Ahrs.Complementary, alpha: 0.98}
```

## Linear-acceleration rejection

All three algorithms accept an `:accel_threshold` option (default `0.1`,
expressed as a fractional deviation from 1 g). When the accelerometer
magnitude departs from gravity by more than the threshold, the
correction term is suppressed and the filter falls back to gyro-only
integration for that step. Standard mitigation for periods of sustained
linear acceleration.

## Origins

Ported from Gus Workman's [`gworkman/ahrs`](https://github.com/gworkman/ahrs).
The algorithm internals are essentially Gus's code with unit conversion
and `dt` tracking lifted out into the framework boundary. Gus's original
work is MIT; the new `BB.Estimator` wrappers and project scaffolding
are Apache-2.0. See per-file SPDX headers for details.

## Development

```bash
mix deps.get
mix test
mix check --no-retry
```

Pointing at a local `bb` checkout for cross-repo development:

```bash
BB_VERSION=local mix deps.get
BB_VERSION=local mix test
```

See the [proposal](https://github.com/beam-bots/proposals/blob/main/accepted/0018-bb-estimator.md)
for design background.
