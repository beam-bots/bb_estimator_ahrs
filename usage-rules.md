<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# bb_estimator_ahrs Usage Rules

`bb_estimator_ahrs` provides three `BB.Estimator` implementations that fuse a
6-DOF IMU's gyroscope and accelerometer into a drift-free orientation for
[Beam Bots](https://hexdocs.pm/bb): `BB.Estimator.Ahrs.Madgwick`,
`BB.Estimator.Ahrs.Mahony`, and `BB.Estimator.Ahrs.Complementary`.
For BB framework basics, see `bb`'s rules
(`mix usage_rules.sync <file> bb:all`); this file covers only what's specific to
these estimators.

## Core principles

1. **An estimator is a callback module you nest in the DSL, not a process you
   supervise.** You wire it into a `sensor` as `{Module, opts}`; `bb` wraps it
   in `BB.Estimator.Server`, validates the options, and routes its output. You
   never write a `child_spec` or call `BB.PubSub.publish/3`.
2. **These are within-sensor estimators — nest them inside an IMU sensor, not a
   link.** The parent sensor's `BB.Message.Sensor.Imu` output is the implicit
   input; the fused orientation publishes in the sensor's own frame. `input`
   blocks are a compile error here (that's the link-nested, cross-sensor form).
3. **Everything is SI.** Inputs are rad/s (angular velocity) and m/s² (linear
   acceleration); orientation is radians. Unit conversion is the publishing
   sensor driver's job — the algorithms do not convert, and the upstream
   library's conversion machinery has been removed.
4. **`dt` comes from `message.monotonic_time`, not the wall clock.** The filter
   emits nothing useful on the first message (no prior timestamp to difference
   against) and integrates from the second onward.

## Wiring it in

Nest an estimator inside any sensor that publishes `BB.Message.Sensor.Imu`. The
estimator's name becomes the last segment of its output path:

```elixir
topology do
  link :base_link do
    sensor :imu, BB.Sensor.SomeImu, bus: "i2c-1", address: 0x68 do
      estimator :orientation, {BB.Estimator.Ahrs.Madgwick, beta: 0.1}
    end
  end
end
```

Subscribers to `[:sensor, :base_link, :imu, :orientation]` receive the fused IMU
message (orientation replaced; angular velocity and linear acceleration passed
through). Subscribers to `[:sensor, :base_link, :imu]` still see the raw sensor
output. Swapping algorithms is a one-line change.

## Choosing an algorithm

All three take `:accel_threshold` (default `0.1`) — the accepted fractional
deviation of `|accel| / g` from `1.0`. Outside that band the accelerometer
correction is suppressed and the step falls back to gyro-only integration
(rejection of sustained linear acceleration). Beyond that they differ:

| Module | Key options (defaults) | When to reach for it |
|---|---|---|
| `BB.Estimator.Ahrs.Madgwick` | `beta: 0.1` | Gradient-descent filter. `:beta` trades tracking speed against noise. Good general default. |
| `BB.Estimator.Ahrs.Mahony` | `kp: 2.0`, `ki: 0.0`, `e_int_limit: 100.0` | PI-controller filter; cheaper than Madgwick. Set `:ki > 0` to estimate gyro bias (`:e_int_limit` is the anti-windup clamp); `:ki = 0.0` is P-only. |
| `BB.Estimator.Ahrs.Complementary` | `alpha: 0.98`, `time_constant: nil` | High-pass gyro blended with low-pass accel tilt. `:alpha` is the fixed gyro weight; set `:time_constant` (τ seconds) instead to make the blend frame-rate-independent (`τ / (τ + dt)`), which overrides `:alpha`. |

Options are validated by each module's `Spark.Options` schema, so a bad key or
type fails at compile time — don't assume an option carries across algorithms
(e.g. `:beta` is Madgwick-only).

## Health reporting

The `estimator` entity carries `bb`'s standard health knobs; use the `~u` sigil
for the time-valued ones. When a dispatch overruns `latency_budget` the
estimator goes `:degraded`; if no input arrives within `lost_after` it goes
`:lost`. Wire the transitions to commands:

```elixir
estimator :orientation, {BB.Estimator.Ahrs.Mahony, kp: 2.0, ki: 0.005},
  latency_budget: ~u(5 millisecond),
  lost_after: ~u(200 millisecond),
  on_lost: :hold_position
```

## Anti-patterns

- **Don't nest these at the link level or declare `input` blocks.** They are
  single-input, within-sensor filters — they belong inside the IMU `sensor`.
  Link-nesting them or adding `input` is a compile error.
- **Don't scale units before handing data to the estimator.** Feeding degrees/s
  or g's produces silent garbage; convert to rad/s and m/s² in the sensor
  driver.
- **Don't carry one algorithm's options to another.** `:beta`, `:kp`/`:ki`, and
  `:alpha`/`:time_constant` are algorithm-specific; only `:accel_threshold` is
  shared.

## Further reading

- [bb_estimator_ahrs docs](https://hexdocs.pm/bb_estimator_ahrs)
- `bb`'s [Understanding Estimators](https://hexdocs.pm/bb/understanding-estimators.html)
  and [Configure Estimator Health](https://hexdocs.pm/bb/configure-estimator-health.html)
