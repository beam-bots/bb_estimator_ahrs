<!--
SPDX-FileCopyrightText: 2026 Gus Workman

SPDX-License-Identifier: MIT
-->

# Ahrs

Ahrs is a modular Elixir library for Attitude and Heading Reference System
(AHRS) algorithms. It provides a robust, algorithm-agnostic interface for
processing data from Inertial Measurement Units (IMUs) to determine 3D
orientation.

The library features strongly-typed sensor containers, automatic time-delta
tracking, and industry-standard robustness features like linear acceleration
rejection and integral anti-windup.

## Available Algorithms

The library currently provides three primary filters:

- **Madgwick:** A highly efficient gradient descent algorithm that is
  computationally inexpensive and well-suited for high-speed updates.
- **Mahony:** A robust Proportional-Integral (PI) controller filter. It tracks
  integral error and is often preferred for its stability on low-power hardware.
- **Complementary Filter:** A lightweight filter that combines high-pass
  integrated gyroscope data with a low-pass accelerometer tilt calculation. It
  is simple to tune and very computationally cheap.

## Usage

To use the library, you initialize an AHRS instance using the top-level `Ahrs`
module. This provides a unified API regardless of the underlying algorithm you
choose.

```elixir
alias Ahrs.Accelerometer.Sample, as: Accel
alias Ahrs.Gyroscope.Sample, as: Gyro

# Initialize the filter (Ahrs.new_madgwick(), Ahrs.new_mahony(), or Ahrs.new_complementary())
ahrs = Ahrs.new_madgwick()

# Create sensor samples from your hardware readings
measurements = %{
  accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
  gyro: %Gyro{x: 0.01, y: 0.0, z: 0.0, units: :rad_s}
}

# Update the filter state.
# The library automatically calculates the time delta (dt) between calls.
ahrs = Ahrs.update(ahrs, measurements)

# Convert the internal state into human-readable Euler angles (supports :radians and :degrees)
{roll, pitch, yaw} = Ahrs.euler_angles(ahrs, units: :degrees)
```

## Features

### Unified API and Algorithm Agnostic

The top-level `Ahrs` module allows you to swap between Madgwick, Mahony, and
Complementary filters by changing a single initialization line. The rest of your
integration code remains identical.

### Automatic Timing

One of the challenges in AHRS systems is accurately measuring the time elapsed
between sensor updates. This library simplifies this by automatically querying
the system monotonic clock during the `Ahrs.update/3` call. You can override
this by passing an explicit `:dt` option in seconds.

### Robust Sensor Rejection

All filters support an `:accel_threshold` (default `0.1` G). This represents the
maximum allowable deviation from earth gravity. Any reading outside the
$[0.9G, 1.1G]$ range is ignored, preventing orientation jumps during high linear
acceleration or vibration.

## Examples & Simulation

The project includes a 3D visual simulator to help you verify and tune your
filter configurations. It uses Phoenix LiveView and Three.js to render a
real-time visualization of the filter output.

See the [Examples README](examples/README.md) for more details.

### Roadmap

- [ ] Add 9-DOF MARG support (Magnetometer integration)
- [ ] Write a Nerves integration example for live hardware

## Installation

The package can be installed by adding `ahrs` to your list of dependencies in
`mix.exs`:

```elixir
def deps do
  [
    {:ahrs, "~> 0.1.0"}
  ]
end
```
