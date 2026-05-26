# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.MahonyTest do
  use ExUnit.Case, async: true
  alias Ahrs.Mahony
  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro
  alias Ahrs.Quaternion, as: Q
  alias Ahrs.Math

  describe "update/3" do
    test "initial run sets last_update_at and preserves state" do
      state = %Mahony{}

      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      new_state = Mahony.update(state, measurements)

      assert new_state.q == state.q
      assert new_state.e_int == {0.0, 0.0, 0.0}
      assert is_integer(new_state.last_update_at)
    end

    test "converges to stable orientation" do
      initial_state = %Mahony{}

      # Measurement: 90 deg pitch up
      measurements = %{
        accel: %Accel{x: -1.0, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      final_state =
        Enum.reduce(1..1000, initial_state, fn _i, state ->
          Mahony.update(state, measurements, dt: 0.01, kp: 5.0)
        end)

      {_roll, pitch, _yaw} = Math.quaternion_to_euler(final_state.q)

      assert_in_delta pitch, :math.pi() / 2.0, 0.01
    end

    test "handles free-fall (gyro integration only)" do
      initial_state = %Mahony{q: %Q{w: 1.0, x: 0.0, y: 0.0, z: 0.0}}

      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 90.0, y: 0.0, z: 0.0, units: :deg_s}
      }

      final_state =
        Enum.reduce(1..100, initial_state, fn _i, state ->
          Mahony.update(state, measurements, dt: 0.01)
        end)

      {roll, pitch, yaw} = Math.quaternion_to_euler(final_state.q)

      assert_in_delta roll, :math.pi() / 2.0, 0.001
      assert_in_delta pitch, 0.0, 0.001
      assert_in_delta yaw, 0.0, 0.001
    end

    test "accumulates integral error when ki > 0" do
      initial_state = %Mahony{}

      # Constant error: device is flat, but accel says it's tilted
      measurements = %{
        accel: %Accel{x: 1.0, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      # Run a few steps with ki
      state1 = Mahony.update(initial_state, measurements, dt: 0.1, ki: 1.0)
      state2 = Mahony.update(state1, measurements, dt: 0.1, ki: 1.0)

      {ex1, ey1, ez1} = state1.e_int
      {ex2, ey2, ez2} = state2.e_int

      # Integral error should be non-zero (specifically ey in this case) and increasing in magnitude
      assert ex1 != 0.0 or ey1 != 0.0 or ez1 != 0.0
      assert abs(ex2) + abs(ey2) + abs(ez2) > abs(ex1) + abs(ey1) + abs(ez1)
    end

    test "respects configurable accel_threshold" do
      initial_state = %Mahony{last_update_at: 0}
      # Accel at 0.85G. Deviation is 0.15G.
      measurements = %{
        accel: %Accel{x: 0.85, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      # Ignored by default (0.1 threshold) because 0.15 > 0.1
      state_ignored = Mahony.update(initial_state, measurements, dt: 0.1)
      # Applied with custom 0.2 threshold because 0.15 < 0.2
      state_applied = Mahony.update(initial_state, measurements, dt: 0.1, accel_threshold: 0.2)

      assert state_ignored.q == initial_state.q
      assert state_applied.q != initial_state.q
    end

    test "clamps integral error (anti-windup)" do
      initial_state = %Mahony{last_update_at: 0}

      measurements = %{
        accel: %Accel{x: 1.0, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      # Run with very low limit and high ki
      final_state =
        Mahony.update(initial_state, measurements, dt: 1.0, ki: 100.0, e_int_limit: 5.0)

      {ex, ey, ez} = final_state.e_int

      assert abs(ex) <= 5.0
      assert abs(ey) <= 5.0
      assert abs(ez) <= 5.0
    end
  end
end
