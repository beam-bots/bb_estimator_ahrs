# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.MadgwickTest do
  use ExUnit.Case, async: true
  alias Ahrs.Madgwick
  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro
  alias Ahrs.Quaternion, as: Q
  alias Ahrs.Math

  describe "update/3" do
    test "initial run sets last_update_at and does not change quaternion" do
      state = %Madgwick{}

      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      new_state = Madgwick.update(state, measurements)

      assert new_state.q == state.q
      assert is_integer(new_state.last_update_at)
    end

    test "converges to stable orientation with :dt override" do
      initial_state = %Madgwick{}

      # Measurement: Gravity pulling on negative X (90 deg pitch up)
      measurements = %{
        accel: %Accel{x: -1.0, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      # Run filter with explicit dt to simulate 10 seconds of data at 100Hz
      final_state =
        Enum.reduce(1..1000, initial_state, fn _i, state ->
          Madgwick.update(state, measurements, dt: 0.01, beta: 0.5)
        end)

      {_roll, pitch, _yaw} = Math.quaternion_to_euler(final_state.q)

      assert_in_delta pitch, :math.pi() / 2.0, 0.01
    end

    test "handles free-fall (gyro integration only)" do
      initial_state = %Madgwick{q: %Q{w: 1.0, x: 0.0, y: 0.0, z: 0.0}}

      # Zero acceleration, rotating 90 deg/s around X
      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 90.0, y: 0.0, z: 0.0, units: :deg_s}
      }

      # Run for 1 second using 100 steps of 0.01s for numerical stability
      final_state =
        Enum.reduce(1..100, initial_state, fn _i, state ->
          Madgwick.update(state, measurements, dt: 0.01)
        end)

      {roll, pitch, yaw} = Math.quaternion_to_euler(final_state.q)

      assert_in_delta roll, :math.pi() / 2.0, 0.001
      assert_in_delta pitch, 0.0, 0.001
      assert_in_delta yaw, 0.0, 0.001
    end

    test "respects beta tuning limits (beta = 0.0, pure gyro)" do
      initial_q = %Q{w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      initial_state = %Madgwick{q: initial_q, last_update_at: 0}

      # Measurement: High acceleration pull, but beta is 0.0 (ignore it)
      # Zero gyro rotation
      measurements = %{
        accel: %Accel{x: -1.0, y: 0.0, z: 0.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      final_state = Madgwick.update(initial_state, measurements, beta: 0.0, dt: 0.01)

      # Quaternion should remain exactly the same as gyro is zero
      assert final_state.q == initial_q
    end

    test "handles non-standard units (m/s² and deg/s)" do
      initial_state = %Madgwick{last_update_at: 0}

      # Rotating 90 deg/s around X axis, using non-standard units
      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 9.80665, units: :m_s2},
        gyro: %Gyro{x: 90.0, y: 0.0, z: 0.0, units: :deg_s}
      }

      final_state = Madgwick.update(initial_state, measurements, dt: 1.0)
      {roll, _pitch, _yaw} = Math.quaternion_to_euler(final_state.q)

      # 90 degrees = pi/2
      assert_in_delta roll, :math.pi() / 2.0, 0.3
    end
  end
end
