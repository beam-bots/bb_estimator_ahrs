# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule AhrsTest do
  use ExUnit.Case, async: true
  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro

  describe "initialization" do
    test "new_madgwick/0 initializes correctly" do
      ahrs = Ahrs.new_madgwick()
      assert ahrs.algorithm == Ahrs.Madgwick
      assert %Ahrs.Madgwick{} = ahrs.state
    end

    test "new_mahony/0 initializes correctly" do
      ahrs = Ahrs.new_mahony()
      assert ahrs.algorithm == Ahrs.Mahony
      assert %Ahrs.Mahony{} = ahrs.state
    end
  end

  describe "update/3 delegation" do
    test "delegates update to underlying algorithm" do
      ahrs = Ahrs.new_madgwick()

      measurements = %{
        accel: %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g},
        gyro: %Gyro{x: 0.0, y: 0.0, z: 0.0, units: :rad_s}
      }

      # Initial update just sets timestamp
      ahrs = Ahrs.update(ahrs, measurements)
      assert is_integer(ahrs.state.last_update_at)

      # Second update with explicit dt should change orientation if we give it rates
      measurements = %{measurements | gyro: %Gyro{x: 1.0, y: 0.0, z: 0.0, units: :rad_s}}
      ahrs = Ahrs.update(ahrs, measurements, dt: 0.1)

      assert ahrs.state.q.x != 0.0
    end
  end

  describe "euler_angles/2" do
    test "passes options through to Math module" do
      ahrs = Ahrs.new_madgwick()
      # Default (radians)
      assert {0.0, 0.0, 0.0} == Ahrs.euler_angles(ahrs)
      # Degrees
      assert {0.0, 0.0, 0.0} == Ahrs.euler_angles(ahrs, units: :degrees)
    end
  end

  describe "quaternion/1" do
    test "returns the current quaternion from state" do
      ahrs = Ahrs.new_madgwick()
      assert ahrs.state.q == Ahrs.quaternion(ahrs)
    end
  end
end
