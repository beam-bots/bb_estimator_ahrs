# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.MathTest do
  use ExUnit.Case, async: true
  alias Ahrs.Math
  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro
  alias Ahrs.Quaternion, as: Q

  @gravity 9.80665

  describe "convert/2 (Accelerometer)" do
    test "returns same sample if units match" do
      sample = %Accel{x: 1.0, y: 2.0, z: 3.0, units: :g}
      assert Math.convert(sample, :g) == sample
    end

    test "converts :g to :m_s2" do
      sample = %Accel{x: 1.0, y: -2.0, z: 0.0, units: :g}
      converted = Math.convert(sample, :m_s2)

      assert %Accel{} = converted
      assert converted.units == :m_s2
      assert_in_delta converted.x, @gravity, 1.0e-6
      assert_in_delta converted.y, -2.0 * @gravity, 1.0e-6
      assert converted.z == 0.0
    end

    test "converts :m_s2 to :g" do
      sample = %Accel{x: @gravity, y: -(@gravity * 2.0), z: 0.0, units: :m_s2}
      converted = Math.convert(sample, :g)

      assert %Accel{} = converted
      assert converted.units == :g
      assert_in_delta converted.x, 1.0, 1.0e-6
      assert_in_delta converted.y, -2.0, 1.0e-6
      assert converted.z == 0.0
    end
  end

  describe "convert/2 (Gyroscope)" do
    test "returns same sample if units match" do
      sample = %Gyro{x: 1.0, y: 2.0, z: 3.0, units: :rad_s}
      assert Math.convert(sample, :rad_s) == sample
    end

    test "converts :deg_s to :rad_s" do
      sample = %Gyro{x: 180.0, y: -90.0, z: 0.0, units: :deg_s}
      converted = Math.convert(sample, :rad_s)

      assert %Gyro{} = converted
      assert converted.units == :rad_s
      assert_in_delta converted.x, :math.pi(), 1.0e-6
      assert_in_delta converted.y, -:math.pi() / 2.0, 1.0e-6
      assert converted.z == 0.0
    end

    test "converts :rad_s to :deg_s" do
      sample = %Gyro{x: :math.pi(), y: -:math.pi() / 2.0, z: 0.0, units: :rad_s}
      converted = Math.convert(sample, :deg_s)

      assert %Gyro{} = converted
      assert converted.units == :deg_s
      assert_in_delta converted.x, 180.0, 1.0e-6
      assert_in_delta converted.y, -90.0, 1.0e-6
      assert converted.z == 0.0
    end
  end

  describe "quaternion_to_euler/1" do
    test "identity quaternion yields zero Euler angles" do
      q = %Q{w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      {roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert roll == 0.0
      assert pitch == 0.0
      assert yaw == 0.0
    end

    test "converts 90 degree pitch (y-axis)" do
      half_sqrt_2 = 1.0 / :math.sqrt(2.0)
      q = %Q{w: half_sqrt_2, x: 0.0, y: half_sqrt_2, z: 0.0}
      {roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert_in_delta roll, 0.0, 1.0e-6
      assert_in_delta pitch, :math.pi() / 2.0, 1.0e-6
      assert_in_delta yaw, 0.0, 1.0e-6
    end

    test "converts 90 degree roll (x-axis)" do
      half_sqrt_2 = 1.0 / :math.sqrt(2.0)
      q = %Q{w: half_sqrt_2, x: half_sqrt_2, y: 0.0, z: 0.0}
      {roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert_in_delta roll, :math.pi() / 2.0, 1.0e-6
      assert_in_delta pitch, 0.0, 1.0e-6
      assert_in_delta yaw, 0.0, 1.0e-6
    end

    test "converts 90 degree yaw (z-axis)" do
      half_sqrt_2 = 1.0 / :math.sqrt(2.0)
      q = %Q{w: half_sqrt_2, x: 0.0, y: 0.0, z: half_sqrt_2}
      {roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert_in_delta roll, 0.0, 1.0e-6
      assert_in_delta pitch, 0.0, 1.0e-6
      assert_in_delta yaw, :math.pi() / 2.0, 1.0e-6
    end

    test "respects units: :degrees option" do
      half_sqrt_2 = 1.0 / :math.sqrt(2.0)
      # 90 deg Roll
      q = %Q{w: half_sqrt_2, x: half_sqrt_2, y: 0.0, z: 0.0}
      {roll, pitch, yaw} = Math.quaternion_to_euler(q, units: :degrees)

      assert_in_delta roll, 90.0, 1.0e-6
      assert_in_delta pitch, 0.0, 1.0e-6
      assert_in_delta yaw, 0.0, 1.0e-6
    end

    test "handles gimbal lock (+90 pitch)" do
      half_sqrt_2 = 1.0 / :math.sqrt(2.0)
      q = %Q{w: half_sqrt_2, x: 0.0, y: half_sqrt_2, z: 0.0}
      {_roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert_in_delta pitch, :math.pi() / 2.0, 1.0e-6
      assert yaw == 0.0
    end

    test "handles gimbal lock (-90 pitch)" do
      half_sqrt_2 = 1.0 / :math.sqrt(2.0)
      q = %Q{w: half_sqrt_2, x: 0.0, y: -half_sqrt_2, z: 0.0}
      {_roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert_in_delta pitch, -:math.pi() / 2.0, 1.0e-6
      assert yaw == 0.0
    end
  end

  describe "accel_to_tilt/1" do
    test "flat orientation" do
      sample = %Accel{x: 0.0, y: 0.0, z: 1.0, units: :g}
      {roll, pitch} = Math.accel_to_tilt(sample)

      assert roll == 0.0
      assert pitch == 0.0
    end

    test "90 degree pitch up" do
      sample = %Accel{x: -1.0, y: 0.0, z: 0.0, units: :g}
      {roll, pitch} = Math.accel_to_tilt(sample)

      assert roll == 0.0
      assert_in_delta pitch, :math.pi() / 2.0, 1.0e-6
    end

    test "90 degree roll right" do
      sample = %Accel{x: 0.0, y: 1.0, z: 0.0, units: :g}
      {roll, pitch} = Math.accel_to_tilt(sample)

      assert_in_delta roll, :math.pi() / 2.0, 1.0e-6
      assert pitch == 0.0
    end

    test "handles free-fall (zero acceleration)" do
      sample = %Accel{x: 0.0, y: 0.0, z: 0.0, units: :g}
      {roll, pitch} = Math.accel_to_tilt(sample)

      assert roll == 0.0
      assert pitch == 0.0
    end
  end

  describe "complex orientations" do
    test "combined 45 degree pitch and 45 degree roll" do
      # Rotation of 45 degrees (pi/4). Quaternion half-angle is pi/8.
      half_angle = :math.pi() / 8.0
      c = :math.cos(half_angle)
      s = :math.sin(half_angle)

      # q = q_pitch * q_roll
      # q_pitch = [c, 0, s, 0]
      # q_roll = [c, s, 0, 0]
      q = %Q{
        w: c * c,
        x: s * c,
        y: c * s,
        z: -s * s
      }

      {roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert_in_delta roll, :math.pi() / 4.0, 1.0e-15
      assert_in_delta pitch, :math.pi() / 4.0, 1.0e-15
      assert_in_delta yaw, 0.0, 1.0e-15
    end

    test "180 degree flip (roll)" do
      q = %Q{w: 0.0, x: 1.0, y: 0.0, z: 0.0}
      {roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert_in_delta abs(roll), :math.pi(), 1.0e-6
      assert_in_delta pitch, 0.0, 1.0e-6
      assert_in_delta yaw, 0.0, 1.0e-6
    end
  end
end
