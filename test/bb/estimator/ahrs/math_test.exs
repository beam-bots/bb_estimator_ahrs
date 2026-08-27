# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Estimator.Ahrs.MathTest do
  use ExUnit.Case, async: true

  alias BB.Estimator.Ahrs.Math
  alias BB.Math.Quaternion, as: Q

  @tolerance 1.0e-9

  describe "gyro_derivative/4" do
    test "is zero when the body is not rotating" do
      assert Math.gyro_derivative(Q.identity(), 0.0, 0.0, 0.0) == {0.0, 0.0, 0.0, 0.0}
    end

    test "matches the half-omega product for rotation about each axis" do
      # From identity, q̇ = ½ * ω, one component per axis.
      for {omega, expected} <- [
            {{2.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}},
            {{0.0, 2.0, 0.0}, {0.0, 0.0, 1.0, 0.0}},
            {{0.0, 0.0, 2.0}, {0.0, 0.0, 0.0, 1.0}}
          ] do
        {gx, gy, gz} = omega

        Q.identity()
        |> Math.gyro_derivative(gx, gy, gz)
        |> Tuple.to_list()
        |> Enum.zip(Tuple.to_list(expected))
        |> Enum.each(fn {actual, expected} ->
          assert_in_delta actual, expected, @tolerance
        end)
      end
    end

    test "stays orthogonal to the orientation, so integrating preserves unit norm" do
      q = Q.from_euler(0.3, -0.2, 0.7, :zyx)
      {dw, dx, dy, dz} = Math.gyro_derivative(q, 0.4, -0.9, 0.15)

      assert_in_delta q.w * dw + q.x * dx + q.y * dy + q.z * dz, 0.0, @tolerance
    end

    test "returns a bare tuple, not a quaternion - a derivative is not a rotation" do
      assert is_tuple(Math.gyro_derivative(Q.identity(), 1.0, 2.0, 3.0))
    end
  end

  describe "gravity/0" do
    test "returns standard gravity in m/s²" do
      assert Math.gravity() == 9.80665
    end
  end

  describe "quaternion_to_euler/2" do
    test "identity quaternion → zero euler angles" do
      {roll, pitch, yaw} = Math.quaternion_to_euler(Q.identity())
      assert_in_delta roll, 0.0, @tolerance
      assert_in_delta pitch, 0.0, @tolerance
      assert_in_delta yaw, 0.0, @tolerance
    end

    test "90° roll" do
      s = 1.0 / :math.sqrt(2.0)
      q = %Q{w: s, x: s, y: 0.0, z: 0.0}
      {roll, pitch, yaw} = Math.quaternion_to_euler(q)

      assert_in_delta roll, :math.pi() / 2, @tolerance
      assert_in_delta pitch, 0.0, @tolerance
      assert_in_delta yaw, 0.0, @tolerance
    end

    test "supports :degrees output" do
      s = 1.0 / :math.sqrt(2.0)
      q = %Q{w: s, x: s, y: 0.0, z: 0.0}
      {roll, _pitch, _yaw} = Math.quaternion_to_euler(q, units: :degrees)
      assert_in_delta roll, 90.0, 1.0e-6
    end
  end

  describe "euler_to_quaternion/3 round-trip" do
    test "30/20/10 degrees round-trips" do
      r = 30.0 * :math.pi() / 180.0
      p = 20.0 * :math.pi() / 180.0
      y = 10.0 * :math.pi() / 180.0

      q = Math.euler_to_quaternion(r, p, y)
      {r2, p2, y2} = Math.quaternion_to_euler(q)

      assert_in_delta r2, r, 1.0e-9
      assert_in_delta p2, p, 1.0e-9
      assert_in_delta y2, y, 1.0e-9
    end
  end

  describe "accel_to_tilt/3" do
    test "stationary, gravity along +Z → zero roll and pitch" do
      {roll, pitch} = Math.accel_to_tilt(0.0, 0.0, 9.81)
      assert_in_delta roll, 0.0, @tolerance
      assert_in_delta pitch, 0.0, @tolerance
    end

    test "gravity along +Y → 90° roll, zero pitch" do
      {roll, pitch} = Math.accel_to_tilt(0.0, 9.81, 0.0)
      assert_in_delta roll, :math.pi() / 2, 1.0e-6
      assert_in_delta pitch, 0.0, 1.0e-6
    end

    test "zero acceleration → zero output" do
      {roll, pitch} = Math.accel_to_tilt(0.0, 0.0, 0.0)
      assert roll == 0.0
      assert pitch == 0.0
    end
  end

  describe "gravity_only?/4" do
    test "exactly 1 g along Z" do
      assert Math.gravity_only?(0.0, 0.0, 9.80665, 0.01)
    end

    test "within ±10% of 1 g" do
      assert Math.gravity_only?(0.0, 0.0, 9.80665 * 1.05, 0.1)
      assert Math.gravity_only?(0.0, 0.0, 9.80665 * 0.95, 0.1)
    end

    test "outside threshold" do
      refute Math.gravity_only?(0.0, 0.0, 9.80665 * 1.5, 0.1)
      refute Math.gravity_only?(0.0, 0.0, 0.0, 0.1)
    end
  end

  describe "rotate_vector/2" do
    test "identity quaternion preserves the vector" do
      assert {1.0, 2.0, 3.0} = Math.rotate_vector({1.0, 2.0, 3.0}, Q.identity())
    end

    test "90° rotation around Z takes +X to +Y" do
      s = 1.0 / :math.sqrt(2.0)
      q = %Q{w: s, x: 0.0, y: 0.0, z: s}
      {x, y, z} = Math.rotate_vector({1.0, 0.0, 0.0}, q)

      assert_in_delta x, 0.0, 1.0e-9
      assert_in_delta y, 1.0, 1.0e-9
      assert_in_delta z, 0.0, 1.0e-9
    end
  end
end
