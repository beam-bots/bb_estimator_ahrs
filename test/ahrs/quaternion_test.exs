# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.QuaternionTest do
  use ExUnit.Case, async: true
  alias Ahrs.Quaternion

  defp magnitude(%Quaternion{w: w, x: x, y: y, z: z}) do
    :math.sqrt(w * w + x * x + y * y + z * z)
  end

  defp assert_quaternion_in_delta(q1, q2, delta \\ 1.0e-15) do
    assert_in_delta q1.w, q2.w, delta
    assert_in_delta q1.x, q2.x, delta
    assert_in_delta q1.y, q2.y, delta
    assert_in_delta q1.z, q2.z, delta
  end

  describe "normalize/1" do
    test "normalizes a non-unit quaternion" do
      q = %Quaternion{w: 2.0, x: 2.0, y: 2.0, z: 2.0}
      normalized = Quaternion.normalize(q)

      assert_in_delta magnitude(normalized), 1.0, 1.0e-15
      assert normalized.w == 0.5
    end

    test "handles negative values" do
      q = %Quaternion{w: -1.0, x: -2.0, y: -3.0, z: -4.0}
      normalized = Quaternion.normalize(q)

      assert_in_delta magnitude(normalized), 1.0, 1.0e-15
      assert normalized.w < 0
      assert normalized.x < 0
    end

    test "normalizes single-axis values" do
      q = %Quaternion{w: 0.0, x: 5.0, y: 0.0, z: 0.0}
      normalized = Quaternion.normalize(q)

      assert_in_delta magnitude(normalized), 1.0, 1.0e-15
      assert normalized == %Quaternion{w: 0.0, x: 1.0, y: 0.0, z: 0.0}
    end

    test "handles zero magnitude (returns original)" do
      q = %Quaternion{w: 0.0, x: 0.0, y: 0.0, z: 0.0}
      assert Quaternion.normalize(q) == q
    end
  end

  describe "conjugate/1" do
    test "returns the correct conjugate" do
      q = %Quaternion{w: 1.0, x: 2.0, y: 3.0, z: 4.0}
      assert Quaternion.conjugate(q) == %Quaternion{w: 1.0, x: -2.0, y: -3.0, z: -4.0}
    end

    test "conjugate of identity is identity" do
      identity = %Quaternion{w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      assert Quaternion.conjugate(identity) == identity
    end

    test "double conjugate returns original" do
      q = %Quaternion{w: 1.0, x: 2.0, y: -3.0, z: 4.0}
      assert q |> Quaternion.conjugate() |> Quaternion.conjugate() == q
    end
  end

  describe "multiply/2" do
    test "identity multiplication" do
      identity = %Quaternion{w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      half_sqrt_2 = 1.0 / :math.sqrt(2.0)
      # 90 deg rotation around X
      q = %Quaternion{w: half_sqrt_2, x: half_sqrt_2, y: 0.0, z: 0.0}

      assert Quaternion.multiply(q, identity) == q
      assert Quaternion.multiply(identity, q) == q
    end

    test "multiplication of two rotations" do
      # Example: 90 deg rotation around X * 90 deg rotation around Y
      # cos(45) = sin(45) = 1/sqrt(2)
      s = 1.0 / :math.sqrt(2.0)
      q1 = %Quaternion{w: s, x: s, y: 0.0, z: 0.0}
      q2 = %Quaternion{w: s, x: 0.0, y: s, z: 0.0}

      res = Quaternion.multiply(q1, q2)

      # Expected: w=0.5, x=0.5, y=0.5, z=0.5
      expected = %Quaternion{w: 0.5, x: 0.5, y: 0.5, z: 0.5}
      assert_quaternion_in_delta(res, expected)
    end

    test "multiplication is non-commutative" do
      s = 1.0 / :math.sqrt(2.0)
      q1 = %Quaternion{w: s, x: s, y: 0.0, z: 0.0}
      q2 = %Quaternion{w: s, x: 0.0, y: s, z: 0.0}

      res1 = Quaternion.multiply(q1, q2)
      res2 = Quaternion.multiply(q2, q1)

      assert res1 != res2
      # For (90X * 90Y) vs (90Y * 90X), the Z component flips sign
      assert_in_delta res1.z, 0.5, 1.0e-15
      assert_in_delta res2.z, -0.5, 1.0e-15
    end

    test "multiplying by conjugate yields identity (for unit quaternions)" do
      q = %Quaternion{w: 0.5, x: 0.5, y: 0.5, z: 0.5}
      conjugate = Quaternion.conjugate(q)
      identity = %Quaternion{w: 1.0, x: 0.0, y: 0.0, z: 0.0}

      res = Quaternion.multiply(q, conjugate)
      assert_quaternion_in_delta(res, identity)
    end
  end
end
