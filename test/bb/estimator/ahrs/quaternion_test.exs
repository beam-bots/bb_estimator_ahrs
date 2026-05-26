# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Estimator.Ahrs.QuaternionTest do
  use ExUnit.Case, async: true
  alias BB.Estimator.Ahrs.Quaternion

  defp magnitude(%Quaternion{w: w, x: x, y: y, z: z}) do
    :math.sqrt(w * w + x * x + y * y + z * z)
  end

  defp assert_quaternion_in_delta(q1, q2, delta \\ 1.0e-15) do
    assert_in_delta q1.w, q2.w, delta
    assert_in_delta q1.x, q2.x, delta
    assert_in_delta q1.y, q2.y, delta
    assert_in_delta q1.z, q2.z, delta
  end

  describe "normalise/1" do
    test "normalises a non-unit quaternion" do
      q = %Quaternion{w: 2.0, x: 2.0, y: 2.0, z: 2.0}
      normalised = Quaternion.normalise(q)

      assert_in_delta magnitude(normalised), 1.0, 1.0e-15
      assert normalised.w == 0.5
    end

    test "handles negative values" do
      q = %Quaternion{w: -1.0, x: -2.0, y: -3.0, z: -4.0}
      normalised = Quaternion.normalise(q)

      assert_in_delta magnitude(normalised), 1.0, 1.0e-15
      assert normalised.w < 0
      assert normalised.x < 0
    end

    test "normalises single-axis values" do
      q = %Quaternion{w: 0.0, x: 5.0, y: 0.0, z: 0.0}
      normalised = Quaternion.normalise(q)

      assert_in_delta magnitude(normalised), 1.0, 1.0e-15
      assert normalised == %Quaternion{w: 0.0, x: 1.0, y: 0.0, z: 0.0}
    end

    test "returns the original on zero magnitude" do
      q = %Quaternion{w: 0.0, x: 0.0, y: 0.0, z: 0.0}
      assert Quaternion.normalise(q) == q
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
      q = %Quaternion{w: half_sqrt_2, x: half_sqrt_2, y: 0.0, z: 0.0}

      assert Quaternion.multiply(q, identity) == q
      assert Quaternion.multiply(identity, q) == q
    end

    test "multiplication of two rotations" do
      s = 1.0 / :math.sqrt(2.0)
      q1 = %Quaternion{w: s, x: s, y: 0.0, z: 0.0}
      q2 = %Quaternion{w: s, x: 0.0, y: s, z: 0.0}

      res = Quaternion.multiply(q1, q2)
      expected = %Quaternion{w: 0.5, x: 0.5, y: 0.5, z: 0.5}

      assert_quaternion_in_delta(res, expected)
    end

    test "multiplication is non-commutative" do
      s = 1.0 / :math.sqrt(2.0)
      q1 = %Quaternion{w: s, x: s, y: 0.0, z: 0.0}
      q2 = %Quaternion{w: s, x: 0.0, y: s, z: 0.0}

      res1 = Quaternion.multiply(q1, q2)
      res2 = Quaternion.multiply(q2, q1)

      refute res1 == res2
      assert_in_delta res1.z, 0.5, 1.0e-15
      assert_in_delta res2.z, -0.5, 1.0e-15
    end

    test "multiplying by conjugate yields identity" do
      q = %Quaternion{w: 0.5, x: 0.5, y: 0.5, z: 0.5}
      conjugate = Quaternion.conjugate(q)
      identity = %Quaternion{w: 1.0, x: 0.0, y: 0.0, z: 0.0}

      res = Quaternion.multiply(q, conjugate)
      assert_quaternion_in_delta(res, identity)
    end
  end

  describe "to_bb/1 and from_bb/1" do
    test "round-trips through BB.Math.Quaternion" do
      q = %Quaternion{w: 0.5, x: 0.5, y: 0.5, z: 0.5}
      bb = Quaternion.to_bb(q)
      back = Quaternion.from_bb(bb)

      assert_quaternion_in_delta(back, q, 1.0e-12)
    end
  end
end
