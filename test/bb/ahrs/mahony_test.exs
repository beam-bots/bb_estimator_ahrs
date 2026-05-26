# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Ahrs.MahonyTest do
  use ExUnit.Case, async: true

  alias BB.Ahrs.Mahony

  @gravity 9.80665

  defp gravity_only_accel, do: {0.0, 0.0, @gravity}

  describe "step/4 - stationary" do
    test "stationary with gravity along +Z stays near identity" do
      state = struct!(Mahony, kp: 2.0, ki: 0.0)

      final =
        Enum.reduce(1..200, state, fn _, s ->
          Mahony.step(s, {0.0, 0.0, 0.0}, gravity_only_accel(), 0.01)
        end)

      assert_in_delta final.q.w, 1.0, 1.0e-3
    end

    test "ki accumulates integral bias estimate over time" do
      state = struct!(Mahony, kp: 0.0, ki: 1.0, e_int_limit: 10.0)

      tilted_accel = {0.0, @gravity * :math.sin(0.3), @gravity * :math.cos(0.3)}

      final =
        Enum.reduce(1..400, state, fn _, s ->
          Mahony.step(s, {0.0, 0.0, 0.0}, tilted_accel, 0.01)
        end)

      {ex, _ey, _ez} = final.e_int
      refute ex == 0.0
    end

    test "integral term is clamped by e_int_limit" do
      state = struct!(Mahony, kp: 0.0, ki: 100.0, e_int_limit: 0.5)
      strong_tilt = {0.0, @gravity * 0.9, @gravity * 0.1}

      final =
        Enum.reduce(1..100, state, fn _, s ->
          Mahony.step(s, {0.0, 0.0, 0.0}, strong_tilt, 0.01)
        end)

      {ex, _ey, _ez} = final.e_int
      assert abs(ex) <= 0.5
    end
  end

  describe "step/4 - accel_threshold" do
    test "ignores accel outside threshold" do
      state = struct!(Mahony, kp: 5.0, ki: 0.0, accel_threshold: 0.1)
      bad_accel = {0.0, 0.0, @gravity * 2.0}

      step_with_bad = Mahony.step(state, {0.0, 0.0, 0.0}, bad_accel, 0.01)

      assert step_with_bad.q == state.q
      assert step_with_bad.e_int == state.e_int
    end
  end

  describe "BB.Estimator integration" do
    alias BB.Math.{Quaternion, Vec3}
    alias BB.Message
    alias BB.Message.Sensor.Imu

    test "handle_input/2 emits an Imu output" do
      {:ok, state} = Mahony.init(kp: 2.0, ki: 0.0, accel_threshold: 0.1, e_int_limit: 100.0)

      {:ok, msg} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.new(0.0, 0.0, @gravity)
        )

      assert {:reply, [out: %Message{payload: %Imu{}}], _} = Mahony.handle_input(msg, state)
    end
  end
end
