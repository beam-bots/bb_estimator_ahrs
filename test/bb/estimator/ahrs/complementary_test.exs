# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Estimator.Ahrs.ComplementaryTest do
  use ExUnit.Case, async: true

  alias BB.Estimator.Ahrs.Complementary
  alias BB.Estimator.Ahrs.Math, as: AhrsMath

  @gravity 9.80665

  defp gravity_only_accel, do: {0.0, 0.0, @gravity}

  describe "step/4 - stationary" do
    test "stationary with gravity along +Z stays near identity" do
      state = struct!(Complementary, alpha: 0.98)

      final =
        Enum.reduce(1..200, state, fn _, s ->
          Complementary.step(s, {0.0, 0.0, 0.0}, gravity_only_accel(), 0.01)
        end)

      assert_in_delta final.q.w, 1.0, 1.0e-3
    end

    test "stationary tilt: tilted gravity pulls roll toward expected angle" do
      state = struct!(Complementary, alpha: 0.95)

      tilted_accel = {0.0, @gravity * :math.sin(0.4), @gravity * :math.cos(0.4)}

      final =
        Enum.reduce(1..400, state, fn _, s ->
          Complementary.step(s, {0.0, 0.0, 0.0}, tilted_accel, 0.01)
        end)

      {roll, _pitch, _yaw} = AhrsMath.quaternion_to_euler(final.q)
      assert_in_delta roll, 0.4, 0.05
    end
  end

  describe "step/4 - time_constant" do
    test "time_constant overrides alpha for frequency independence" do
      state = struct!(Complementary, time_constant: 1.0)

      tilted_accel = {0.0, @gravity * :math.sin(0.3), @gravity * :math.cos(0.3)}

      final =
        Enum.reduce(1..1000, state, fn _, s ->
          Complementary.step(s, {0.0, 0.0, 0.0}, tilted_accel, 0.01)
        end)

      {roll, _pitch, _yaw} = AhrsMath.quaternion_to_euler(final.q)
      assert_in_delta roll, 0.3, 0.05
    end
  end

  describe "step/4 - accel_threshold" do
    test "ignores accel outside threshold" do
      state = struct!(Complementary, alpha: 0.5, accel_threshold: 0.1)
      bad_accel = {0.0, 0.0, @gravity * 2.0}

      step_with_bad =
        Complementary.step(state, {0.0, 0.0, 0.0}, bad_accel, 0.01)

      assert step_with_bad.q == state.q
    end
  end

  describe "BB.Estimator integration" do
    alias BB.Math.{Quaternion, Vec3}
    alias BB.Message
    alias BB.Message.Sensor.Imu

    test "handle_input/2 emits an Imu output" do
      {:ok, state} =
        Complementary.init(alpha: 0.98, time_constant: nil, accel_threshold: 0.1)

      {:ok, msg} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.new(0.0, 0.0, @gravity)
        )

      assert {:reply, [out: %Message{payload: %Imu{}}], _} =
               Complementary.handle_input(msg, state)
    end
  end

  describe "handle_options/2" do
    # The behaviour's default implementation returns the state untouched, so
    # without this a gain bound to a robot parameter accepts new values, reports
    # them back when read, and goes on filtering with the old ones. That is
    # worse than not being tunable: it sends whoever is tuning it looking
    # somewhere else entirely.
    test "adopts every option it was given" do
      {:ok, state} = Complementary.init(alpha: 0.98, time_constant: nil, accel_threshold: 0.1)

      {:ok, updated} =
        Complementary.handle_options(
          [alpha: 0.9, time_constant: 0.5, accel_threshold: 0.3],
          state
        )

      assert updated.alpha == 0.9
      assert updated.time_constant == 0.5
      assert updated.accel_threshold == 0.3
    end

    test "leaves the filter's own state alone" do
      {:ok, state} = Complementary.init(alpha: 0.98, time_constant: nil, accel_threshold: 0.1)
      stepped = %{state | last_monotonic_time: 12_345}

      {:ok, updated} =
        Complementary.handle_options(
          [alpha: 0.9, time_constant: 0.5, accel_threshold: 0.3],
          stepped
        )

      assert updated.q == stepped.q
      assert updated.last_monotonic_time == 12_345
    end
  end
end
