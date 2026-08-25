# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Estimator.Ahrs.MadgwickTest do
  use ExUnit.Case, async: true

  alias BB.Estimator.Ahrs.Madgwick
  alias BB.Estimator.Ahrs.Math, as: AhrsMath

  @gravity 9.80665

  defp gravity_only_accel, do: {0.0, 0.0, @gravity}

  describe "step/4 - stationary" do
    test "stationary with gravity along +Z converges to identity" do
      state = struct!(Madgwick, beta: 0.5)

      final =
        Enum.reduce(1..200, state, fn _, s ->
          Madgwick.step(s, {0.0, 0.0, 0.0}, gravity_only_accel(), 0.01)
        end)

      assert_in_delta final.q.w, 1.0, 1.0e-3
      assert_in_delta final.q.x, 0.0, 1.0e-3
      assert_in_delta final.q.y, 0.0, 1.0e-3
      assert_in_delta final.q.z, 0.0, 1.0e-3
    end

    test "stationary with accel-only converges to tilt from gravity" do
      state = struct!(Madgwick, beta: 0.5)

      tilted_accel = {0.0, @gravity * :math.sin(0.5), @gravity * :math.cos(0.5)}

      final =
        Enum.reduce(1..400, state, fn _, s ->
          Madgwick.step(s, {0.0, 0.0, 0.0}, tilted_accel, 0.01)
        end)

      {roll, _pitch, _yaw} = AhrsMath.quaternion_to_euler(final.q)
      assert_in_delta roll, 0.5, 0.05
    end
  end

  describe "step/4 - pure gyro integration" do
    test "constant gyro about Z integrates yaw" do
      state = struct!(Madgwick, beta: 0.0)

      omega = 1.0
      duration = 1.0
      steps = 1000

      final =
        Enum.reduce(1..steps, state, fn _, s ->
          Madgwick.step(s, {0.0, 0.0, omega}, gravity_only_accel(), duration / steps)
        end)

      {_roll, _pitch, yaw} = AhrsMath.quaternion_to_euler(final.q)
      assert_in_delta yaw, omega * duration, 0.05
    end
  end

  describe "step/4 - accel_threshold" do
    test "ignores accel outside threshold" do
      state = struct!(Madgwick, beta: 1.0, accel_threshold: 0.1)
      bad_accel = {0.0, 0.0, @gravity * 2.0}

      step_with_bad =
        Madgwick.step(state, {0.0, 0.0, 0.0}, bad_accel, 0.01)

      assert step_with_bad.q == state.q
    end
  end

  describe "BB.Estimator integration" do
    alias BB.Math.{Quaternion, Vec3}
    alias BB.Message
    alias BB.Message.Sensor.Imu

    test "init/1 with default options" do
      assert {:ok, state} =
               Madgwick.init(
                 beta: 0.1,
                 accel_threshold: 0.1,
                 bb: %{robot: nil, path: []},
                 estimator_context: %BB.Estimator.Context{
                   robot: nil,
                   path: [],
                   target_frame: :imu,
                   transforms: %{}
                 }
               )

      assert state.beta == 0.1
      assert state.accel_threshold == 0.1
    end

    test "handle_input/2 first message records time, emits one output" do
      {:ok, state} = Madgwick.init(beta: 0.1, accel_threshold: 0.1)

      {:ok, msg} =
        Imu.new(:imu,
          orientation: Quaternion.identity(),
          angular_velocity: Vec3.zero(),
          linear_acceleration: Vec3.new(0.0, 0.0, @gravity)
        )

      assert {:reply, [out: %Message{payload: %Imu{}}], new_state} =
               Madgwick.handle_input(msg, state)

      assert new_state.last_monotonic_time == msg.monotonic_time
    end

    test "handle_input/2 ignores non-IMU messages" do
      {:ok, state} = Madgwick.init(beta: 0.1, accel_threshold: 0.1)
      assert {:noreply, ^state} = Madgwick.handle_input(:something_else, state)
    end
  end

  describe "handle_options/2" do
    # The behaviour's default implementation returns the state untouched, so
    # without this a gain bound to a robot parameter accepts new values, reports
    # them back when read, and goes on filtering with the old ones. That is
    # worse than not being tunable: it sends whoever is tuning it looking
    # somewhere else entirely.
    test "adopts every option it was given" do
      {:ok, state} = Madgwick.init(beta: 0.1, accel_threshold: 0.1)
      {:ok, updated} = Madgwick.handle_options([beta: 0.02, accel_threshold: 0.25], state)

      assert updated.beta == 0.02
      assert updated.accel_threshold == 0.25
    end

    test "leaves the filter's own state alone" do
      {:ok, state} = Madgwick.init(beta: 0.1, accel_threshold: 0.1)
      stepped = %{state | last_monotonic_time: 12_345}
      {:ok, updated} = Madgwick.handle_options([beta: 0.02, accel_threshold: 0.25], stepped)

      assert updated.q == stepped.q
      assert updated.last_monotonic_time == 12_345
    end
  end
end
