# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Estimator.Ahrs.Mahony do
  @moduledoc """
  Mahony AHRS filter, 6-DOF IMU variant, implemented as a `BB.Estimator`.

  Uses a Proportional-Integral controller to fuse gyroscope and
  accelerometer measurements. Computationally cheaper than Madgwick and
  particularly robust under sustained drift.

  ## Usage

      sensor :imu, BB.Sensor.SomeImu, ... do
        estimator :orientation, {BB.Estimator.Ahrs.Mahony, kp: 2.0, ki: 0.005}
      end

  ## Options

  - `:kp` — proportional gain on the accel/mag error vector. Default `2.0`.
  - `:ki` — integral gain (gyro bias estimation). Default `0.0` (P-only).
  - `:accel_threshold` — accepted deviation of `|accel| / g` from `1.0`
    before the correction is suppressed. Default `0.1`.
  - `:e_int_limit` — anti-windup clamp on the integral term. Default `100.0`.

  Ported from [gworkman/ahrs](https://github.com/gworkman/ahrs)' `Ahrs.Mahony`.
  """

  use BB.Estimator,
    options_schema: [
      kp: [type: :float, required: false, default: 2.0, doc: "Proportional gain"],
      ki: [type: :float, required: false, default: 0.0, doc: "Integral gain"],
      accel_threshold: [
        type: :float,
        required: false,
        default: 0.1,
        doc: "Maximum fractional deviation of accelerometer magnitude from 1 g"
      ],
      e_int_limit: [
        type: :float,
        required: false,
        default: 100.0,
        doc: "Anti-windup clamp on the integral term"
      ]
    ]

  alias BB.Estimator.Ahrs.Math, as: AhrsMath
  alias BB.Estimator.Ahrs.Quaternion, as: Q
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Sensor.Imu

  defstruct q: %Q{},
            e_int: {0.0, 0.0, 0.0},
            last_monotonic_time: nil,
            kp: 2.0,
            ki: 0.0,
            accel_threshold: 0.1,
            e_int_limit: 100.0

  @type t :: %__MODULE__{
          q: Q.t(),
          e_int: {float(), float(), float()},
          last_monotonic_time: integer() | nil,
          kp: float(),
          ki: float(),
          accel_threshold: float(),
          e_int_limit: float()
        }

  # ----------------------------------------------------------------------------
  # BB.Estimator callbacks
  # ----------------------------------------------------------------------------

  @impl BB.Estimator
  def init(opts) do
    {:ok,
     %__MODULE__{
       kp: Keyword.fetch!(opts, :kp),
       ki: Keyword.fetch!(opts, :ki),
       accel_threshold: Keyword.fetch!(opts, :accel_threshold),
       e_int_limit: Keyword.fetch!(opts, :e_int_limit)
     }}
  end

  @impl BB.Estimator
  def handle_options(opts, %__MODULE__{} = state) do
    {:ok,
     %{
       state
       | kp: Keyword.fetch!(opts, :kp),
         ki: Keyword.fetch!(opts, :ki),
         accel_threshold: Keyword.fetch!(opts, :accel_threshold),
         e_int_limit: Keyword.fetch!(opts, :e_int_limit)
     }}
  end

  @impl BB.Estimator
  def handle_input(%Message{payload: %Imu{} = imu} = message, %__MODULE__{} = state) do
    state = ingest(state, imu, message.monotonic_time)

    {:ok, out} =
      Imu.new(message.frame_id,
        orientation: Q.to_bb(state.q),
        angular_velocity: imu.angular_velocity,
        linear_acceleration: imu.linear_acceleration
      )

    {:reply, [out: out], state}
  end

  def handle_input(_other, state), do: {:noreply, state}

  # ----------------------------------------------------------------------------
  # Pure step (callable directly from tests).
  # ----------------------------------------------------------------------------

  @doc """
  Runs one Mahony step. `gyro` is rad/s, `accel` is m/s², `dt` is
  seconds. Returns the updated state.
  """
  @spec step(t(), {float(), float(), float()}, {float(), float(), float()}, float()) :: t()
  def step(%__MODULE__{} = state, _gyro, _accel, dt) when dt <= 0.0, do: state

  def step(%__MODULE__{q: q, e_int: {ex_int, ey_int, ez_int}} = state, gyro, accel, dt) do
    {gx, gy, gz} = gyro
    {ax, ay, az} = accel
    a_norm = :math.sqrt(ax * ax + ay * ay + az * az)
    a_norm_g = a_norm / AhrsMath.gravity()

    {gx, gy, gz, new_e_int} =
      if a_norm == 0.0 or abs(a_norm_g - 1.0) > state.accel_threshold do
        {gx, gy, gz, {ex_int, ey_int, ez_int}}
      else
        apply_correction(
          q,
          {ax / a_norm, ay / a_norm, az / a_norm},
          {gx, gy, gz},
          {ex_int, ey_int, ez_int},
          state,
          dt
        )
      end

    {q_dot_w, q_dot_x, q_dot_y, q_dot_z} = Q.gyro_derivative(q, gx, gy, gz)

    new_q =
      Q.normalise(%Q{
        w: q.w + q_dot_w * dt,
        x: q.x + q_dot_x * dt,
        y: q.y + q_dot_y * dt,
        z: q.z + q_dot_z * dt
      })

    %{state | q: new_q, e_int: new_e_int}
  end

  # ----------------------------------------------------------------------------
  # Internals
  # ----------------------------------------------------------------------------

  defp ingest(state, %Imu{} = imu, monotonic_time) do
    dt = compute_dt(state.last_monotonic_time, monotonic_time)

    state =
      step(state, vec_to_tuple(imu.angular_velocity), vec_to_tuple(imu.linear_acceleration), dt)

    %{state | last_monotonic_time: monotonic_time}
  end

  defp compute_dt(nil, _now), do: 0.0
  defp compute_dt(last, now), do: max((now - last) / 1_000_000_000.0, 0.0)

  defp vec_to_tuple(%Vec3{} = v), do: {Vec3.x(v), Vec3.y(v), Vec3.z(v)}

  defp apply_correction(q, {ax, ay, az}, {gx, gy, gz}, {ex_int, ey_int, ez_int}, state, dt) do
    {vx, vy, vz} = estimate_gravity_direction(q)
    {ex, ey, ez} = cross_product({ax, ay, az}, {vx, vy, vz})

    {nx_int, ny_int, nz_int} =
      if state.ki > 0.0 do
        limit = state.e_int_limit

        {
          clamp(ex_int + ex * state.ki * dt, -limit, limit),
          clamp(ey_int + ey * state.ki * dt, -limit, limit),
          clamp(ez_int + ez * state.ki * dt, -limit, limit)
        }
      else
        {ex_int, ey_int, ez_int}
      end

    ngx = gx + state.kp * ex + nx_int
    ngy = gy + state.kp * ey + ny_int
    ngz = gz + state.kp * ez + nz_int

    {ngx, ngy, ngz, {nx_int, ny_int, nz_int}}
  end

  defp estimate_gravity_direction(%Q{w: w, x: x, y: y, z: z}) do
    {
      2.0 * (x * z - w * y),
      2.0 * (w * x + y * z),
      w * w - x * x - y * y + z * z
    }
  end

  defp cross_product({ax, ay, az}, {vx, vy, vz}) do
    {
      ay * vz - az * vy,
      az * vx - ax * vz,
      ax * vy - ay * vx
    }
  end

  defp clamp(val, min, _max) when val < min, do: min
  defp clamp(val, _min, max) when val > max, do: max
  defp clamp(val, _min, _max), do: val
end
