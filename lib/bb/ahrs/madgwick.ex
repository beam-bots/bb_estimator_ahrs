# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Ahrs.Madgwick do
  @moduledoc """
  Madgwick AHRS filter, 6-DOF IMU variant, implemented as a
  `BB.Estimator`.

  Fuses gyroscope and accelerometer measurements into a drift-free
  orientation using a gradient-descent optimisation step. Consumes
  `BB.Message.Sensor.Imu` inputs and republishes them with `:orientation`
  replaced by the fused estimate; `:angular_velocity` and
  `:linear_acceleration` are passed through unchanged.

  ## Usage

      sensor :imu, BB.Sensor.SomeImu, ... do
        estimator :orientation, {BB.Ahrs.Madgwick, beta: 0.1}
      end

  ## Options

  - `:beta` — gradient-descent step size. Higher = tracks faster but
    noisier. Default `0.1`.
  - `:accel_threshold` — accepted deviation of `|accel| / g` from `1.0`
    before the accelerometer correction is suppressed (gyro integration
    only). Default `0.1` (i.e. ±10% of 1 g).

  Ported from [gworkman/ahrs](https://github.com/gworkman/ahrs)'
  `Ahrs.Madgwick`.
  """

  use BB.Estimator,
    options_schema: [
      beta: [type: :float, required: false, default: 0.1, doc: "Gradient descent gain"],
      accel_threshold: [
        type: :float,
        required: false,
        default: 0.1,
        doc: "Maximum fractional deviation of accelerometer magnitude from 1 g"
      ]
    ]

  alias BB.Ahrs.Math, as: AhrsMath
  alias BB.Ahrs.Quaternion, as: Q
  alias BB.Math.Quaternion, as: NxQuaternion
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Sensor.Imu

  defstruct q: %Q{}, last_monotonic_time: nil, beta: 0.1, accel_threshold: 0.1

  @type t :: %__MODULE__{
          q: Q.t(),
          last_monotonic_time: integer() | nil,
          beta: float(),
          accel_threshold: float()
        }

  # ----------------------------------------------------------------------------
  # BB.Estimator callbacks
  # ----------------------------------------------------------------------------

  @impl BB.Estimator
  def init(opts) do
    {:ok,
     %__MODULE__{
       beta: Keyword.fetch!(opts, :beta),
       accel_threshold: Keyword.fetch!(opts, :accel_threshold)
     }}
  end

  @impl BB.Estimator
  def handle_input(%Message{payload: %Imu{} = imu} = message, %__MODULE__{} = state) do
    state = ingest(state, imu, message.monotonic_time)
    out = build_output_message(message, imu, state)
    {:reply, [out: out], state}
  end

  def handle_input(_other, state), do: {:noreply, state}

  # ----------------------------------------------------------------------------
  # Pure step (callable directly from tests).
  # ----------------------------------------------------------------------------

  @doc """
  Runs one Madgwick step against `state`. `dt` is the time since the
  previous step in seconds; `gyro` is rad/s, `accel` is m/s².

  Use this directly for testing or for callers who don't want the
  `BB.Estimator` envelope.
  """
  @spec step(t(), {float(), float(), float()}, {float(), float(), float()}, float()) :: t()
  def step(%__MODULE__{} = state, _gyro, _accel, dt) when dt <= 0.0, do: state

  def step(%__MODULE__{q: q_in} = state, {gx, gy, gz}, {ax, ay, az}, dt) do
    q = Q.normalise(q_in)

    {q_dot_w, q_dot_x, q_dot_y, q_dot_z} = Q.gyro_derivative(q, gx, gy, gz)

    {q_dot_w, q_dot_x, q_dot_y, q_dot_z} =
      apply_correction(q, {q_dot_w, q_dot_x, q_dot_y, q_dot_z}, {ax, ay, az}, state)

    new_q =
      Q.normalise(%Q{
        w: q.w + q_dot_w * dt,
        x: q.x + q_dot_x * dt,
        y: q.y + q_dot_y * dt,
        z: q.z + q_dot_z * dt
      })

    %{state | q: new_q}
  end

  # ----------------------------------------------------------------------------
  # Internals
  # ----------------------------------------------------------------------------

  defp ingest(state, %Imu{} = imu, monotonic_time) do
    dt = compute_dt(state.last_monotonic_time, monotonic_time)

    {gx, gy, gz} = vec_to_tuple(imu.angular_velocity)
    {ax, ay, az} = vec_to_tuple(imu.linear_acceleration)

    state =
      if dt > 0.0,
        do: step(state, {gx, gy, gz}, {ax, ay, az}, dt),
        else: state

    %{state | last_monotonic_time: monotonic_time}
  end

  defp compute_dt(nil, _now), do: 0.0
  defp compute_dt(last, now), do: max((now - last) / 1_000_000_000.0, 0.0)

  defp vec_to_tuple(%Vec3{} = v), do: {Vec3.x(v), Vec3.y(v), Vec3.z(v)}

  defp apply_correction(q, q_dot, {ax, ay, az}, state) do
    g = state.accel_threshold
    a_norm_g = :math.sqrt(ax * ax + ay * ay + az * az) / AhrsMath.gravity()

    if abs(a_norm_g - 1.0) > g do
      q_dot
    else
      {q_dot_w, q_dot_x, q_dot_y, q_dot_z} = q_dot
      a_norm_si = a_norm_g * AhrsMath.gravity()
      nax = ax / a_norm_si
      nay = ay / a_norm_si
      naz = az / a_norm_si

      {s_w, s_x, s_y, s_z} = gradient_descent_step(q, nax, nay, naz)

      {
        q_dot_w - state.beta * s_w,
        q_dot_x - state.beta * s_x,
        q_dot_y - state.beta * s_y,
        q_dot_z - state.beta * s_z
      }
    end
  end

  defp gradient_descent_step(q, ax, ay, az) do
    t2qw = 2.0 * q.w
    t2qx = 2.0 * q.x
    t2qy = 2.0 * q.y
    t2qz = 2.0 * q.z
    t4qw = 4.0 * q.w
    t4qx = 4.0 * q.x
    t4qy = 4.0 * q.y
    t8qx = 8.0 * q.x
    t8qy = 8.0 * q.y
    qwqw = q.w * q.w
    qxqx = q.x * q.x
    qyqy = q.y * q.y
    qzqz = q.z * q.z

    s_w = t4qw * qyqy + t2qy * ax + t4qw * qxqx - t2qx * ay

    s_x =
      t4qx * qzqz - t2qz * ax + 4.0 * qwqw * q.x - t2qw * ay - t4qx + t8qx * qxqx +
        t8qx * qyqy + t4qx * az

    s_y =
      4.0 * qwqw * q.y + t2qw * ax + t4qy * qzqz - t2qz * ay - t4qy + t8qy * qxqx +
        t8qy * qyqy + t4qy * az

    s_z = 4.0 * qxqx * q.z - t2qx * ax + 4.0 * qyqy * q.z - t2qy * ay

    s_norm = :math.sqrt(s_w * s_w + s_x * s_x + s_y * s_y + s_z * s_z)

    if s_norm > 0 do
      {s_w / s_norm, s_x / s_norm, s_y / s_norm, s_z / s_norm}
    else
      {0.0, 0.0, 0.0, 0.0}
    end
  end

  defp build_output_message(in_msg, in_imu, %__MODULE__{q: q}) do
    {:ok, msg} =
      Imu.new(in_msg.frame_id,
        orientation: Q.to_bb(q),
        angular_velocity: in_imu.angular_velocity,
        linear_acceleration: in_imu.linear_acceleration
      )

    msg
  end

  @doc """
  Convenience: returns the current orientation as a `BB.Math.Quaternion`.
  Useful for direct callers.
  """
  @spec orientation(t()) :: NxQuaternion.t()
  def orientation(%__MODULE__{q: q}), do: Q.to_bb(q)
end
