# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Estimator.Ahrs.Complementary do
  @moduledoc """
  Complementary filter for 6-DOF IMUs, implemented as a `BB.Estimator`.

  Integrates the gyroscope (high-pass) and blends in an accelerometer-
  derived tilt estimate (low-pass) using a fixed weight or a time
  constant.

  ## Usage

      sensor :imu, BB.Sensor.SomeImu, ... do
        estimator :orientation, {BB.Estimator.Ahrs.Complementary, alpha: 0.98}
      end

  ## Options

  - `:alpha` — fixed gyro weight, `0.0..1.0`. Default `0.98` (98% gyro,
    2% accel per update).
  - `:time_constant` — optional time constant τ in seconds. When set,
    overrides `:alpha` with `τ / (τ + dt)`, making the filter
    frequency-independent.
  - `:accel_threshold` — accepted deviation of `|accel| / g` from `1.0`
    before the correction is suppressed. Default `0.1`.

  Ported from [gworkman/ahrs](https://github.com/gworkman/ahrs)'
  `Ahrs.Complementary`.
  """

  use BB.Estimator,
    options_schema: [
      alpha: [type: :float, required: false, default: 0.98, doc: "Fixed gyro weight"],
      time_constant: [
        type: {:or, [:float, nil]},
        required: false,
        default: nil,
        doc: "Optional τ in seconds; overrides :alpha when set"
      ],
      accel_threshold: [
        type: :float,
        required: false,
        default: 0.1,
        doc: "Maximum fractional deviation of accelerometer magnitude from 1 g"
      ]
    ]

  alias BB.Estimator.Ahrs.Math, as: AhrsMath
  alias BB.Estimator.Ahrs.Quaternion, as: Q
  alias BB.Math.Vec3
  alias BB.Message
  alias BB.Message.Sensor.Imu

  defstruct q: %Q{},
            last_monotonic_time: nil,
            alpha: 0.98,
            time_constant: nil,
            accel_threshold: 0.1

  @type t :: %__MODULE__{
          q: Q.t(),
          last_monotonic_time: integer() | nil,
          alpha: float(),
          time_constant: nil | float(),
          accel_threshold: float()
        }

  # ----------------------------------------------------------------------------
  # BB.Estimator callbacks
  # ----------------------------------------------------------------------------

  @impl BB.Estimator
  def init(opts) do
    {:ok,
     %__MODULE__{
       alpha: Keyword.fetch!(opts, :alpha),
       time_constant: Keyword.fetch!(opts, :time_constant),
       accel_threshold: Keyword.fetch!(opts, :accel_threshold)
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
  # Pure step
  # ----------------------------------------------------------------------------

  @doc "Runs one Complementary filter step."
  @spec step(t(), {float(), float(), float()}, {float(), float(), float()}, float()) :: t()
  def step(%__MODULE__{} = state, _gyro, _accel, dt) when dt <= 0.0, do: state

  def step(%__MODULE__{q: q_in} = state, gyro, accel, dt) do
    q = Q.normalise(q_in)

    q_gyro = integrate_gyro(q, gyro, dt)
    new_q = apply_correction(q_gyro, accel, state, dt)

    %{state | q: new_q}
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

  defp integrate_gyro(q, {gx, gy, gz}, dt) do
    {q_dot_w, q_dot_x, q_dot_y, q_dot_z} = Q.gyro_derivative(q, gx, gy, gz)

    Q.normalise(%Q{
      w: q.w + q_dot_w * dt,
      x: q.x + q_dot_x * dt,
      y: q.y + q_dot_y * dt,
      z: q.z + q_dot_z * dt
    })
  end

  defp apply_correction(q_gyro, {ax, ay, az}, state, dt) do
    a_norm_g = :math.sqrt(ax * ax + ay * ay + az * az) / AhrsMath.gravity()

    if abs(a_norm_g - 1.0) > state.accel_threshold do
      q_gyro
    else
      {_roll_g, _pitch_g, yaw_g} = AhrsMath.quaternion_to_euler(q_gyro)
      {roll_a, pitch_a} = AhrsMath.accel_to_tilt(ax, ay, az)
      q_accel = AhrsMath.euler_to_quaternion(roll_a, pitch_a, yaw_g)

      alpha = effective_alpha(state, dt)
      blend(q_gyro, q_accel, alpha)
    end
  end

  defp effective_alpha(%__MODULE__{time_constant: nil, alpha: a}, _dt), do: a

  defp effective_alpha(%__MODULE__{time_constant: tau}, dt) do
    denom = tau + dt
    if denom == 0.0, do: 1.0, else: tau / denom
  end

  defp blend(q_gyro, q_accel, alpha) do
    one_minus = 1.0 - alpha

    dot =
      q_gyro.w * q_accel.w + q_gyro.x * q_accel.x + q_gyro.y * q_accel.y + q_gyro.z * q_accel.z

    q_accel =
      if dot < 0.0,
        do: %Q{w: -q_accel.w, x: -q_accel.x, y: -q_accel.y, z: -q_accel.z},
        else: q_accel

    Q.normalise(%Q{
      w: alpha * q_gyro.w + one_minus * q_accel.w,
      x: alpha * q_gyro.x + one_minus * q_accel.x,
      y: alpha * q_gyro.y + one_minus * q_accel.y,
      z: alpha * q_gyro.z + one_minus * q_accel.z
    })
  end
end
