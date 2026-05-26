# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.Complementary do
  @moduledoc """
  Implementation of a simple Complementary Filter (6-DOF IMU variant).

  This filter combines high-pass integrated gyroscope data with a low-pass
  accelerometer-based tilt calculation. It is extremely simple, computationally
  cheap, and robust against short-term vibrations.
  """

  @behaviour Ahrs.Algorithm

  alias Ahrs.Math
  alias Ahrs.Quaternion, as: Q

  defstruct q: %Q{}, last_update_at: nil

  @type t :: %__MODULE__{
          q: Q.t(),
          last_update_at: integer() | nil
        }

  @default_alpha 0.98
  @default_accel_threshold 0.1

  @doc """
  Updates the Complementary filter state with new sensor measurements.

  ## Options
    * `:dt` - Explicit delta time in seconds.
    * `:alpha` - Fixed gyroscope weight (default 0.98).
    * `:time_constant` - Optional time constant (tau) in seconds. If provided,
      alpha is calculated as tau / (tau + dt), making the filter frequency-independent.
    * `:accel_threshold` - Minimum acceleration magnitude (G) to apply correction (default 0.1).
  """
  @impl Ahrs.Algorithm
  def update(%__MODULE__{last_update_at: last_ts} = state, measurements, opts \\ []) do
    # Calculate dt
    {dt, current_ts} = Math.calculate_dt(last_ts, opts)

    case dt do
      nil ->
        %__MODULE__{state | last_update_at: current_ts}

      d when d == 0.0 ->
        %__MODULE__{state | last_update_at: current_ts}

      dt ->
        new_q = run_complementary(state.q, measurements, dt, opts)
        %__MODULE__{q: new_q, last_update_at: current_ts}
    end
  end

  defp run_complementary(q_in, %{accel: accel, gyro: gyro}, dt, opts) do
    q = Q.normalize(q_in)

    # 1. Prediction: Integrate gyroscope (High Pass)
    gyro_rad = Math.convert(gyro, :rad_s)

    {q_dot_w, q_dot_x, q_dot_y, q_dot_z} =
      Q.gyro_derivative(q, gyro_rad.x, gyro_rad.y, gyro_rad.z)

    q_gyro =
      %Q{
        w: q.w + q_dot_w * dt,
        x: q.x + q_dot_x * dt,
        y: q.y + q_dot_y * dt,
        z: q.z + q_dot_z * dt
      }
      |> Q.normalize()

    # Determine weighting (alpha)
    alpha = calculate_alpha(dt, opts)
    threshold = Keyword.get(opts, :accel_threshold, @default_accel_threshold)

    # 2. Correction: Calculate tilt from accelerometer (Low Pass)
    accel_g = Math.convert(accel, :g)
    a_norm = :math.sqrt(accel_g.x * accel_g.x + accel_g.y * accel_g.y + accel_g.z * accel_g.z)

    if abs(a_norm - 1.0) > threshold do
      # Ignore noisy accelerometer reading
      q_gyro
    else
      # Extract current yaw from integrated state to prevent "Yaw Leakage"
      {_roll, _pitch, yaw} = Math.quaternion_to_euler(q_gyro)
      {roll, pitch} = Math.accel_to_tilt(accel)

      # Construct correction quaternion preserving current heading
      q_accel = Math.euler_to_quaternion(roll, pitch, yaw)

      # 3. Combine using alpha weighting
      one_minus_alpha = 1.0 - alpha

      # Short-path interpolation check
      dot =
        q_gyro.w * q_accel.w + q_gyro.x * q_accel.x + q_gyro.y * q_accel.y + q_gyro.z * q_accel.z

      q_accel =
        if dot < 0.0,
          do: %Q{w: -q_accel.w, x: -q_accel.x, y: -q_accel.y, z: -q_accel.z},
          else: q_accel

      res = %Q{
        w: alpha * q_gyro.w + one_minus_alpha * q_accel.w,
        x: alpha * q_gyro.x + one_minus_alpha * q_accel.x,
        y: alpha * q_gyro.y + one_minus_alpha * q_accel.y,
        z: alpha * q_gyro.z + one_minus_alpha * q_accel.z
      }

      Q.normalize(res)
    end
  end

  defp calculate_alpha(dt, opts) do
    case opts[:time_constant] do
      tau when is_number(tau) ->
        denom = tau + dt
        if denom == 0.0, do: 1.0, else: tau / denom

      _ ->
        Keyword.get(opts, :alpha, @default_alpha)
    end
  end
end
