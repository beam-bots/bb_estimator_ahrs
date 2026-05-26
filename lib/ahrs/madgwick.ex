# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.Madgwick do
  @moduledoc """
  Implementation of the Madgwick AHRS algorithm (6-DOF IMU variant).

  This filter uses a gradient descent algorithm to compute the direction of the
  gyroscope measurement error as a quaternion derivative.

  The state is wrapped in an `%Ahrs.Madgwick{}` struct to automatically handle
  time delta (dt) tracking using system monotonic time.
  """

  @behaviour Ahrs.Algorithm

  alias Ahrs.Math
  alias Ahrs.Quaternion, as: Q

  defstruct q: %Q{}, last_update_at: nil

  @type t :: %__MODULE__{
          q: Q.t(),
          last_update_at: integer() | nil
        }

  @default_beta 0.1
  @default_accel_threshold 0.1

  @doc """
  Updates the Madgwick filter state with new sensor measurements.

  ## Options
    * `:dt` - Explicit delta time in seconds. If omitted, the library automatically
      calculates the delta using system monotonic time.
    * `:beta` - Filter gain (default 0.1).
    * `:accel_threshold` - Acceptable deviation from 1G for accelerometer readings (default 0.1).
      Readings outside [1.0 - threshold, 1.0 + threshold] are ignored.
  """
  @impl Ahrs.Algorithm
  def update(%__MODULE__{last_update_at: last_ts} = state, measurements, opts \\ []) do
    beta = Keyword.get(opts, :beta, @default_beta)
    threshold = Keyword.get(opts, :accel_threshold, @default_accel_threshold)

    # Calculate dt
    {dt, current_ts} = Math.calculate_dt(last_ts, opts)

    case dt do
      # Initial run or no dt provided/calculated, just record timestamp
      nil ->
        %__MODULE__{state | last_update_at: current_ts}

      d when d == 0.0 ->
        # No time elapsed, but we must update the timestamp
        %__MODULE__{state | last_update_at: current_ts}

      dt ->
        # Run math
        new_q = run_madgwick(state.q, measurements, dt, beta, threshold)
        %__MODULE__{q: new_q, last_update_at: current_ts}
    end
  end

  defp run_madgwick(q_in, %{accel: accel, gyro: gyro}, dt, beta, threshold) do
    # Ensure safe math by starting with a normalized quaternion
    q = Q.normalize(q_in)

    # 1. Prediction Step: Calculate gyro derivative
    gyro_rad = Math.convert(gyro, :rad_s)

    {q_dot_w, q_dot_x, q_dot_y, q_dot_z} =
      Q.gyro_derivative(q, gyro_rad.x, gyro_rad.y, gyro_rad.z)

    # 2. Correction Step: Apply gradient descent feedback
    # We ignore the accelerometer if the magnitude deviates significantly from 1G
    accel_g = Math.convert(accel, :g)
    a_norm = :math.sqrt(accel_g.x * accel_g.x + accel_g.y * accel_g.y + accel_g.z * accel_g.z)

    {q_dot_w, q_dot_x, q_dot_y, q_dot_z} =
      if abs(a_norm - 1.0) > threshold do
        # Ignore noisy accelerometer reading
        {q_dot_w, q_dot_x, q_dot_y, q_dot_z}
      else
        ax = accel_g.x / a_norm
        ay = accel_g.y / a_norm
        az = accel_g.z / a_norm

        {s_w, s_x, s_y, s_z} = compute_gradient_descent(q, ax, ay, az)

        {
          q_dot_w - beta * s_w,
          q_dot_x - beta * s_x,
          q_dot_y - beta * s_y,
          q_dot_z - beta * s_z
        }
      end

    # 3. Integration Step
    new_q = %Q{
      w: q.w + q_dot_w * dt,
      x: q.x + q_dot_x * dt,
      y: q.y + q_dot_y * dt,
      z: q.z + q_dot_z * dt
    }

    Q.normalize(new_q)
  end

  defp compute_gradient_descent(q, ax, ay, az) do
    # Pre-compute common terms
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

    # Objective function f and Jacobian J
    s_w = t4qw * qyqy + t2qy * ax + t4qw * qxqx - t2qx * ay

    s_x =
      t4qx * qzqz - t2qz * ax + 4.0 * qwqw * q.x - t2qw * ay - t4qx + t8qx * qxqx + t8qx * qyqy +
        t4qx * az

    s_y =
      4.0 * qwqw * q.y + t2qw * ax + t4qy * qzqz - t2qz * ay - t4qy + t8qy * qxqx + t8qy * qyqy +
        t4qy * az

    s_z = 4.0 * qxqx * q.z - t2qx * ax + 4.0 * qyqy * q.z - t2qy * ay

    # Normalize step magnitude
    s_norm = :math.sqrt(s_w * s_w + s_x * s_x + s_y * s_y + s_z * s_z)

    if s_norm > 0 do
      {s_w / s_norm, s_x / s_norm, s_y / s_norm, s_z / s_norm}
    else
      {0.0, 0.0, 0.0, 0.0}
    end
  end
end
