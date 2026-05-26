# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.Mahony do
  @moduledoc """
  Implementation of the Mahony AHRS algorithm (6-DOF IMU variant).

  This filter uses a Proportional-Integral (PI) controller to compensate for
  gyroscope drift using accelerometer data. It is computationally lighter than
  Madgwick and highly robust.

  The state is wrapped in an `%Ahrs.Mahony{}` struct to track orientation,
  accumulated integral error (e_int), and time delta (dt).
  """

  @behaviour Ahrs.Algorithm

  alias Ahrs.Math
  alias Ahrs.Quaternion, as: Q

  defstruct q: %Q{}, e_int: {0.0, 0.0, 0.0}, last_update_at: nil

  @type t :: %__MODULE__{
          q: Q.t(),
          e_int: {float(), float(), float()},
          last_update_at: integer() | nil
        }

  @default_kp 2.0
  @default_ki 0.0
  @default_accel_threshold 0.1
  @default_e_int_limit 100.0

  @impl Ahrs.Algorithm
  def update(%__MODULE__{last_update_at: last_ts} = state, measurements, opts \\ []) do
    # Calculate dt
    {dt, current_ts} = Math.calculate_dt(last_ts, opts)

    case dt do
      # Initial run or no dt, just record timestamp
      nil ->
        %__MODULE__{state | last_update_at: current_ts}

      d when d == 0.0 ->
        %__MODULE__{state | last_update_at: current_ts}

      dt ->
        # Run math
        {new_q, new_e_int} = run_mahony(state, measurements, dt, opts)
        %__MODULE__{q: new_q, e_int: new_e_int, last_update_at: current_ts}
    end
  end

  defp run_mahony(
         %__MODULE__{q: q_in, e_int: {ex_int, ey_int, ez_int}},
         %{accel: accel, gyro: gyro},
         dt,
         opts
       ) do
    # Read algorithm options
    kp = Keyword.get(opts, :kp, @default_kp)
    ki = Keyword.get(opts, :ki, @default_ki)
    threshold = Keyword.get(opts, :accel_threshold, @default_accel_threshold)
    limit = Keyword.get(opts, :e_int_limit, @default_e_int_limit)

    # Note: Redundant initial normalization removed. Output is normalized at end of step.
    q = q_in

    # Normalize sensor inputs
    accel_g = Math.convert(accel, :g)
    gyro_rad = Math.convert(gyro, :rad_s)

    a_norm = :math.sqrt(accel_g.x * accel_g.x + accel_g.y * accel_g.y + accel_g.z * accel_g.z)

    # Prediction phase (gyro)
    {gx, gy, gz} = {gyro_rad.x, gyro_rad.y, gyro_rad.z}

    # Correction phase (accel)
    {gx, gy, gz, new_ex_int, new_ey_int, new_ez_int} =
      if abs(a_norm - 1.0) > threshold do
        # Ignore noisy accelerometer reading
        {gx, gy, gz, ex_int, ey_int, ez_int}
      else
        ax = accel_g.x / a_norm
        ay = accel_g.y / a_norm
        az = accel_g.z / a_norm

        # Estimated direction of gravity from quaternion (v)
        {vx, vy, vz} = estimate_gravity_direction(q)

        # Error is cross product between measured gravity (a) and estimated gravity (v)
        {ex, ey, ez} = cross_product({ax, ay, az}, {vx, vy, vz})

        # Compute and accumulate integral feedback with anti-windup clamping
        {nx_int, ny_int, nz_int} =
          if ki > 0.0 do
            {
              clamp(ex_int + ex * ki * dt, -limit, limit),
              clamp(ey_int + ey * ki * dt, -limit, limit),
              clamp(ez_int + ez * ki * dt, -limit, limit)
            }
          else
            # Persist accumulated bias even if ki is temporarily 0
            {ex_int, ey_int, ez_int}
          end

        # Apply PI feedback to gyro rates
        ngx = gx + kp * ex + nx_int
        ngy = gy + kp * ey + ny_int
        ngz = gz + kp * ez + nz_int

        {ngx, ngy, ngz, nx_int, ny_int, nz_int}
      end

    # Integrate corrected rates
    {q_dot_w, q_dot_x, q_dot_y, q_dot_z} = Q.gyro_derivative(q, gx, gy, gz)

    new_q = %Q{
      w: q.w + q_dot_w * dt,
      x: q.x + q_dot_x * dt,
      y: q.y + q_dot_y * dt,
      z: q.z + q_dot_z * dt
    }

    {Q.normalize(new_q), {new_ex_int, new_ey_int, new_ez_int}}
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

  defp clamp(val, min, max) do
    cond do
      val < min -> min
      val > max -> max
      true -> val
    end
  end
end
