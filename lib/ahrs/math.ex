# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.Math do
  @moduledoc """
  Stateless mathematical utilities for unit conversions, Euler angles, and tilt calculations.
  """

  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro
  alias Ahrs.Quaternion, as: Q

  @gravity 9.80665
  @gimbal_lock_threshold 0.99999

  @doc """
  Returns the gravitational constant used for unit conversions (9.80665 m/s²).
  """
  def gravity, do: @gravity

  @doc """
  Calculates the time delta (dt) between updates.

  Returns a `{dt_seconds, current_ts}` tuple.
  """
  @spec calculate_dt(integer() | nil, keyword()) :: {float() | nil, integer()}
  def calculate_dt(last_ts, opts) do
    current_time = System.monotonic_time(:microsecond)

    case Keyword.get(opts, :dt) do
      dt when is_number(dt) ->
        {dt, current_time}

      nil ->
        if is_nil(last_ts) do
          {nil, current_time}
        else
          dt_seconds = (current_time - last_ts) / 1_000_000.0
          {dt_seconds, current_time}
        end
    end
  end

  @doc """
  Converts a sensor sample to the specified units.
  """
  @spec convert(Accel.t() | Gyro.t(), atom()) :: Accel.t() | Gyro.t()
  def convert(%Accel{units: units} = sample, units), do: sample

  def convert(%Accel{x: x, y: y, z: z, units: :g}, :m_s2) do
    %Accel{x: x * @gravity, y: y * @gravity, z: z * @gravity, units: :m_s2}
  end

  def convert(%Accel{x: x, y: y, z: z, units: :m_s2}, :g) do
    %Accel{x: x / @gravity, y: y / @gravity, z: z / @gravity, units: :g}
  end

  def convert(%Gyro{units: units} = sample, units), do: sample

  def convert(%Gyro{x: x, y: y, z: z, units: :deg_s}, :rad_s) do
    factor = :math.pi() / 180.0
    %Gyro{x: x * factor, y: y * factor, z: z * factor, units: :rad_s}
  end

  def convert(%Gyro{x: x, y: y, z: z, units: :rad_s}, :deg_s) do
    factor = 180.0 / :math.pi()
    %Gyro{x: x * factor, y: y * factor, z: z * factor, units: :deg_s}
  end

  @doc """
  Converts a quaternion orientation into Euler angles.
  Returns a tuple: {roll, pitch, yaw}
  Uses the standard Z-Y-X Tait-Bryan convention.

  ## Options
    * `:units` - can be `:radians` (default) or `:degrees`.
  """
  @spec quaternion_to_euler(Q.t(), keyword()) ::
          {roll :: float(), pitch :: float(), yaw :: float()}
  def quaternion_to_euler(%Q{w: w, x: x, y: y, z: z}, opts \\ []) do
    units = Keyword.get(opts, :units, :radians)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)

    {roll, pitch, yaw} =
      if abs(sinp) >= @gimbal_lock_threshold do
        # Gimbal lock (Pitch = +/- 90 degrees)
        p = if sinp < 0, do: -:math.pi() / 2, else: :math.pi() / 2
        # In gimbal lock, we override roll and yaw to stay stable
        r = normalize_angle(2.0 * :math.atan2(x, w))
        {r, p, 0.0}
      else
        # Standard Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        r = :math.atan2(sinr_cosp, cosr_cosp)

        # Standard Pitch (y-axis rotation)
        p = :math.asin(max(-1.0, min(1.0, sinp)))

        # Standard Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        y = :math.atan2(siny_cosp, cosy_cosp)

        {r, p, y}
      end

    format_output({roll, pitch, yaw}, units)
  end

  @doc """
  Converts Euler angles (radians) into a quaternion.
  Uses the standard Z-Y-X Tait-Bryan convention.
  """
  @spec euler_to_quaternion(roll :: float(), pitch :: float(), yaw :: float()) :: Q.t()
  def euler_to_quaternion(roll, pitch, yaw) do
    cr = :math.cos(roll * 0.5)
    sr = :math.sin(roll * 0.5)
    cp = :math.cos(pitch * 0.5)
    sp = :math.sin(pitch * 0.5)
    cy = :math.cos(yaw * 0.5)
    sy = :math.sin(yaw * 0.5)

    %Q{
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    }
  end

  defp format_output({r, p, y}, :radians), do: {r, p, y}

  defp format_output({r, p, y}, :degrees) do
    factor = 180.0 / :math.pi()
    {r * factor, p * factor, y * factor}
  end

  @doc """
  Calculates pitch and roll (radians) directly from an accelerometer sample using trigonometry.
  Note: This calculation is susceptible to linear acceleration and cannot determine yaw.
  Returns a tuple: {roll, pitch}
  """
  @spec accel_to_tilt(Accel.t()) :: {roll :: float(), pitch :: float()}
  def accel_to_tilt(%Accel{x: ax, y: ay, z: az}) do
    case :math.sqrt(ax * ax + ay * ay + az * az) do
      norm when norm == 0.0 ->
        # Free-fall or zero reading: Orientation is unknown, return zeroed angles
        {0.0, 0.0}

      _norm ->
        roll = :math.atan2(ay, az)
        pitch = :math.atan2(-ax, :math.sqrt(ay * ay + az * az))
        {roll, pitch}
    end
  end

  @doc """
  Rotates a 3D vector by a quaternion.
  This is mathematically equivalent to `q * v * conjugate(q)` where `v` is a pure quaternion `[0, x, y, z]`.
  Returns a `{x, y, z}` tuple.
  """
  @spec rotate_vector({float(), float(), float()}, Q.t()) :: {float(), float(), float()}
  def rotate_vector({vx, vy, vz}, %Q{w: qw, x: qx, y: qy, z: qz}) do
    # Optimized explicit formula for rotating a vector by a quaternion
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # v' = v + q.w * t + cross(q.xyz, t)
    {
      vx + qw * tx + qy * tz - qz * ty,
      vy + qw * ty + qz * tx - qx * tz,
      vz + qw * tz + qx * ty - qy * tx
    }
  end

  @doc """
  Normalizes an angle to the range (-PI, PI].
  """
  def normalize_angle(angle) do
    two_pi = 2.0 * :math.pi()
    a = :math.fmod(angle + :math.pi(), two_pi)
    if a <= 0, do: a + :math.pi(), else: a - :math.pi()
  end
end
