# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Ahrs.Math do
  @moduledoc """
  Stateless mathematical utilities shared across the AHRS filters.

  All inputs and outputs are in SI units (rad/s for angular velocity,
  m/s² for linear acceleration, radians for angles). Unit conversion is
  the responsibility of the sensor driver populating the inbound
  `BB.Message.Sensor.Imu` payload.

  Ported from [gworkman/ahrs](https://github.com/gworkman/ahrs)'
  `Ahrs.Math`.
  """

  alias BB.Ahrs.Quaternion, as: Q

  @gravity 9.80665
  @gimbal_lock_threshold 0.99999

  @doc "Standard gravity in m/s²."
  @spec gravity() :: float()
  def gravity, do: @gravity

  @doc """
  Converts a quaternion to Euler angles using the Z-Y-X Tait-Bryan
  convention. Returns `{roll, pitch, yaw}` in radians, unless `:degrees`
  is passed.
  """
  @spec quaternion_to_euler(Q.t(), keyword()) ::
          {roll :: float(), pitch :: float(), yaw :: float()}
  def quaternion_to_euler(%Q{w: w, x: x, y: y, z: z}, opts \\ []) do
    units = Keyword.get(opts, :units, :radians)

    sinp = 2.0 * (w * y - z * x)

    {roll, pitch, yaw} =
      if abs(sinp) >= @gimbal_lock_threshold do
        p = if sinp < 0, do: -:math.pi() / 2, else: :math.pi() / 2
        r = normalize_angle(2.0 * :math.atan2(x, w))
        {r, p, 0.0}
      else
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        r = :math.atan2(sinr_cosp, cosr_cosp)

        p = :math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        y = :math.atan2(siny_cosp, cosy_cosp)

        {r, p, y}
      end

    format_output({roll, pitch, yaw}, units)
  end

  @doc "Converts Euler angles in radians to a quaternion (Z-Y-X Tait-Bryan)."
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
  Calculates roll and pitch in radians from a 3-axis acceleration
  vector. Yaw is unobservable from acceleration alone.

  Returns `{roll, pitch}`. The accel components are in any unit; only
  their relative magnitudes matter.
  """
  @spec accel_to_tilt(float(), float(), float()) :: {roll :: float(), pitch :: float()}
  def accel_to_tilt(ax, ay, az) do
    case :math.sqrt(ax * ax + ay * ay + az * az) do
      norm when norm == 0.0 ->
        {0.0, 0.0}

      _norm ->
        roll = :math.atan2(ay, az)
        pitch = :math.atan2(-ax, :math.sqrt(ay * ay + az * az))
        {roll, pitch}
    end
  end

  @doc """
  Rotates a 3D `{x, y, z}` vector by a quaternion. Equivalent to
  `q * v * conjugate(q)`.
  """
  @spec rotate_vector({float(), float(), float()}, Q.t()) :: {float(), float(), float()}
  def rotate_vector({vx, vy, vz}, %Q{w: qw, x: qx, y: qy, z: qz}) do
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    {
      vx + qw * tx + qy * tz - qz * ty,
      vy + qw * ty + qz * tx - qx * tz,
      vz + qw * tz + qx * ty - qy * tx
    }
  end

  @doc "Normalises an angle to the range `(-π, π]`."
  @spec normalize_angle(float()) :: float()
  def normalize_angle(angle) do
    two_pi = 2.0 * :math.pi()
    a = :math.fmod(angle + :math.pi(), two_pi)
    if a <= 0, do: a + :math.pi(), else: a - :math.pi()
  end

  @doc """
  Returns `true` if the magnitude of an accel vector in m/s² is within
  `threshold` of standard gravity, where `threshold` is a fractional
  value (e.g. `0.1` means ±10% around 1 g).

  Used by Madgwick and Mahony to reject readings dominated by linear
  acceleration rather than gravity.
  """
  @spec gravity_only?(float(), float(), float(), float()) :: boolean()
  def gravity_only?(ax, ay, az, threshold) do
    norm_g = :math.sqrt(ax * ax + ay * ay + az * az) / @gravity
    abs(norm_g - 1.0) <= threshold
  end
end
