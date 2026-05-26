# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.Quaternion do
  @moduledoc """
  Represents a 3D orientation as a quaternion (w, x, y, z).
  """

  defstruct w: 1.0, x: 0.0, y: 0.0, z: 0.0

  @type t :: %__MODULE__{
          w: float(),
          x: float(),
          y: float(),
          z: float()
        }

  @doc """
  Normalizes a quaternion to a unit quaternion.
  """
  @spec normalize(t()) :: t()
  def normalize(%__MODULE__{w: w, x: x, y: y, z: z} = q) do
    # we have to prevent divide by zero when normalizing
    case :math.sqrt(w * w + x * x + y * y + z * z) do
      norm when norm == 0.0 -> q
      norm -> %__MODULE__{w: w / norm, x: x / norm, y: y / norm, z: z / norm}
    end
  end

  @doc """
  Returns the conjugate of a quaternion.
  For a unit quaternion, the conjugate is its inverse.
  """
  @spec conjugate(t()) :: t()
  def conjugate(%__MODULE__{w: w, x: x, y: y, z: z}) do
    %__MODULE__{w: w, x: -x, y: -y, z: -z}
  end

  @doc """
  Multiplies two quaternions.
  Used to combine rotations. Note that quaternion multiplication is non-commutative.
  """
  @spec multiply(t(), t()) :: t()
  def multiply(%__MODULE__{w: w1, x: x1, y: y1, z: z1}, %__MODULE__{w: w2, x: x2, y: y2, z: z2}) do
    %__MODULE__{
      w: w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
      x: w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      y: w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      z: w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    }
  end

  @doc """
  Calculates the rate of change (derivative) of a quaternion based on angular velocity (rad/s).

  Returns a 4-tuple of `{dw, dx, dy, dz}`.
  """
  @spec gyro_derivative(t(), float(), float(), float()) :: {float(), float(), float(), float()}
  def gyro_derivative(%__MODULE__{w: w, x: x, y: y, z: z}, gx, gy, gz) do
    {
      0.5 * (-x * gx - y * gy - z * gz),
      0.5 * (w * gx + y * gz - z * gy),
      0.5 * (w * gy - x * gz + z * gx),
      0.5 * (w * gz + x * gy - y * gx)
    }
  end
end
