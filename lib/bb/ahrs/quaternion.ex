# SPDX-FileCopyrightText: 2026 Gus Workman
# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: MIT

defmodule BB.Ahrs.Quaternion do
  @moduledoc """
  Scalar (w, x, y, z) quaternion used internally by the AHRS algorithms.

  Kept separate from `BB.Math.Quaternion` (Nx-backed) because the AHRS
  filters run at hundreds of Hz and the Nx dispatch overhead per
  operation dominates the actual arithmetic. Conversion to
  `BB.Math.Quaternion` happens at each estimator's input / output
  boundary.

  Ported from
  [gworkman/ahrs](https://github.com/gworkman/ahrs)' `Ahrs.Quaternion`.
  """

  defstruct w: 1.0, x: 0.0, y: 0.0, z: 0.0

  @type t :: %__MODULE__{
          w: float(),
          x: float(),
          y: float(),
          z: float()
        }

  alias BB.Math.Quaternion, as: NxQuaternion

  @doc """
  Normalises a quaternion to a unit quaternion. Returns the input
  unchanged when its norm is zero.
  """
  @spec normalise(t()) :: t()
  def normalise(%__MODULE__{w: w, x: x, y: y, z: z} = q) do
    case :math.sqrt(w * w + x * x + y * y + z * z) do
      norm when norm == 0.0 -> q
      norm -> %__MODULE__{w: w / norm, x: x / norm, y: y / norm, z: z / norm}
    end
  end

  @doc "Returns the conjugate of a unit quaternion (also its inverse)."
  @spec conjugate(t()) :: t()
  def conjugate(%__MODULE__{w: w, x: x, y: y, z: z}) do
    %__MODULE__{w: w, x: -x, y: -y, z: -z}
  end

  @doc """
  Hamilton-product multiplication. Non-commutative; composes the
  right-hand rotation first, then the left.
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
  Calculates `q̇`, the time-derivative of a quaternion under angular
  velocity `(gx, gy, gz)` in rad/s. Returns a `{ẇ, ẋ, ẏ, ż}` 4-tuple.
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

  @doc "Identity quaternion (no rotation)."
  @spec identity() :: t()
  def identity, do: %__MODULE__{}

  @doc "Converts to a `BB.Math.Quaternion` for use in a message payload."
  @spec to_bb(t()) :: NxQuaternion.t()
  def to_bb(%__MODULE__{w: w, x: x, y: y, z: z}), do: NxQuaternion.new(w, x, y, z)

  @doc "Constructs from a `BB.Math.Quaternion`."
  @spec from_bb(NxQuaternion.t()) :: t()
  def from_bb(%NxQuaternion{} = q) do
    %__MODULE__{
      w: NxQuaternion.w(q),
      x: NxQuaternion.x(q),
      y: NxQuaternion.y(q),
      z: NxQuaternion.z(q)
    }
  end
end
