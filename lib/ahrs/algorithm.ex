# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.Algorithm do
  @moduledoc """
  Defines the behavior for AHRS algorithms.
  """

  alias Ahrs.Accelerometer.Sample, as: Accel
  alias Ahrs.Gyroscope.Sample, as: Gyro

  @type measurements :: %{
          accel: Accel.t(),
          gyro: Gyro.t()
        }

  @doc """
  Updates the filter state based on new sensor measurements.

  - `state`: The current state of the filter (usually a struct containing a quaternion and metadata).
  - `measurements`: A map containing accelerometer and gyroscope samples.
  - `opts`: Algorithm-specific tuning options (e.g., `:beta`, `:dt`).

  Returns the updated state struct.
  """
  @callback update(state :: struct(), measurements(), opts :: keyword()) :: struct()
end
