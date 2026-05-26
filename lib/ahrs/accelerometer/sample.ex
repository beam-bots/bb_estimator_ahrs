# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.Accelerometer.Sample do
  @moduledoc """
  Strongly typed container for accelerometer data.
  """

  @enforce_keys [:x, :y, :z, :units]
  defstruct [:x, :y, :z, :units]

  @type t :: %__MODULE__{
          x: float(),
          y: float(),
          z: float(),
          units: :g | :m_s2
        }
end
