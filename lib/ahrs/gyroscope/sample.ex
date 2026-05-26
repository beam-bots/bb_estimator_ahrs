# SPDX-FileCopyrightText: 2026 Gus Workman
#
# SPDX-License-Identifier: MIT

defmodule Ahrs.Gyroscope.Sample do
  @moduledoc """
  Strongly typed container for gyroscope data.
  """

  @enforce_keys [:x, :y, :z, :units]
  defstruct [:x, :y, :z, :units]

  @type t :: %__MODULE__{
          x: float(),
          y: float(),
          z: float(),
          units: :rad_s | :deg_s
        }
end
