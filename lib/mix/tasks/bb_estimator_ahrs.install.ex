# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

if Code.ensure_loaded?(Igniter) do
  defmodule Mix.Tasks.BbEstimatorAhrs.Install do
    @shortdoc "Installs BB.Estimator.Ahrs into a project"
    @moduledoc """
    #{@shortdoc}

    AHRS estimators are nested inside IMU sensors in the robot topology,
    so this installer prints a snippet for the chosen algorithm rather
    than editing the topology directly.

    ## Example

    ```bash
    mix igniter.install bb_estimator_ahrs
    mix igniter.install bb_estimator_ahrs --algorithm mahony
    ```

    ## Options

    * `--algorithm` - Which algorithm to demonstrate in the snippet:
      `madgwick` (default), `mahony`, or `complementary`.
    """

    use Igniter.Mix.Task

    @algorithms ~w(madgwick mahony complementary)
    @default_algorithm "madgwick"

    @impl Igniter.Mix.Task
    def info(_argv, _parent) do
      %Igniter.Mix.Task.Info{
        schema: [algorithm: :string],
        aliases: [a: :algorithm]
      }
    end

    @impl Igniter.Mix.Task
    def igniter(igniter) do
      algorithm =
        igniter.args.options
        |> Keyword.get(:algorithm, @default_algorithm)
        |> validate_algorithm()

      Igniter.add_notice(igniter, notice(algorithm))
    end

    defp validate_algorithm(algorithm) when algorithm in @algorithms, do: algorithm

    defp validate_algorithm(algorithm) do
      Mix.raise(
        "Unknown algorithm #{inspect(algorithm)}. Expected one of: #{Enum.join(@algorithms, ", ")}"
      )
    end

    defp notice("madgwick"), do: notice_for("BB.Estimator.Ahrs.Madgwick", "beta: 0.1")
    defp notice("mahony"), do: notice_for("BB.Estimator.Ahrs.Mahony", "kp: 2.0, ki: 0.005")
    defp notice("complementary"), do: notice_for("BB.Estimator.Ahrs.Complementary", "alpha: 0.98")

    defp notice_for(module, options) do
      """
      bb_estimator_ahrs: nest an estimator inside an IMU sensor:

          sensor :imu, BB.Sensor.SomeImu, bus: "i2c-1", address: 0x68 do
            estimator :orientation, {#{module}, #{options}}
          end

      Subscribers to [:sensor, <link>, :imu, :orientation] receive the fused
      orientation; [:sensor, <link>, :imu] still sees the raw IMU.

      Other algorithms: BB.Estimator.Ahrs.Madgwick (beta),
      BB.Estimator.Ahrs.Mahony (kp/ki),
      BB.Estimator.Ahrs.Complementary (alpha or time_constant).
      """
    end
  end
else
  defmodule Mix.Tasks.BbEstimatorAhrs.Install do
    @shortdoc "Installs BB.Estimator.Ahrs into a project"
    @moduledoc false
    use Mix.Task

    def run(_argv) do
      Mix.shell().error("""
      The bb_estimator_ahrs.install task requires igniter. Please install igniter and try again.

          mix igniter.install bb_estimator_ahrs
      """)

      exit({:shutdown, 1})
    end
  end
end
