<!--
SPDX-FileCopyrightText: 2026 Gus Workman

SPDX-License-Identifier: MIT
-->

# AHRS Examples & Simulator

This directory contains examples of how to use the `ahrs` library.

## 3D Visual Simulator

The `simulator.exs` script is a self-contained Phoenix LiveView application that
provides a real-time 3D visualization of the AHRS filter algorithms.

It generates simulated, noisy sensor data (Accelerometer and Gyroscope) based on
your keyboard input and passes it through the library's filters. The resulting
orientation is then rendered using Three.js.

### How to Run

From the root of the project, execute:

```bash
elixir examples/simulator.exs
```

Then open your browser to `http://localhost:4000`.

### Controls

- **Filter Selection:** Toggle between Madgwick, Mahony, and Complementary
  filters using the UI dropdown.
- **Noise Tuning:** Independently adjust the jitter for the simulated Gyroscope
  and Accelerometer.
- **Rotation:**
  - **W / S**: Pitch Up / Down
  - **A / D**: Roll Left / Right
  - **Q / E**: Yaw Left / Right (Relative)

---

## Credits

The 3D model used in the simulator is the classic **Utah Teapot**.

We would like to thank the **University of Utah Computer Graphics Department**
for providing this iconic model. More information about the teapot and its
history in computer graphics can be found here:
[https://graphics.cs.utah.edu/teapot/](https://graphics.cs.utah.edu/teapot/)
