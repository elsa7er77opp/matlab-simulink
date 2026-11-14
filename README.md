# Modelling, Control and Wireless Operation of a 2WD Mobile Robot

The model couples armature voltage, motor current, wheel speed and chassis motion. Forward kinematics maps wheel speeds to body velocity and yaw rate. Inverse kinematics derives wheel references from a requested body motion.

## Engineering scope

The repository implements state-space analysis, closed-loop PI simulation, digital RST polynomial design and pole inspection. Simulink and Multibody builders assemble models from scripts and STEP components. The firmware generator produces an ESP32 sketch with a wireless interface for commands and telemetry. Some physical parameters still use estimates.

## Getting started

From MATLAB, change directory to `MATLAB` and run the example below. Simulink and Simscape Multibody are required for their respective builders. `generate_esp32_code` generates the wireless firmware. Set a device-specific access-point passphrase before flashing.

```matlab
P = robot_params(false);
S = robot_ss(P);
ramp = simulate_robot(P, S, 'ramp');
square = simulate_robot(P, S, 'square');
plot(ramp.t, [ramp.vref ramp.v]);
legend('Reference', 'Simulated speed');
```

## Status and verification

Simulation verified for two scenarios. Physical testing in progress.

MATLAB R2024b executed both `ramp` and `square`. Assertions checked finite velocity and position, positive forward displacement and nonempty histories (900 and 1,583 samples). RST performance, multibody animation and physical tracking were not measured in this review.

## Source and evidence

- [robot_ss.m](MATLAB/robot_ss.m)
- [robot_kinematics.m](MATLAB/robot_kinematics.m)
- [plot_poles.m](MATLAB/plot_poles.m)
- [rst_design.m](MATLAB/rst_design.m)
- [build_robot_sim.m](MATLAB/build_robot_sim.m)
- [build_robot_simscape.m](MATLAB/Simscape_STEP/build_robot_simscape.m)
- [ESP32_Robot.ino](ESP32_Robot/ESP32_Robot.ino)
- [README_AR.md](README_AR.md)

## Author

Ahmed Mohamed Elsayed Elreefy. Engineering work includes requirements, technical decisions, AI-assisted implementation, debugging and acceptance review.
