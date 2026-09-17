# Pi Arm Output-Shaft Visualizer

This package visualizes the arm pose from six **motor output-shaft angles** without
opening SocketCAN or controlling hardware.

The interactive path is:

```text
motor output-shaft sliders (rad)
  -> /pi_arm/model_output_shaft_states
  -> pi_arm_transmission::output_to_joint_position()
  -> six active ROS joint positions
  -> /joint_states
  -> robot_state_publisher
  -> RViz
```

Input names are fixed to:

```text
motor_01_output, motor_02_output, motor_03_output,
motor_04_output, motor_05_output, motor_06_output
```

Output active-joint order is fixed to:

```text
joint_01, joint_02, joint_04, joint_05, joint_06, joint_09
```

Angles follow `sensor_msgs/JointState` conventions and are in radians. The input
is explicitly **model_output**, i.e. the reducer-output coordinate used by the
`model_version: 3` cable model. Do not publish ROM-relative feedback directly.
ROM-relative values must first be converted with an approved, fingerprint-matched
calibration using `model_output = rom_relative_output + software_offset`. The GUI
slider ranges are generated from `pi_arm_description/config/cable_kinematics.yaml`.
The mapper rejects a complete sample if any output shaft exceeds its configured
range; it never clamps one motor independently.

The configured ranges are theoretical zero-relative envelopes derived from the
joint limits and cable model. They are not a substitute for low-speed live
acceptance testing of cable tension and actual travel.

## Build and run on Ubuntu 24.04 / ROS 2 Jazzy

```bash
cd ~/pi_arm
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pi_arm_output_visualizer
source install/setup.bash

ros2 launch pi_arm_output_visualizer output_shaft_visualizer.launch.py
```

Do not run `pi_arm_description/display.launch.py` at the same time because its
GUI also publishes `/joint_states`.

If a combination of output-shaft angles is outside the inverse model domain
(notably the chord models for axes 4 and 5), the converter logs a warning and
keeps the last valid RViz pose.

The converter can also be driven without the GUI by publishing all six named
positions to `/pi_arm/model_output_shaft_states`:

```bash
ros2 launch pi_arm_output_visualizer output_shaft_visualizer.launch.py use_gui:=false
```

In another sourced terminal, publish one complete output-shaft state in radians:

```bash
ros2 topic pub --once /pi_arm/model_output_shaft_states sensor_msgs/msg/JointState \
  "{name: [motor_01_output, motor_02_output, motor_03_output, motor_04_output, motor_05_output, motor_06_output], position: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
