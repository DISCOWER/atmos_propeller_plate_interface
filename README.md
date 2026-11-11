# ATMOS Propeller Plate Interface

This ROS 2 package converts requested propeller forces into normalized actuator commands for PX4. It is intended for the ATMOS propeller plate, one possible actuation layer for ATMOS, a frictionless free-flyer used in space robotics research. Learn more about [ATMOS and the propeller plate](https://atmos.discower.io/). For an example of this interface being used in an MPC, see [DISCOWER/px4-mpc](https://github.com/DISCOWER/px4-mpc).

## How it works

The node takes four requested propeller forces, in Newtons, in a `Float32MultiArray`. It uses separate measured curves for positive and negative forces to convert them into normalized inputs for PX4.

With a namespace `<ns>`, the topics are:

- Input: `/<ns>/prop_plate/external_motor_cmd` (`std_msgs/msg/Float32MultiArray`)
- Output: `/<ns>/fmu/in/vehicle_command` (`px4_msgs/msg/VehicleCommand`)

Use one array value per propeller, in the order expected by your setup. Replace `<ns>` in the examples below with your robot's namespace.

## Use the package

1. Add this package to a ROS 2 workspace with its dependencies installed.
2. Build and source the workspace:

   ```bash
   colcon build --packages-select propeller_plate_iface
   source install/setup.bash
   ```

3. Launch the node in a namespace that has a matching configuration entry in `config/parameters.yaml`:

   ```bash
   ros2 launch propeller_plate_iface propeller_plate_iface.launch.py namespace:="<ns>"
   ```

4. Publish a force array to test it:

   ```bash
   ros2 topic pub --once "/<ns>/prop_plate/external_motor_cmd" std_msgs/msg/Float32MultiArray "{data: [1.0, 0.5, -0.5, -1.0]}"
   ```

Check the output with `ros2 topic echo "/<ns>/fmu/in/vehicle_command"`.

## Using your own propeller plate

The curves in `config/parameters.yaml` were calculated from measurements for the listed ATMOS setups. They may not match a different propeller plate, so measure your own force response and fit separate curves for positive and negative forces.

Each curve is a list of polynomial coefficients in ascending power order. For example, `[c0, c1, c2]` means $u = c_0 + c_1 F + c_2 F^2$, where $F$ is the force in Newtons and $u$ is the normalized input.

Put your coefficients in `positive_thrust_curve` and `negative_thrust_curve` under your robot's namespace in `config/parameters.yaml`. Use the same namespace when launching the node.
