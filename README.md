# ATMOS: propeller plate interface package

This package provides a ROS 2 node that interfaces with a propeller plate to control motor commands based on external inputs. It subscribes to desired forces for each motor, which are then converted to PWM signals using the polynomial coefficients defined in `config/parameters.yaml`. 

### Usage:
1. **Installation**: Ensure you have ROS 2 installed and set up. Clone the repository containing this package into your ROS 2 workspace.
2. **Configuration**: Modify the `config/parameters.yaml` file to set the polynomial coefficients for the thrust curves according to your propeller plate specifications. The current parameters are based on the ATMOS publication.
3. **Running the Node**: Launch the node using ROS 2 launch files or directly via the command line:
    ```bash
    ros2 launch propeller_plate_iface propeller_plate_iface.launch.py namespace:=pop
    ```
    or with a custom namespace:
    ```bash
    ros2 launch propeller_plate_iface propeller_plate_iface.launch.py namespace:=<atmos-robot-namespace>
    ```