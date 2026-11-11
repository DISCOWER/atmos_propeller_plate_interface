#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleCommand
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

TIMEOUT = 1.0  # seconds
HYSTERESIS_THRESHOLD = 0.05  # N


class ActuatorCommandPublisher(Node):

    def __init__(self):
        super().__init__('actuator_command_publisher')

        # Hysteresis state: 1 for positive, -1 for negative, 0 for zero/unknown
        self.hysteresis_state = [0.0, 0.0, 0.0, 0.0]

        # Get polynomial coefficients from parameters or use defaults
        self.declare_parameter("positive_thrust_curve", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("negative_thrust_curve", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        while not self.get_parameter("positive_thrust_curve").value or \
                not self.get_parameter("negative_thrust_curve").value:
            self.get_logger().warn("Waiting for thrust curve parameters...")
            rclpy.spin_once(self, timeout_sec=0.5)

        # Get hosts
        self.positive_thrust_curve = (
            self.get_parameter("positive_thrust_curve").get_parameter_value().double_array_value
        )
        self.negative_thrust_curve = (
            self.get_parameter("negative_thrust_curve").get_parameter_value().double_array_value
        )
        self.get_logger().info(f"Loaded positive thrust curve: {self.positive_thrust_curve}")
        self.get_logger().info(f"Loaded negative thrust curve: {self.negative_thrust_curve}")

        # QoS profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(
            VehicleCommand,
            'fmu/in/vehicle_command',
            qos_profile
        )
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'prop_plate/external_motor_cmd',
            self.listener_callback,
            10
        )

        self.received_data = None
        self.data_timestamp = self.get_clock().now().nanoseconds / 1e9  # seconds
        # Publish at 100Hz
        self.timer = self.create_timer(0.01, self.publish_command)

    def listener_callback(self, msg):
        # Check time now vs timeout
        self.data_timestamp = self.get_clock().now().nanoseconds / 1e9  # seconds

        # Check if data length is correct
        if len(msg.data) != 4:
            self.get_logger().warn(
                f"Received actuator command of length {len(msg.data)}. Expected length is 4.", throttle_duration_sec=1.0
            )
            return
        self.received_data = msg.data

    def get_ppm_from_forces(self, forces):
        # Convert desired forces (N) to normalized actuator commands
        normalized_pwm = [0.0] * 4
        threshold = HYSTERESIS_THRESHOLD

        for i, thrust in enumerate(forces):
            # Below threshold: stop the propeller and clear the direction memory,
            # otherwise the polynomial's constant term would keep it at idle thrust
            if abs(thrust) < threshold:
                self.hysteresis_state[i] = 0
                normalized_pwm[i] = 0.0
                continue

            state = self.hysteresis_state[i]

            # Hysteresis: only change direction once past the opposite threshold
            if state >= 0:
                self.hysteresis_state[i] = -1 if thrust < -threshold else 1
            else:
                self.hysteresis_state[i] = 1 if thrust > threshold else -1

            # Simple linear mapping for demonstration purposes
            # Assuming force range: 0N to 10N maps to PWM: 1000us to 2000us
            pwm = 0.0

            # Use the direction to select curve
            if self.hysteresis_state[i] == 1:
                # Positive curve
                t = max(thrust, 0.0)
                for k in range(len(self.positive_thrust_curve)):
                    pwm += self.positive_thrust_curve[k] * (t ** k)
                pwm = max(0.0, min(0.8, pwm))
            elif self.hysteresis_state[i] == -1:
                # Negative curve
                t = min(thrust, 0.0)
                for k in range(len(self.negative_thrust_curve)):
                    pwm += self.negative_thrust_curve[k] * (t ** k)
                pwm = max(-1.0, min(0.0, pwm))
            else:
                pwm = 0.0
            normalized_pwm[i] = pwm
        
        return normalized_pwm
    
    def publish_command(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds)

        # PX4 command type
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR

        # Force index = 0 → so param7 must be 0.0
        msg.param7 = 0.0
        index = int(msg.param7 + 0.5)

        if (self.get_clock().now().nanoseconds / 1e9 - self.data_timestamp) < TIMEOUT and self.received_data is not None:
            desired_forces = self.received_data
            _normalized_input = self.get_ppm_from_forces(desired_forces)
        else:
            _normalized_input = [0.0] * 4
            self.hysteresis_state = [0.0] * 4

        # Fill vehicle command with data
        msg.param1 = _normalized_input[0]
        msg.param2 = _normalized_input[1]
        msg.param3 = _normalized_input[2]
        msg.param4 = _normalized_input[3]
        msg.from_external = True

        # Publish the message
        self.publisher.publish(msg)

        self.get_logger().debug(
            f"Publishing actuator group 0 (index={index}): "
            f"{[msg.param1, msg.param2, msg.param3, msg.param4]}"
        )


def main(args=None):
    rclpy.init()
    node = ActuatorCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        rclpy.shutdown()
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
