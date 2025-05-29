import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from odrive_can.msg import ControlMessage


class CmdVelToMotorControl(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_motor_control')

        # Subscription to the /cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/tyr/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for ODrive axes
        self.odrive_axis1_publisher = self.create_publisher(
            ControlMessage,
            'odrive_axis1/control_message',
            10
        )
        self.odrive_axis2_publisher = self.create_publisher(
            ControlMessage,
            'odrive_axis2/control_message',
            10
        )
        self.odrive_axis3_publisher = self.create_publisher(
            ControlMessage,
            'odrive_axis3/control_message',
            10
        )
        self.odrive_axis4_publisher = self.create_publisher(
            ControlMessage,
            'odrive_axis4/control_message',
            10
        )

    def cmd_vel_callback(self, msg):
        # Read linear and angular velocities from /cmd_vel
        linear_x = msg.linear.x  # Forward/backward velocity
        angular_z = msg.angular.z  # Rotational velocity

        # Differential drive formulas
        wheel_radius = 0.2  
        wheel_base = 0.62     # Example: 30 cm distance between wheels

        # Calculate wheel velocities
        left_wheel_velocity = (linear_x - angular_z * wheel_base / 2) / wheel_radius
        right_wheel_velocity = (linear_x + angular_z * wheel_base / 2) / wheel_radius

        # Adjust the sign for the right wheels
        input_vel_axis1 = left_wheel_velocity
        input_vel_axis4 = left_wheel_velocity
        input_vel_axis2 = -right_wheel_velocity
        input_vel_axis3 = -right_wheel_velocity

        # Create control messages for each axis
        control_msg_axis1 = ControlMessage(
            control_mode=2,  # Velocity control
            input_mode=1,    # Input velocity mode
            input_pos=0.0,   # Unused in this mode
            input_vel=input_vel_axis1,
            input_torque=1.0
        )
        self.odrive_axis1_publisher.publish(control_msg_axis1)

        control_msg_axis2 = ControlMessage(
            control_mode=2,
            input_mode=1,
            input_pos=0.0,
            input_vel=input_vel_axis2,
            input_torque=1.0
        )
        self.odrive_axis2_publisher.publish(control_msg_axis2)

        control_msg_axis3 = ControlMessage(
            control_mode=2,
            input_mode=1,
            input_pos=0.0,
            input_vel=input_vel_axis3,
            input_torque=1.0
        )
        self.odrive_axis3_publisher.publish(control_msg_axis3)

        control_msg_axis4 = ControlMessage(
            control_mode=2,
            input_mode=1,
            input_pos=0.0,
            input_vel=input_vel_axis4,
            input_torque=1.0
        )
        self.odrive_axis4_publisher.publish(control_msg_axis4)


def main(args=None):
    rclpy.init(args=args)

    # Create the CmdVelToMotorControl node
    cmd_vel_to_motor_control_node = CmdVelToMotorControl()

    # Spin the node to start processing incoming messages
    rclpy.spin(cmd_vel_to_motor_control_node)

    # Destroy the node
    cmd_vel_to_motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
