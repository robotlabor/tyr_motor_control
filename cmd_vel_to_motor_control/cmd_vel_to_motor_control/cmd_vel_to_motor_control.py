import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from odrive_can.msg import ControlMessage, ControllerStatus


class CmdVelToMotorControl(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_motor_control')

        # Robot parameters
        self.wheel_radius = 0.2         # meters
        self.wheel_base = 0.62          # meters

        # Last received cmd_vel
        self.last_linear = 0.0
        self.last_angular = 0.0

        # Encoder feedback (masters)
        self.front_left_rpm = 0.0
        self.front_right_rpm = 0.0

        # Deadband threshold
        self.deadband = 0.05

        # ---------------------------------------------------------------
        # Subscriptions
        # ---------------------------------------------------------------
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/tyr/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.front_left_sub = self.create_subscription(
            ControllerStatus,
            'odrive_axis3/controller_status',
            self.front_left_status_callback,
            10
        )

        self.front_right_sub = self.create_subscription(
            ControllerStatus,
            'odrive_axis4/controller_status',
            self.front_right_status_callback,
            10
        )

        # ---------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------
        self.pub = {
            1: self.create_publisher(ControlMessage, 'odrive_axis1/control_message', 10),  # rear right
            2: self.create_publisher(ControlMessage, 'odrive_axis2/control_message', 10),  # rear left
            3: self.create_publisher(ControlMessage, 'odrive_axis3/control_message', 10),  # front left
            4: self.create_publisher(ControlMessage, 'odrive_axis4/control_message', 10),  # front right
        }

        # ---------------------------------------------------------------
        # Timer: continuous control loop at 50 Hz
        # ---------------------------------------------------------------
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Master-slave motor control running.")


    # ================================================================
    # Feedback callbacks
    # ================================================================
    def front_left_status_callback(self, msg):
        # Left side is inverted
        self.front_left_rpm = -msg.vel_estimate

    def front_right_status_callback(self, msg):
        self.front_right_rpm = msg.vel_estimate


    # ================================================================
    # cmd_vel callback
    # ================================================================
    def cmd_vel_callback(self, msg):
        self.last_linear = msg.linear.x
        self.last_angular = msg.angular.z


    # ================================================================
    # MAIN CONTROL LOOP (50 Hz)
    # ================================================================
    def control_loop(self):

        # Apply deadband
        linear = 0.0 if abs(self.last_linear) < self.deadband else self.last_linear
        angular = 0.0 if abs(self.last_angular) < self.deadband else self.last_angular

        angular = -angular
        # ============================================================
        # RIGHT FRONT = MASTER
        # ============================================================
        v_rf = (linear + angular * self.wheel_base / 2.0) / self.wheel_radius

        self.send_velocity(4, v_rf)

        # ============================================================
        # RIGHT REAR = FOLLOW RIGHT FRONT ENCODER
        # ============================================================
        self.send_velocity(1, self.front_right_rpm)

        # ============================================================
        # LEFT FRONT = COMPUTED FROM DIFFERENTIAL DRIVE
        # ============================================================
        v_lf = (linear - angular * self.wheel_base / 2.0) / self.wheel_radius

        # Left side inverted
        self.send_velocity(3, -v_lf)

        # ============================================================
        # LEFT REAR = FOLLOW LEFT FRONT ENCODER
        # ============================================================
        self.send_velocity(2, -self.front_left_rpm)


    # ================================================================
    # Send velocity to ODrive
    # ================================================================
    def send_velocity(self, axis_id, velocity):
        msg = ControlMessage()
        msg.control_mode = 2        # velocity control
        msg.input_mode = 1          # passthrough
        msg.input_vel = float(velocity)
        msg.input_pos = 0.0
        msg.input_torque = 20.0     # must be float!
        self.pub[axis_id].publish(msg)


def main(args=None):
        rclpy.init(args=args)
        node = CmdVelToMotorControl()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
        main()
