import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import serial
import numpy as np

class ROVPWMController(Node):
    def __init__(self):
        super().__init__('rov_pwm_controller')

        # --- Parameters ---
        self.declare_parameter('arduino_port', '/dev/ttyUSB0')
        port = self.get_parameter('arduino_port').get_parameter_value().string_value
        self.arduino = serial.Serial(port, baudrate=115200, timeout=0.1)

        # --- Thruster Mixing Matrix (6x6) ---
        self.mixing_matrix = np.array([
            [1,  0,  0,  1, 0,  0],  # T1
            [1,  0, 0, -1, 0,  0],  # T2
            [1,  0, 0,  1, 0,  0],  # T3
            [1,  0,  0, -1, 0,  0],  # T4
            [0,  1,  1,  0, 0,  1],  # T5
            [0,  1,  1,  0, 0, -1],  # T6
        ])

        # --- Init ---
        self.thrust_input = None
        self.ballast_input = None

        # --- Subscribers ---
        self.create_subscription(Twist, '/rov/thruster_cmd_raw', self.twist_callback, 10)
        self.create_subscription(Vector3, '/rov/ballast_cmd', self.ballast_callback, 10)

        # --- Timer to send to Arduino ---
        self.create_timer(0.05, self.send_pwm)  # 20 Hz

        self.get_logger().info("ROV PWM Controller Initialized")

    def twist_callback(self, msg: Twist):
        self.thrust_input = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.z,  # yaw
            msg.angular.y,  # pitch
            msg.angular.x   # roll
        ])

    def ballast_callback(self, msg: Vector3):
        self.ballast_input = [msg.x, msg.y]  # front, rear

    def send_pwm(self):
        if self.thrust_input is None or self.ballast_input is None:
            return

        # --- Thruster Mixing ---
        thrust_output = self.mixing_matrix @ self.thrust_input
        thrust_output = np.clip(thrust_output, -1.0, 1.0)
        thrust_pwm = [int(1500 + 400 * t) for t in thrust_output]

        # --- Ballast ---
        front = np.clip(self.ballast_input[0], -1.0, 1.0)
        rear  = np.clip(self.ballast_input[1], -1.0, 1.0)
        ballast_pwm = [int(1500 + 400 * front), int(1500 + 400 * rear)]

        # --- Combine and Send to Arduino ---
        final_pwm = thrust_pwm + ballast_pwm
        packet = ','.join(str(p) for p in final_pwm) + '\n'
        try:
            self.arduino.write(packet.encode())
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            return

        # Optional debug log
        self.get_logger().debug(f"Sent PWM: {final_pwm}")

def main(args=None):
    rclpy.init(args=args)
    node = ROVPWMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()