import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

from SX127x.LoRa import LoRa
from SX127x.board_config import BOARD
from SX127x.constants import MODE

import time

class LoRaInterface(LoRa):
    def __init__(self, node):
        super().__init__(verbose=False)
        self.node = node
        self.set_mode(MODE.SLEEP)
        self.set_freq(433.0)
        self.set_spreading_factor(7)
        self.set_bw(7)
        self.set_coding_rate(5)
        self.set_rx_crc(True)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def on_rx_done(self):
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        msg = bytes(payload).decode("utf-8", errors="ignore").strip()
        self.node.handle_packet(msg)

        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

class LoRaReceiverNode(Node):
    def __init__(self):
        super().__init__('lora_spi_receiver_node')

        self.twist_pub = self.create_publisher(Twist, '/rov/thruster_cmd_raw', 10)
        self.ballast_pub = self.create_publisher(Vector3, '/rov/ballast_cmd', 10)

        BOARD.setup()
        self.lora = LoRaInterface(self)

        self.get_logger().info('LoRa SPI Receiver Node Initialized')
        self.timer = self.create_timer(0.05, self.check_lora)

    def check_lora(self):
        try:
            self.lora.on_rx_done()
        except Exception as e:
            self.get_logger().error(f"LoRa error: {e}")

    def handle_packet(self, msg: str):
        try:
            parts = [float(x) for x in msg.split(',')]
            if len(parts) != 8:
                self.get_logger().warn(f"Invalid packet: {msg}")
                return

            # Thruster: x, y, z, yaw, pitch, roll
            twist = Twist()
            twist.linear.x, twist.linear.y, twist.linear.z = parts[0:3]
            twist.angular.z, twist.angular.y, twist.angular.x = parts[3:6]
            self.twist_pub.publish(twist)

            # Ballast: front, rear
            ballast = Vector3()
            ballast.x, ballast.y = parts[6:8]
            ballast.z = 0.0
            self.ballast_pub.publish(ballast)

            self.get_logger().info(f"Thruster: {parts[0:6]}, Ballast: {parts[6:]}")
        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")

    def destroy_node(self):
        super().destroy_node()
        BOARD.teardown()

def main(args=None):
    rclpy.init(args=args)
    node = LoRaReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


