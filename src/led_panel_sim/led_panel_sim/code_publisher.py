import rclpy
import random
import numpy as np
from rclpy.node import Node
from ros_gz_interfaces.msg import MaterialColor
import string

class CodePublisher(Node):
    def __init__(self):
        super().__init__('code_publisher')
        self.publisher = self.create_publisher(MaterialColor, '/world/default/material_color', 256)

        self.rand_code = ''.join(random.choices(string.ascii_lowercase + string.digits, k=100))
        self.char_index = 0
        self.sub_index = 0
        self.precomputed_matrices = [self.generate_matrix(c) for c in self.rand_code]

        self.timer = self.create_timer(2.0, self.publish_code_callback)

    def generate_matrix(self, char):
        binary_value = f'{ord(char):08b}'
        matrix = [np.zeros((16, 16), dtype=int) for _ in range(2)]
        for i in [0, 4]:
            binary_matrix = np.array([[int(bit) for bit in binary_value[i:i+4]]]).reshape(2, 2)
            matrix[i // 4] = np.kron(binary_matrix, np.ones((8, 8), dtype=int))
        return matrix

    def publish_code_callback(self):
        if self.char_index >= len(self.rand_code):
            self.get_logger().info("Finished publishing all characters.")
            self.timer.cancel()
            return

        char = self.rand_code[self.char_index]
        sub_matrix = self.precomputed_matrices[self.char_index][self.sub_index]

        self.get_logger().info(f"Publishing char '{char}' sub-matrix {self.sub_index}")

        for row in range(16):
            for col in range(16):
                msg = MaterialColor()
                msg.entity_match = 1
                msg.entity.name = f"led_{row}_{col}"
                on = sub_matrix[row, col] == 1
                color = (1.0, 1.0, 1.0) if on else (0.0, 0.0, 0.0)
                msg.emissive.r, msg.emissive.g, msg.emissive.b = color
                msg.emissive.a = 1.0
                self.publisher.publish(msg)

        # Update indices for next callback
        if self.sub_index == 0:
            self.sub_index = 1
        else:
            self.sub_index = 0
            self.char_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = CodePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.get_logger().info("Destroying node and shutting down.")
        node.destroy_node()
        if rclpy.ok():  # Only shut down if not already shut down
            rclpy.shutdown()


if __name__ == '__main__':
    main()

