import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import MaterialColor
import qrcode
import numpy as np
import random
import string

class QRLEDDisplay(Node):
    def __init__(self):
        super().__init__('qr_led_display')
        self.publisher = self.create_publisher(MaterialColor, '/world/default/material_color', 256)
        self.index = 0
        self.qr_chunks = []

        self.generate_qr_chunks()
        self.timer = self.create_timer(2.0, self.display_next_chunk)

    def generate_qr_chunks(self):
        rand_str = ''.join(random.choices(string.ascii_lowercase + string.digits, k=100))
        self.get_logger().info(f"QR String: {rand_str}")

        qr = qrcode.QRCode(
            version=7, 
            box_size=1,
            border=0
        )
        qr.add_data(rand_str)
        qr.make(fit=True)

        qr_matrix = np.array(qr.get_matrix(), dtype=int)
        h, w = qr_matrix.shape
        assert h >= 45 and w >= 45, "QR matrix is too small"
        # Expanded matrix to 48x48 to adjust for 16x16 chunks
        qr_matrix = np.pad(qr_matrix, ((0, 3), (0, 3)), mode='constant', constant_values=0)

        self.qr_chunks = []
        for i in [0, 16, 32]:
            for j in [0, 16, 32]:
                chunk = qr_matrix[i:i+16, j:j+16]
                if chunk.shape == (16, 16):
                    self.qr_chunks.append(chunk)

    def display_next_chunk(self):
        if not self.qr_chunks:
            return

        matrix = self.qr_chunks[self.index % len(self.qr_chunks)]
        self.index += 1

        for row in range(16):
            for col in range(16):
                msg = MaterialColor()
                msg.entity_match = 1
                msg.entity.name = f"led_{row}_{col}"
                on = matrix[row, col] == 1
                color = (1.0, 1.0, 1.0) if on else (0.0, 0.0, 0.0)

                msg.diffuse.r, msg.diffuse.g, msg.diffuse.b = color
                msg.diffuse.a = 1.0
                msg.ambient = msg.diffuse
                msg.specular = msg.diffuse
                msg.emissive = msg.diffuse

                self.publisher.publish(msg)

def main(args=None):
    rclpy.init()
    node = QRLEDDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
