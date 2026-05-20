import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('target_color_r', 220)
        self.declare_parameter('target_color_g', 60)
        self.declare_parameter('target_color_b', 60)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.t = 0.0
        self.timer = self.create_timer(0.1, self._publish)

    def _publish(self):
        self.t += 0.1
        frame_id = self.get_parameter('frame_id').value
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        square_x = int(width * 0.5 + math.sin(self.t * 0.6) * 70)
        square_y = int(height * 0.5 + math.cos(self.t * 0.4) * 40)
        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = frame_id
        img.height = height
        img.width = width
        img.encoding = 'rgb8'
        img.is_bigendian = 0
        img.step = width * 3
        data = bytearray(width * height * 3)
        for y in range(height):
            for x in range(width):
                idx = (y * width + x) * 3
                if abs(x - square_x) < 35 and abs(y - square_y) < 35:
                    data[idx] = int(self.get_parameter('target_color_r').value)
                    data[idx + 1] = int(self.get_parameter('target_color_g').value)
                    data[idx + 2] = int(self.get_parameter('target_color_b').value)
                else:
                    shade = 30 + int(20 * math.sin((x + y) * 0.01 + self.t))
                    data[idx] = shade
                    data[idx + 1] = shade
                    data[idx + 2] = shade + 10
        img.data = bytes(data)
        info = CameraInfo()
        info.header = img.header
        info.height = height
        info.width = width
        fx = 525.0
        fy = 525.0
        cx = width / 2.0
        cy = height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.image_pub.publish(img)
        self.info_pub.publish(info)


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
