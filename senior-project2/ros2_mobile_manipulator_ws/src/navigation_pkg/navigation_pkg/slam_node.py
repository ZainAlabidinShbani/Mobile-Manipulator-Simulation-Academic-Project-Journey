import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(2.0, self._publish_map)

    def _publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.1
        grid.info.width = 80
        grid.info.height = 80
        grid.info.origin.position.x = -4.0
        grid.info.origin.position.y = -4.0
        grid.info.origin.orientation.w = 1.0
        data = [-1] * (grid.info.width * grid.info.height)
        for y in range(grid.info.height):
            for x in range(grid.info.width):
                idx = y * grid.info.width + x
                if x in (0, grid.info.width - 1) or y in (0, grid.info.height - 1):
                    data[idx] = 100
                elif 55 <= x <= 63 and 36 <= y <= 44:
                    data[idx] = 100
                elif 20 <= x <= 30 and 20 <= y <= 30:
                    data[idx] = 60
                else:
                    data[idx] = 0
        grid.data = data
        self.map_pub.publish(grid)


def main():
    rclpy.init()
    node = SlamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
