import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('juanito')
        self.get_logger().info('Test node juanito has been started.')

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()