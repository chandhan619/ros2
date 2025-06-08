import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32

topic_1 = 'float_topic'
topic_2 = 'string_topic'

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription_float = self.create_subscription(Float32, topic_1, self.float_callback, 10)
        self.subscription_string = self.create_subscription(String, topic_2, self.string_callback, 10)

    def float_callback(self, msg):
        self.get_logger().info(f'Received float: {msg.data}')

    def string_callback(self, msg):
        self.get_logger().info(f'Received string: {msg.data}')
def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()