import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32

topic_1 = 'float_topic'
topic_2 = 'string_topic'

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_float = self.create_publisher(Float32, topic_1, 10)
        self.publisher_string = self.create_publisher(String, topic_2, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg_float = Float32()
        msg_float.data = float(self.count)
        self.publisher_float.publish(msg_float)

        msg_string = String()
        msg_string.data = f"Message number {self.count}"
        self.publisher_string.publish(msg_string)

        self.get_logger().info(f'Publishing: {msg_float.data}, {msg_string.data}')
        self.count += 1
    
def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()