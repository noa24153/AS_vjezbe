import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'broj',
            self.listener_callback,
            10)
        self.subscription  
        self.publisher_= self.create_publisher(String, 'kvadrat_broja', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        msgPub = String()
        kvadrat = int(msg.data) **2
        msgPub.data = 'Kvadrat broja %d je %d' % (int(msg.data), kvadrat)
        self.publisher_.publish(msgPub)
        self.get_logger().info('Publishing: "%s"' % msgPub.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
