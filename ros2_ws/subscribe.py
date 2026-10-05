import rclpy
from rclpy.node import Node
from ctuav_link_interfaces.msg import LinkStatus

class MinimalSubscriber(Node):

    def __init__(self, msg_queue=None):
        super().__init__('minimal_subscriber')
        self.msg_queue = msg_queue
        self.subscription = self.create_subscription(
            LinkStatus,
            '/uav/uav_01/status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if self.msg_queue is not None:
            self.msg_queue.put(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()