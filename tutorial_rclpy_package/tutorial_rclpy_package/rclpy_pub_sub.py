import rclpy
from rclpy.node import Node

from tutorial_rclcpp_pub_sub.msg import TutorialClient      # Custom message
from tutorial_rclcpp_service.srv import TutorialServiceAdd      # Custom service

from std_msgs.msg import String


class MinimalRclpyNode(Node):

    def __init__(self):
        super().__init__('minimal_rclpy_node')
        
        self.subscription = self.create_subscription(   # Create a subscription
            TutorialClient,                             # with message type
            'topic',                                    # path to listen topic
            self.cb_new_client_message,                 # callback function to be called when new topic received
            10)                                         # 

        self.pub_string = self.create_publisher(        # Create a publisher
            String,                                     # Message type
            'talker',                                   # path to publish a topic
            10)                                         #

        timer_period = 0.5                              # seconds
        self.timer = self.create_timer(                 # Create a timer loop
            timer_period,                               # Timer period
            self.cb_timer_update)                       # Callback function
        self.pub_cnt = 0

    def cb_new_client_message(self, msg):
        self.get_logger().info('Input client: "%d"' % msg.count) # CHANGE

    def cb_timer_update(self) -> None:
        msg = String()
        msg.data = 'Hello %d'%self.pub_cnt
        self.pub_string.publish(msg)
        self.pub_cnt += 1
    

def main(args=None):
    rclpy.init(args=args)

    minimal_rclpy_node = MinimalRclpyNode()
    rclpy.spin(minimal_rclpy_node)
    minimal_rclpy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()