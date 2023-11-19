import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from tutorial_rclcpp_pub_sub.msg import TutorialClient      # Custom message
from tutorial_rclcpp_service.srv import TutorialServiceAdd      # Custom service
from std_msgs.msg import String


class MinimalRclpyNode(Node):

    def __init__(self):
        super().__init__('rclpy_service_client')
        
        self.subscription = self.create_subscription(   # Create a subscription
            TutorialClient,                             # with message type
            'topic',                                    # path to listen topic
            self.cb_new_client_message,                 # callback function to be called when new topic received
            10)                                         # 

        self.cb = None
        self.cli = self.create_client(                  # Create a service client
            TutorialServiceAdd,                         # Service type
            'add_two_ints',                             # path to the service
            callback_group = self.cb                    # Add to callback group
            )
        while not self.cli.wait_for_service(timeout_sec=1.0):   # Wait for server
            self.get_logger().info('service not available, waiting again...')
        self.req = TutorialServiceAdd.Request()

        self.timer_cb = MutuallyExclusiveCallbackGroup()
        timer_period = 0.5                              # seconds
        self.timer = self.create_timer(                 # Create a timer loop
            timer_period,                               # Timer period
            self.cb_timer_update,                       # Callback function
            callback_group = self.timer_cb)             # Add to callback group
        self.timer_cnt = 0

    def cb_new_client_message(self, msg):
        self.get_logger().info('Input client: "%d"' % msg.count) # CHANGE

    async def cb_timer_update(self) -> None:
        response = await self.send_request(self.timer_cnt, 10)
        self.get_logger().info(
            "Result of add_two_ints: for %d + %d = %d" % (self.timer_cnt, 10, response.sum)
        )
        self.timer_cnt += 1

    async def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        while rclpy.ok():
            if self.future.done() and self.future.result():
                self.get_logger().info("send_request: exit")
                return self.future.result()

        return None
    

def main(args=None):
    rclpy.init(args=args)

    minimal_rclpy_node = MinimalRclpyNode()
    executor = MultiThreadedExecutor()          # For service response
    executor.add_node(minimal_rclpy_node)
    executor.spin()
    #rclpy.spin(minimal_rclpy_node)
    #minimal_rclpy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()