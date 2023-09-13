import time
import unittest
import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32

PKG = 'hummingbot'
NODE_NAME = "test"
SUB_TOPIC_NAMES = ["/gazebo_hummingbot_client/left_vel", "/gazebo_hummingbot_client/right_vel"]
MIN_RANGE = -2.0
MAX_RANGE = 2.0

class TestHummingbot(unittest.TestCase):

    def __init__(self, *args):
        super(TestHummingbot, self).__init__(*args)
        self.left_speed = None
        self.right_speed = None

        rclpy.init()
        node = rclpy.create_node(NODE_NAME)
        qos = QoSProfile(depth=1)

        self.left_sub = node.create_subscription(Float32, SUB_TOPIC_NAMES[0], self.callback_left, qos)
        self.right_sub = node.create_subscription(Float32, SUB_TOPIC_NAMES[1], self.callback_right, qos)

        # Wait for subscribers to initialize
        time.sleep(1)

    def test_greater(self):
        """Checks if the values are greater or equal to the minimum number."""
        self.assertIsNotNone(self.left_speed)
        self.assertIsNotNone(self.right_speed)
        self.assertGreaterEqual(self.left_speed, MIN_RANGE)
        self.assertGreaterEqual(self.right_speed, MIN_RANGE)

    def test_less(self):
        """Checks if the values are less or equal to the maximum number."""
        self.assertIsNotNone(self.left_speed)
        self.assertIsNotNone(self.right_speed)
        self.assertLessEqual(self.left_speed, MAX_RANGE)
        self.assertLessEqual(self.right_speed, MAX_RANGE)

    def test_stop_service(self):
        """Checks if the values are zero when stop service is called."""
        self.assertIsNotNone(self.left_speed)
        self.assertIsNotNone(self.right_speed)
        self.assertEqual(self.left_speed, 0.0)
        self.assertEqual(self.right_speed, 0.0)

    def callback_left(self, msg):
        self.left_speed = msg.data

    def callback_right(self, msg):
        self.right_speed = msg.data

    def tearDown(self):
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
