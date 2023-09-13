import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty

NODE_NAME = 'joy_teleop'
SUB_TOPIC_NAME = '/joy'
PUB_TOPIC_NAME = '/control'
SRV_TOPIC_NAME = '/stop'

def call_stop_service():
    client = node.create_client(Empty, SRV_TOPIC_NAME)
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service %s not available' % SRV_TOPIC_NAME)
        return
    request = Empty.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

def callback(data):
    """
    Receives joystick messages (subscribed to Joy topic).
    then converts the joystick inputs into Twist commands.

    axis 1 aka left stick vertical controls linear speed.

    axis 0 aka left stick horizontal controls angular speed.
    """
    twist = Twist()

    stop_button = data.buttons[3]  # stop button of the joystick
    if stop_button:
        call_stop_service()
        return

    twist.linear.x = data.axes[1]
    twist.angular.z = data.axes[0]
    pub.publish(twist)

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    node.get_logger().info("Node initialized!")

    pub = node.create_publisher(Twist, PUB_TOPIC_NAME, 10)
    node.get_logger().info("Waiting for Joy message!")

    sub = node.create_subscription(Joy, SUB_TOPIC_NAME, callback, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
