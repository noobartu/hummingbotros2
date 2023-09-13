import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_srvs.srv import Empty

NODE_NAME = "rover"
SUB_TOPIC_NAME = "/control"
VELOCITY_TOPIC_PARAMS = ['/velocity_topic_left', '/velocity_topic_right']
VELOCITY_COEF_PARAM = 'velocity_coef'
SRV_TOPIC_NAME = "/stop"

def handler_stop_service(request, response):
    """Stops wheels by publishing 0 velocity."""
    pub_left.publish(0)
    pub_right.publish(0)
    return response

def calculate_velocity(linear, angular):
    """
    Calculates speed of the wheels and returns them.

    Formula:
    left wheel velocity: linear - (angular / 2)
    right wheel velocity: linear + (angular / 2)

    Parameters:
    linear (float32): value of twist.linear.x
    angular (float32): value of twist.angular.z

    Returns:
    float32, float32
    """
    left_wheel = vel_coef * (linear - (angular / 2))
    right_wheel = vel_coef * (linear + (angular / 2))

    return left_wheel, right_wheel

def callback_create_publisher(twist):
    """
    Publishes speeds of the wheels.

    Parameters:
    twist: the message of teleop node
    """
    left_speed, right_speed = calculate_velocity(twist.linear.x, twist.angular.z)
    msg_left.data = left_speed
    msg_right.data = right_speed
    pub_left.publish(msg_left)
    pub_right.publish(msg_right)

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)

    # Read velocity topics and coefficient parameter
    vel_coef = node.get_parameter(VELOCITY_COEF_PARAM).get_parameter_value().double_value
    vel_topic_left = node.get_parameter(VELOCITY_TOPIC_PARAMS[0]).get_parameter_value().string_value
    vel_topic_right = node.get_parameter(VELOCITY_TOPIC_PARAMS[1]).get_parameter_value().string_value

    # Publishers created
    pub_left = node.create_publisher(Float32, vel_topic_left, 10)
    pub_right = node.create_publisher(Float32, vel_topic_right, 10)

    # Message types created
    msg_left = Float32()
    msg_right = Float32()

    # Subscriber created
    sub = node.create_subscription(Twist, SUB_TOPIC_NAME, callback_create_publisher, 10)

    # Service created
    srv = node.create_service(Empty, SRV_TOPIC_NAME, handler_stop_service)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
