import rclpy
from sensor_msgs.msg import Joy

NODE_NAME = "test_publisher"
TOPIC_NAME = "/joy"
AXES_MSG = [1.0, 1.0]
BUTTON_MSG = [0, 0, 0, 1]

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    pub = node.create_publisher(Joy, TOPIC_NAME, 10)
    message = Joy()

    rate = node.create_rate(2)

    try:
        while rclpy.ok():
            message.axes = AXES_MSG
            message.buttons = BUTTON_MSG
            pub.publish(message)
            rate.sleep()
    except KeyboardInterrupt:
        pass

    node.get_logger().info("Node has stopped!")
    rclpy.shutdown()
