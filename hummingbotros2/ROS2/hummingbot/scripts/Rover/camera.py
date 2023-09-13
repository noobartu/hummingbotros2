import rclpy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

CAMERA_TOPIC_NAME = '/gazebo_client/hummingdrone_camera'

def callback(img):
    """Get the image from Gazebo and convert to OpenCV image format using CvBridge"""
    try:
        frame = bridge.imgmsg_to_cv2(img, 'bgr8')
    except CvBridgeError as e:
        print(str(e))

    # Convert the image to a Numpy array since most cv2 functions require Numpy arrays.
    frame = np.array(frame, dtype=np.uint8)

    # Debug: Print Shape and size as bytes
    node.get_logger().info(f'Image shape: {frame.shape}')

    # Show image
    cv2.imshow('Image Window', frame)
    cv2.waitKey(1000)

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('rover_camera_receiver')

    node.get_logger().info('Camera receiver is running...')

    cv2.namedWindow("Image Window", 1)
    bridge = CvBridge()

    sub = node.create_subscription(Image, CAMERA_TOPIC_NAME, callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the OpenCV window when the node is shut down
    cv2.destroyAllWindows()

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
