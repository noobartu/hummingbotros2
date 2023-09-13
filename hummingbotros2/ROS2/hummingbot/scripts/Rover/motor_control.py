import rclpy
from adafruit_motorhat import Adafruit_MotorHAT
from std_msgs.msg import String, Float32

NODE_NAME = "motor_control"
VELOCITY_TOPIC_PARAMS = ["/velocity_topic_left", "/velocity_topic_right"]

# Stops all motors
def all_stop():
    motor_left.setSpeed(0)
    motor_right.setSpeed(0)
    motor_left.run(Adafruit_MotorHAT.RELEASE)
    motor_right.run(Adafruit_MotorHAT.RELEASE)

def set_motor_speed(speed, motor_id):
    """
    Takes a float number and sets motor speed of any wheel.

    speed   : Velocity of the wheel that is taken from the joystick

    motor_id : ID of left or right wheel

    """
    if motor_id == 1:
        motor = motor_left
    else:
        motor = motor_right

    motor.setSpeed(int(abs(speed)))

    if speed >= 0:
        motor.run(Adafruit_MotorHAT.FORWARD)
    else:
        motor.run(Adafruit_MotorHAT.BACKWARD)

def callback_left(velocity):
    set_motor_speed(velocity.data, motor_id=1)

def callback_right(velocity):
    set_motor_speed(velocity.data, motor_id=2)

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)

    # Setup motor controller
    motor_driver = Adafruit_MotorHAT(i2c_busnum=1)

    # Initialize IDs for left and right motor
    motor_left_ID = 1
    motor_right_ID = 2

    # Get motor drivers
    motor_left = motor_driver.get_motor(motor_left_ID)
    motor_right = motor_driver.get_motor(motor_right_ID)

    # Stop the motors as a precaution
    all_stop()

    # Take dynamic topic names from launch file
    vel_topic_left = node.declare_parameter(VELOCITY_TOPIC_PARAMS[0]).get_parameter_value().string_value
    vel_topic_right = node.declare_parameter(VELOCITY_TOPIC_PARAMS[1]).get_parameter_value().string_value

    # Create subscribers
    node.create_subscription(Float32, vel_topic_left, callback_left, 10)
    node.create_subscription(Float32, vel_topic_right, callback_right, 10)

    # Start running
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Stop motors before exiting
    all_stop()

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
