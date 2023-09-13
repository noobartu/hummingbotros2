#include <memory>
#include <vector>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/rendering.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace gazebo
{
    class CameraSensorPlugin : public SensorPlugin
    {
    public:
        CameraSensorPlugin() {}
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
            // Get Camera Sensor
            this->cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

            // Get ROS topic name from SDF file
            std::string topicName = "hummingdrone_camera"; // Replace with your desired topic name

            // Initialize ROS 2
            rclcpp::init(0, nullptr);

            // Create ROS 2 Node
            this->rosNode = std::make_shared<rclcpp::Node>("gazebo_client");

            RCLCPP_INFO(this->rosNode->get_logger(), "Hummingdrone camera plugin is attached.");

            // Initialize the ROS 2 publisher
            this->rosPub = this->rosNode->create_publisher<sensor_msgs::msg::Image>(topicName, 1000);

            // Publish data every update
            this->updateConnection = this->cameraSensor->ConnectUpdated(
                std::bind(&CameraSensorPlugin::OnRosMsg, this));
        }

    private:
        void OnRosMsg()
        {
            // Get image size
            imgSize = this->cameraSensor->Camera()->ImageByteSize();

            // Create ROS 2 message
            sensor_msgs::msg::Image imgMsgROS;
            imgMsgROS.header.frame_id = "hummingdrone_camera";
            imgMsgROS.header.stamp.sec = this->cameraSensor->LastMeasurementTime().sec;
            imgMsgROS.header.stamp.nanosec = this->cameraSensor->LastMeasurementTime().nsec;

            imgMsgROS.height = this->cameraSensor->ImageHeight();
            imgMsgROS.width = this->cameraSensor->ImageWidth();
            imgMsgROS.encoding = "bgr8"; // This is for cvBridge
            imgMsgROS.is_bigendian = 0;
            imgMsgROS.step = this->cameraSensor->ImageWidth() * this->cameraSensor->Camera()->ImageDepth();

            const unsigned char *imgData = this->cameraSensor->Camera()->ImageData();

            for (int i = 0; i < imgSize; i++)
            {
                imgMsgROS.data.push_back(imgData[i]);
            }

            // Publish data
            this->rosPub->publish(imgMsgROS);
        }

    private:
        // Pointer to the camera
        sensors::CameraSensorPtr cameraSensor;

        // Image size of the frame
        int imgSize;

        // A ROS 2 node
        std::shared_ptr<rclcpp::Node> rosNode;

        // A ROS 2 publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rosPub;

        // A thread that keeps running the ROS queue
        event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_SENSOR_PLUGIN(CameraSensorPlugin);
} // namespace gazebo
