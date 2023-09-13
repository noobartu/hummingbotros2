#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace gazebo
{
    class HummingbotPlugin : public ModelPlugin
    {
    public:
        HummingbotPlugin() {}

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            if (_model->GetJointCount() == 0)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("HummingbotPlugin"),
                    "Invalid joint count, Hummingbot plugin didn't load?");
                return;
            }

            model = _model;

            leftJoint = model->GetJoints()[2];
            rightJoint = model->GetJoints()[3];

            pid = common::PID(1, 0, 0);

            model->GetJointController()->SetVelocityPID(leftJoint->GetScopedName(), pid);
            model->GetJointController()->SetVelocityPID(rightJoint->GetScopedName(), pid);

            SetVelocity(rightJoint, 0.0);
            SetVelocity(leftJoint, 0.0);

            rclcpp::init(0, nullptr);

            rosNode = std::make_shared<rclcpp::Node>("gazebo_hummingbot_client");
            rosLeftSub = rosNode->create_subscription<std_msgs::msg::Float32>(
                "left_vel", 1, [this](std_msgs::msg::Float32::SharedPtr msg) {
                    SetVelocity(leftJoint, msg->data);
                });

            rosRightSub = rosNode->create_subscription<std_msgs::msg::Float32>(
                "right_vel", 1, [this](std_msgs::msg::Float32::SharedPtr msg) {
                    SetVelocity(rightJoint, msg->data);
                });

            rosLeftQueueThread = std::thread([this]() { rosLeftQueueThreadFunc(); });
            rosRightQueueThread = std::thread([this]() { rosRightQueueThreadFunc(); });
        }

    private:
        void SetVelocity(const physics::JointPtr &joint, const double &vel)
        {
            model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), -1 * vel);
        }

        void rosLeftQueueThreadFunc()
        {
            rclcpp::Rate rate(100); // Adjust the rate as needed
            while (rclcpp::ok())
            {
                rclcpp::spin_some(rosNode);
                rate.sleep();
            }
        }

        void rosRightQueueThreadFunc()
        {
            rclcpp::Rate rate(100); // Adjust the rate as needed
            while (rclcpp::ok())
            {
                rclcpp::spin_some(rosNode);
                rate.sleep();
            }
        }

        physics::ModelPtr model;
        physics::JointPtr rightJoint;
        physics::JointPtr leftJoint;
        common::PID pid;

        rclcpp::Node::SharedPtr rosNode;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rosLeftSub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rosRightSub;

        std::thread rosLeftQueueThread;
        std::thread rosRightQueueThread;
    };
    GZ_REGISTER_MODEL_PLUGIN(HummingbotPlugin);
} // namespace gazebo
