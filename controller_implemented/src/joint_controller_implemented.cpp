#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace controller_implemented
{
    class JointControllerImplemented : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
            bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh)
            {
                // Saving Joint Name
                std::string joint_name;
                if (!nh.getParam("joint", joint_name))
                {
                    ROS_ERROR("Could not find joint name");
                    return false;
                }

                // Getting initial Values
                joint_ = hw->getHandle(joint_name);
                command_ = joint_.getPosition();

                // Loading Proportional Gain
                if (!nh.getParam("gain", gain_))
                {
                    ROS_ERROR("Could not find gain value");
                    return false;
                }

                // Start subscriber
                sub_command_ = nh.subscribe<std_msgs::Float64>("command", 1, &JointControllerImplemented::setCommandCB, this);

                return true;
            }

            // Running Controller
            void update(const ros::Time& time, const ros::Duration& period)
            {
                double error = command_ - joint_.getPosition();
                double commanded_effort = error*gain_;
                joint_.setCommand(commanded_effort);
            }

            // Callback for Subscriber
            void setCommandCB(const std_msgs::Float64ConstPtr& msg)
            {
                command_ = msg->data;
            }

            // Controller Startup
            void starting(const ros::Time& time) {}

            // Stopping Controller
            void stopping(const ros::Time& time) {}

        private:
            hardware_interface::JointHandle joint_;
            double gain_;
            double command_;
            ros::Subscriber sub_command_;
    };

    PLUGINLIB_EXPORT_CLASS(controller_implemented::JointControllerImplemented, controller_interface::ControllerBase);
}