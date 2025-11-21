#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "my_controller/my_controller.hpp"

namespace my_controller
{
    controller_interface::CallbackReturn MyController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration MyController::get_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        conf.names.reserve(num_joints_);
        for (size_t i = 0; i < num_joints_; i++)
        {
            conf.names.push_back("joint_" + std::to_string(i + 1) + "/" + hardware_interface::HW_IF_POSITION);
        }
        return conf;
    }

    controller_interface::InterfaceConfiguration MyController::command_interface_configuration() const
    {
        // TODO 3
        // Specify the command interfaces that this controller will write to.
        // You can run `ros2 control list_hardware_interfaces` in your terminal to view all available interfaces.

        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        conf.names.reserve(num_joints_);

        for (size_t i = 0; i < num_joints_; i++)
        {
            conf.names.push_back("joint_" + std::to_string(i + 1) + "/" + hardware_interface::HW_IF_POSITION);
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration MyController::state_interface_configuration() const
    {
        // TODO 4
        // Specify the state interfaces that this controller will read from.
        // You can run `ros2 control list_hardware_interfaces` in your terminal to view all available interfaces.

        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        conf.names.reserve(num_joints_);

        for (size_t i = 0; i < num_joints_; i++)
        {
            conf.names.push_back("joint_" + std::to_string(i + 1) + "/" + hardware_interface::HW_IF_POSITION);
        }

        return conf;
    }

    controller_interface::CallbackReturn MyController::on_configure(const rclcpp_lifecycle::State &)
    {
        // TODO 1
        // Read controller parameters (i.e. max_velocity, kp) from node parameters and initialize related variables.
        max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
        kp_ = get_node()->get_parameter("kp").as_double_array();
        num_joints_ = kp_.size();
        target_cache_.assign(num_joints_, 0.0);



        // TODO 2
        // Create a subscription to receive target joint positions and update rt_target_pos_ accordingly.

        cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
            "~/command", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
            {
                rt_target_pos_.writeFromNonRT(msg->data);
            });
        return controller_interface::CallbackReturn::SUCCESS;

    }

    controller_interface::CallbackReturn MyController::on_activate(const rclcpp_lifecycle::State &)
    {
        // TODO 5
        // Initialize the real-time target position buffer with the current joint positions.
        // Also, set the command interfaces to the current joint positions to avoid jumps.

        std::vector<double> current(num_joints_, 0.0);
        for (size_t i = 0; i < num_joints_; i++)
        {
            current[i] = state_interfaces_[i].get_value();
            command_interfaces_[i].set_value(current[i]);
        }
        rt_target_pos_.initRT(current);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type MyController::update(const rclcpp::Time &, const rclcpp::Duration &period)
    {
        // TODO 6
        // Get latest target from the real-time buffer
        std::copy_n(rt_target_pos_.readFromRT()->data(), num_joints_, target_cache_.data());

        // TODO 7
        // For each joint, update its command interface so that the joint moves towards its target position.
        // Make sure that the velocity for each joint does not exceed max_velocity_.

        const double max_step = max_velocity_ * period.seconds();
        for (size_t i = 0; i < num_joints_; i++)
        {
            const double error = target_cache_[i] - state_interfaces_[i].get_value();
            const double u = std::clamp(kp_[i] * error, -max_step, +max_step);
            command_interfaces_[i].set_value(state_interfaces_[i].get_value() + u);
        }

        return controller_interface::return_type::OK;
    }
} // namespace my_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_controller::MyController, controller_interface::ControllerInterface)
