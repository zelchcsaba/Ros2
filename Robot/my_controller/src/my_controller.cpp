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

        return {};
    }

    controller_interface::InterfaceConfiguration MyController::state_interface_configuration() const
    {
        // TODO 4
        // Specify the state interfaces that this controller will read from.
        // You can run `ros2 control list_hardware_interfaces` in your terminal to view all available interfaces.

        return {};
    }

    controller_interface::CallbackReturn MyController::on_configure(const rclcpp_lifecycle::State &)
    {
        // TODO 1
        // Read controller parameters (i.e. max_velocity, kp) from node parameters and initialize related variables.
        // max_velocity_ = ...
        // kp_ = ...
        // num_joints_ = ...
        // target_cache_ = ...

        // TODO 2
        // Create a subscription to receive target joint positions and update rt_target_pos_ accordingly.

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn MyController::on_activate(const rclcpp_lifecycle::State &)
    {
        // TODO 5
        // Initialize the real-time target position buffer with the current joint positions.
        // Also, set the command interfaces to the current joint positions to avoid jumps.

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type MyController::update(const rclcpp::Time &, const rclcpp::Duration &period)
    {
        // TODO 6
        // Get latest target from the real-time buffer

        // TODO 7
        // For each joint, update its command interface so that the joint moves towards its target position.
        // Make sure that the velocity for each joint does not exceed max_velocity_.

        return controller_interface::return_type::OK;
    }
} // namespace my_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_controller::MyController, controller_interface::ControllerInterface)
