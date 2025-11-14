#ifndef MY_CONTROLLER_HPP_
#define MY_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace my_controller
{
    class MyController : public controller_interface::ControllerInterface
    {
    public:
        MyController() = default;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

        controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

    private:
        // Parameters
        double max_velocity_;
        std::vector<double> kp_;

        // Internal
        size_t num_joints_;
        std::vector<double> target_cache_;
        realtime_tools::RealtimeBuffer<std::vector<double>> rt_target_pos_;

        // ROS
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;

        // Helpers
        controller_interface::InterfaceConfiguration get_interface_configuration() const;
    };
} // namespace my_controller

#endif
