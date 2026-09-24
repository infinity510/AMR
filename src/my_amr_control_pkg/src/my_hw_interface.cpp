#include "my_amr_control_pkg/my_hw_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_amr_control_pkg
{

    hardware_interface::CallbackReturn MyAMRHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Do wheels ke liye memory allocate karein: [0] = Left wheel, [1] = Right wheel
        hw_commands_velocity_.resize(info_.joints.size(), 0.0);
        hw_states_position_.resize(info_.joints.size(), 0.0);
        hw_states_velocity_.resize(info_.joints.size(), 0.0);

        // ESP32 micro-ROS ke sath communicate karne ke liye internal ROS 2 node banayein
        hw_node_ = std::make_shared<rclcpp::Node>("amr_hw_bridge_node");

        // Motor commands publish karne ke liye publisher banayein
        esp32_publisher_ = hw_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor_commands", 10);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MyAMRHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MyAMRHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn MyAMRHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (size_t i = 0; i < hw_commands_velocity_.size(); ++i)
        {
            hw_commands_velocity_[i] = 0.0;
            hw_states_position_[i] = 0.0;
            hw_states_velocity_[i] = 0.0;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MyAMRHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MyAMRHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        // Temporary simulation feedback: velocity ko integrate karke position update karein
        for (size_t i = 0; i < hw_commands_velocity_.size(); ++i)
        {
            hw_states_velocity_[i] = hw_commands_velocity_[i];
            hw_states_position_[i] += hw_commands_velocity_[i] * period.seconds();
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MyAMRHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // diff_drive_controller se aayi wheel velocities ko ESP32 ke liye publish karein
        std_msgs::msg::Float64MultiArray motor_msg;
        motor_msg.data.push_back(hw_commands_velocity_[0]);
        motor_msg.data.push_back(hw_commands_velocity_[1]);

        esp32_publisher_->publish(motor_msg);

        return hardware_interface::return_type::OK;
    }

} // namespace my_amr_control_pkg

// Yeh macro namespace block ke bahar hona zaroori hai
PLUGINLIB_EXPORT_CLASS(
    my_amr_control_pkg::MyAMRHardwareInterface,
    hardware_interface::SystemInterface)