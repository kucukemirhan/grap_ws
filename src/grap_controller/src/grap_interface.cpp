#include "grap_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace grap_controller
{
    GrapInterface::GrapInterface()
    {
        
    }

    GrapInterface::~GrapInterface()
    {
        if(grapSerial_.IsOpen())
        {
            try
            {
                grapSerial_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("GrapInterface"), "Something went wrong while closing the connection with port " << port_);
            }
        }
    }

    CallbackReturn GrapInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("GrapInterface"), "No serial port provided! Aborting");
            return CallbackReturn::FAILURE;
        }

        position_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        prev_position_commands_.reserve(info_.joints.size());

        return CallbackReturn::SUCCESS; 

    }

    std::vector<hardware_interface::StateInterface> GrapInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> GrapInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn GrapInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("GrapInterface"), "Starting the robot hardware...");
        position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        prev_position_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        position_states_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        try
        {
            grapSerial_.Open(port_);
            grapSerial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("GrapInterface"), "Something went wrong while interacting with the port " << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("GrapInterface"), "Hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;
    }

    
    CallbackReturn GrapInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("GrapInterface"), "Stopping the robot hardware...");
        if(grapSerial_.IsOpen())
        {
            try
            {
                grapSerial_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("GrapInterface"), "Something went wron while closing the connection with the port " << port_);
                return CallbackReturn::FAILURE;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("GrapInterface"), "Hardware stopped");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GrapInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        position_states_ = position_commands_;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type GrapInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        if(position_commands_ == prev_position_commands_)
        {
            return hardware_interface::return_type::OK;
        }

        std::string msg;
        int base = static_cast<int> (((position_commands_.at(0) + (M_PI/2)) * 180) / M_PI);
        msg.append("j1=");
        msg.append(std::to_string(base));
        msg.append(",");

        int shoulder = 180 - static_cast<int> (((position_commands_.at(1) + (M_PI/2)) * 180) / M_PI);
        msg.append("j2=");
        msg.append(std::to_string(shoulder));
        msg.append(",");

        int elbow = static_cast<int> (((position_commands_.at(2) + (M_PI/2)) * 180) / M_PI);
        msg.append("j3=");
        msg.append(std::to_string(elbow));
        msg.append(",");

        int gripper = static_cast<int> ((-position_commands_.at(3) * 180) / (M_PI/2));
        msg.append("j4=");
        msg.append(std::to_string(gripper));
        msg.append("\n");

        try
        {
            grapSerial_.Write(msg);
        }
        catch(...)
        {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("GrapInterface"), "Something went wrong while sending the message " << msg);
                return hardware_interface::return_type::ERROR;
        }

        prev_position_commands_ = position_commands_;
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(grap_controller::GrapInterface, hardware_interface::SystemInterface);
