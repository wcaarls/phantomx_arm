#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

#include "phantomx_arm_hw/phantomx_arm_hw.hpp"

#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(rclcpp::get_logger("PhantomXArmHardware"), x)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(rclcpp::get_logger("PhantomXArmHardware"), x)

hardware_interface::CallbackReturn PhantomXArmHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
    
  port_ = info_.hardware_parameters["port"];
  
  if (info_.joints.size() != 7)
  {
    ROS_ERROR_STREAM("Expected 7 joints, got " << info_.joints.size());
    return CallbackReturn::ERROR;
  }

  for (int ii=0; ii < 7; ++ii)
  {
    hardware_interface::ComponentInfo & joint = info_.joints[ii];
    
    if (joint.command_interfaces.size() != 1)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " has " << joint.command_interfaces.size() << " command interfaces, expected 1");
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " has command interface " << joint.command_interfaces[0].name << ", expected " << hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " has " << joint.state_interfaces.size() << " state interfaces, expected 2");
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " state interface 0 is " << joint.state_interfaces[0].name << ", expected " << hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      ROS_ERROR_STREAM("Joint " << joint.name << " state interface 1 is " << joint.state_interfaces[1].name << ", expected " << hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PhantomXArmHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t ii = 0; ii != 7; ++ii)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[ii].name, hardware_interface::HW_IF_POSITION, &pos_[ii]
            )
        );
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[ii].name, hardware_interface::HW_IF_VELOCITY, &vel_[ii]
            )
        );
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PhantomXArmHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t ii = 0; ii != 5; ++ii)
    {
      command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
              info_.joints[ii].name, hardware_interface::HW_IF_POSITION, &cmd_[ii]
          )
      );
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn PhantomXArmHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    for (size_t ii = 0; ii != 7; ++ii)
    {
      pos_[ii] = 0.0f;
      vel_[ii] = 0.0f;
      cmd_[ii] = 0.0f;
    }
    
    if (!driver_.open(port_))
    {
      ROS_ERROR_STREAM("Could not open port" << port_);
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PhantomXArmHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    driver_.close();

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type PhantomXArmHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  unsigned char ids[5] = {1, 2, 3, 4, 5};
  unsigned char buf[5][2][2];

  if (!driver_.is_port_open())
  {
    ROS_ERROR_STREAM("Port is closed");
    return hardware_interface::return_type::ERROR; // exit(1)
  }
  
  if (!driver_.read(ids, 5, 36, 2, &buf[0][0][0]))
  {
    ROS_INFO_STREAM("Could not read from Arbotix board");
    return hardware_interface::return_type::ERROR;
  }

  for (size_t ii = 0; ii != 7; ++ii)
  {
    int pos = buf[ii][0][1]*256+buf[ii][0][0],
        speed = buf[ii][1][1]*256+buf[ii][1][0];

    // Skip fake joints
    if (ii==4) ii = 6;

    if (pos != 65535)
      pos_[ii] = driver_.pos2rad(pos);
    if (speed != 65535)
      vel_[ii] = driver_.speed2rads(speed);
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PhantomXArmHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  unsigned char ids[5] = {1, 2, 3, 4, 5};
  unsigned char buf[5][2];

  if (!driver_.is_port_open())
  {
    ROS_ERROR_STREAM("Port is closed");
    return hardware_interface::return_type::ERROR; // exit(1)
  }

  for (size_t ii = 0; ii != 7; ++ii)
  {
    // Skip fake joints
    if (ii==4) ii = 6;

    int pos = driver_.rad2pos(cmd_[ii]);
    buf[ii][0] = pos&255;
    buf[ii][1] = (pos>>8)&255;
  }

  if (!driver_.write(ids, 5, 30, 1, &buf[0][0]))
  {
    ROS_INFO_STREAM("Could not write to Arbotix board");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(PhantomXArmHardware, hardware_interface::SystemInterface)
