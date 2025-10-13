#include <vector>
#include <string>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>

#include <phantomx_arm_hw/arbotix.hpp>

class PhantomXArmHardware : public hardware_interface::SystemInterface
{
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PhantomXArmHardware)

    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    Arbotix driver_;
    std::string port_;
  
    double cmd_[7]; // 4 arm joints, 2 fake joints, 1 gripper joint
    double pos_[7];
    double vel_[7];
    
  public:
    PhantomXArmHardware() { }
};
