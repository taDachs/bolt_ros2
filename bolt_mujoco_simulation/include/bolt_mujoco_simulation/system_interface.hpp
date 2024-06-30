#pragma once

#include <map>
#include <thread>

#include "bolt_mujoco_simulation/mujoco_simulator.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace bolt_mujoco_simulation {
class Simulator : public hardware_interface::SystemInterface {
 public:
  using return_type = hardware_interface::return_type;
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  RCLCPP_SHARED_PTR_DEFINITIONS(Simulator)

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  return_type read(const rclcpp::Time& time,
                   const rclcpp::Duration& period) override;
  return_type write(const rclcpp::Time& time,
                    const rclcpp::Duration& period) override;

 private:
  // Command buffers for the controllers
  std::map<std::string, double> m_position_commands;
  std::map<std::string, double> m_velocity_commands;
  std::map<std::string, double> m_effort_commands;

  // State buffers for the controllers
  std::map<std::string, double> m_positions;
  std::map<std::string, double> m_velocities;
  std::map<std::string, double> m_efforts;

  // gains
  std::map<std::string, double> m_k_p;
  std::map<std::string, double> m_k_d;
  std::map<std::string, double> m_k_t;

  // Run MuJoCo's solver in a separate thread
  std::thread m_simulation;

  // Parameters
  std::string m_mujoco_model;
  std::string m_mesh_dir;
};
}  // namespace bolt_mujoco_simulation
