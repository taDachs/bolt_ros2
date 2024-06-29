#include "bolt_mujoco_simulation/system_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "bolt_mujoco_simulation/mujoco_simulator.hpp"
#include "bolt_mujoco_simulation/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bolt_mujoco_simulation {
Simulator::CallbackReturn Simulator::on_init(
    const hardware_interface::HardwareInfo &info) {
  // Keep an internal copy of the given configuration
  if (hardware_interface::SystemInterface::on_init(info) !=
      Simulator::CallbackReturn::SUCCESS) {
    return Simulator::CallbackReturn::ERROR;
  }

  // Start the simulator in parallel.
  // Let the thread's destructor clean-up all resources
  // once users close the simulation window.
  m_mujoco_model = info_.hardware_parameters["mujoco_model"];
  m_simulation = std::thread(MuJoCoSimulator::simulate, m_mujoco_model);
  m_simulation.detach();

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    m_positions[joint.name] = std::numeric_limits<double>::quiet_NaN();
    m_velocities[joint.name] = std::numeric_limits<double>::quiet_NaN();
    m_efforts[joint.name] = std::numeric_limits<double>::quiet_NaN();
    m_position_commands[joint.name] = std::numeric_limits<double>::quiet_NaN();
    m_velocity_commands[joint.name] = 0.0;
    m_stiffness[joint.name] = std::stod(joint.parameters.at("p"));
    m_damping[joint.name] = std::stod(joint.parameters.at("d"));

    if (joint.command_interfaces.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs two possible command interfaces.",
                   joint.name.c_str());
      return Simulator::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[1].name ==
              hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs the following command interfaces in that "
                   "order: %s, %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return Simulator::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs 3 state interfaces.", joint.name.c_str());
      return Simulator::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT)) {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs the following state interfaces in that "
                   "order: %s, %s, and %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY,
                   hardware_interface::HW_IF_EFFORT);
      return Simulator::CallbackReturn::ERROR;
    }
  }

  return Simulator::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Simulator::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto &joint : info_.joints) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION,
        &m_positions[joint.name]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY,
        &m_velocities[joint.name]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &m_efforts[joint.name]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Simulator::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto &joint : info_.joints) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION,
        &m_position_commands[joint.name]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY,
        &m_velocity_commands[joint.name]));
  }

  return command_interfaces;
}

Simulator::return_type Simulator::prepare_command_mode_switch(
    [[maybe_unused]] const std::vector<std::string> &start_interfaces,
    [[maybe_unused]] const std::vector<std::string> &stop_interfaces) {
  // TODO: Exclusive OR for position and velocity commands

  return return_type::OK;
}

Simulator::return_type Simulator::read(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period) {
  MuJoCoSimulator::getInstance().read(m_positions, m_velocities, m_efforts);

  // Start with the current positions as safe default, but let active
  // controllers overrride them in each cycle.
  if (std::any_of(m_position_commands.begin(), m_position_commands.end(),
                  [](auto v) { return std::isnan(v.second); })) {
    m_position_commands = m_positions;
  }

  return return_type::OK;
}

Simulator::return_type Simulator::write(
    [[maybe_unused]] const rclcpp::Time &time,
    [[maybe_unused]] const rclcpp::Duration &period) {
  MuJoCoSimulator::getInstance().write(m_position_commands, m_velocity_commands,
                                       m_stiffness, m_damping);
  return return_type::OK;
}
}  // namespace bolt_mujoco_simulation

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bolt_mujoco_simulation::Simulator,
                       hardware_interface::SystemInterface)
