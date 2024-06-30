#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace bolt_mujoco_simulation {

class MuJoCoSimulator {
 private:
  MuJoCoSimulator();
  void syncStates();

 public:
  // Modern singleton approach
  MuJoCoSimulator(const MuJoCoSimulator &) = delete;
  MuJoCoSimulator &operator=(const MuJoCoSimulator &) = delete;
  MuJoCoSimulator(MuJoCoSimulator &&) = delete;
  MuJoCoSimulator &operator=(MuJoCoSimulator &&) = delete;

  // Use this in ROS2 code
  static MuJoCoSimulator &getInstance() {
    static MuJoCoSimulator simulator;
    return simulator;
  }

  // MuJoCo data structures
  mjModel *m = NULL;  // MuJoCo model
  mjData *d = NULL;   // MuJoCo data
  mjvCamera cam;      // abstract camera
  mjvOption opt;      // visualization options
  mjvScene scn;       // abstract scene
  mjrContext con;     // custom GPU context

  // mouse interaction
  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;
  double lastx = 0;
  double lasty = 0;

  // Buffers for data exchange with ROS2-control
  std::map<std::string, double> pos_cmd;
  std::map<std::string, double> vel_cmd;
  std::map<std::string, double> eff_cmd;
  std::map<std::string, double> pos_state;
  std::map<std::string, double> vel_state;
  std::map<std::string, double> eff_state;
  std::map<std::string, double> k_p;  // Proportional gain
  std::map<std::string, double> k_d;   // Derivative gain
  std::map<std::string, double> k_t;   // feedforward (torque) gain

  int freeflyer_nq;

  // Safety guards for buffers
  std::mutex state_mutex;
  std::mutex command_mutex;

  // ROS node
  std::shared_ptr<rclcpp::Node> node;

  // Control input callback for the solver
  static void controlCB(const mjModel *m, mjData *d);
  void controlCBImpl(const mjModel *m, mjData *d);

  // Call this in a separate thread
  static int simulate(const std::string &model_xml, const std::string &mesh_dir);
  int simulateImpl(const std::string &model_xml, const std::string &mesh_dir);

  static void keyboardCB(GLFWwindow *window, int key, int scancode, int act,
                         int mods);
  void keyboardCBImpl(GLFWwindow *window, int key, int scancode, int act,
                      int mods);

  static void mouseButtonCB(GLFWwindow *window, int button, int act, int mods);
  void mouseButtonCBImpl(GLFWwindow *window, int button, int act, int mods);

  // Mouse move callback
  static void mouseMoveCB(GLFWwindow *window, double xpos, double ypos);
  void mouseMoveCBImpl(GLFWwindow *window, double xpos, double ypos);

  // Scroll callback
  static void scrollCB(GLFWwindow *window, double xoffset, double yoffset);
  void scrollCBImpl(GLFWwindow *window, double xoffset, double yoffset);

  // Non-blocking
  void read(std::map<std::string, double> &pos,
            std::map<std::string, double> &vel,
            std::map<std::string, double> &eff);
  void write(const std::map<std::string, double> &pos,
             const std::map<std::string, double> &vel,
             const std::map<std::string, double> &eff,
             const std::map<std::string, double> &k_p,
             const std::map<std::string, double> &k_d,
             const std::map<std::string, double> &k_t);
};
}  // namespace bolt_mujoco_simulation
