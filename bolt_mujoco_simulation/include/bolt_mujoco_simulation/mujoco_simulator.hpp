#pragma once

#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

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
  mjModel *m = NULL; // MuJoCo model
  mjData *d = NULL;  // MuJoCo data
  mjvCamera cam;     // abstract camera
  mjvOption opt;     // visualization options
  mjvScene scn;      // abstract scene
  mjrContext con;    // custom GPU context

   // mouse interaction
  bool button_left   = false;
  bool button_middle = false;
  bool button_right  = false;
  double lastx       = 0;
  double lasty       = 0;


  // Buffers for data exchange with ROS2-control
  std::vector<double> pos_cmd;
  std::vector<double> vel_cmd;
  std::vector<double> pos_state;
  std::vector<double> vel_state;
  std::vector<double> eff_state;
  std::vector<double> stiff; // Proportional gain
  std::vector<double> damp;  // Derivative gain

  // Safety guards for buffers
  std::mutex state_mutex;
  std::mutex command_mutex;

  // Control input callback for the solver
  static void controlCB(const mjModel *m, mjData *d);
  void controlCBImpl(const mjModel *m, mjData *d);

  // Call this in a separate thread
  static int simulate(const std::string &model_xml);
  int simulateImpl(const std::string &model_xml);

  static void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods);
  void keyboardCBImpl(GLFWwindow* window, int key, int scancode, int act, int mods);

  static void mouseButtonCB(GLFWwindow* window, int button, int act, int mods);
  void mouseButtonCBImpl(GLFWwindow* window, int button, int act, int mods);

  // Non-blocking
  void read(std::vector<double> &pos, std::vector<double> &vel,
            std::vector<double> &eff);
  void write(const std::vector<double> &pos, const std::vector<double> &vel,
             const std::vector<double> &stiff, const std::vector<double> &damp);
};
} // namespace bolt_mujoco_simulation
