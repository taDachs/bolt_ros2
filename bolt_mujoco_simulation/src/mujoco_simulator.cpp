#include "bolt_mujoco_simulation/mujoco_simulator.hpp"
#include <memory>
#include <iostream>

namespace bolt_mujoco_simulation {
MuJoCoSimulator::MuJoCoSimulator() {}

void MuJoCoSimulator::keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  getInstance().keyboardCBImpl(window, key, scancode, act, mods);
}

void MuJoCoSimulator::keyboardCBImpl(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  (void)window;
  (void)scancode;
  (void)mods;

  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(m, d);
    mju_copy(d->qpos, m->key_qpos, m->nq); // initial states from xml
    mj_forward(m, d);
  }
}

void MuJoCoSimulator::mouseButtonCB(GLFWwindow* window, int button, int act, int mods)
{
  getInstance().mouseButtonCBImpl(window, button, act, mods);
}

void MuJoCoSimulator::mouseButtonCBImpl(GLFWwindow* window, int button, int act, int mods)
{
  (void)button;
  (void)act;
  (void)mods;

  // update button state
  button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}



void MuJoCoSimulator::controlCB(const mjModel *m, mjData *d) {
  getInstance().controlCBImpl(m, d);
}

void MuJoCoSimulator::controlCBImpl([[maybe_unused]] const mjModel *m,
                                    mjData *d) {
  command_mutex.lock();

  for (size_t i = 0; i < pos_cmd.size(); ++i) {
    // Joint-level impedance control
    d->ctrl[i] = stiff[i] * (pos_cmd[i] - d->qpos[i]) +            // stiffness
                 damp[i] * (vel_cmd[i] - d->actuator_velocity[i]); // damping
    // d->ctrl[i] = 0;
  }
  command_mutex.unlock();
}

int MuJoCoSimulator::simulate(const std::string &model_xml) {
  return getInstance().simulateImpl(model_xml);
}

int MuJoCoSimulator::simulateImpl(const std::string &model_xml) {
  // Make sure that the ROS2-control system_interface only gets valid data in
  // read(). We lock until we are done with simulation setup.
  state_mutex.lock();

  // load and compile model
  char error[1000] = "Could not load binary model";
  m = mj_loadXML(model_xml.c_str(), nullptr, error, 1000);
  if (!m) {
    mju_error_s("Load model error: %s", error);
    return 1;
  }

  // Set initial state with the keyframe mechanism from xml
  d = mj_makeData(m);
  mju_copy(d->qpos, m->key_qpos, m->nq);

  // Initialize buffers for ROS2-control.
  pos_state.resize(m->nu);
  vel_state.resize(m->nu);
  eff_state.resize(m->nu);
  pos_cmd.resize(m->nu);
  vel_cmd.resize(m->nu);
  stiff.resize(m->nu);
  damp.resize(m->nu);

  // Start where we are
  syncStates();
  state_mutex.unlock();

  // visuals
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow *window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  // glfwSetKeyCallback(window, keyboardCB);
  // glfwSetMouseButtonCallback(window, mouseButtonCB);
  // glfwSetCursorPosCallback(window, mouseMoveCB);
  // glfwSetScrollCallback(window, scrollCB);

  // Connect our specific control input callback for MuJoCo's engine.
  mjcb_control = MuJoCoSimulator::controlCB;

  while (!glfwWindowShouldClose(window)) {
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0) {
      mj_step(m, d);

      // Provide fresh data for ROS2-control
      state_mutex.lock();
      syncStates();
      state_mutex.unlock();
      std::cout << d->time << "\n";
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();

  return 0;
}

void MuJoCoSimulator::read(std::vector<double> &pos, std::vector<double> &vel,
                           std::vector<double> &eff) {
  // Realtime in ROS2-control is more important than fresh data exchange.
  if (state_mutex.try_lock()) {
    pos = pos_state;
    vel = vel_state;
    eff = eff_state;
    state_mutex.unlock();
  }
}

void MuJoCoSimulator::write(const std::vector<double> &pos,
                            const std::vector<double> &vel,
                            const std::vector<double> &stiff,
                            const std::vector<double> &damp) {
  // Realtime in ROS2-control is more important than fresh data exchange.
  if (command_mutex.try_lock()) {
    pos_cmd = pos;
    vel_cmd = vel;
    this->stiff = stiff;
    this->damp = damp;
    command_mutex.unlock();
  }
}

void MuJoCoSimulator::syncStates() {
  for (auto i = 0; i < m->nu; ++i) {
    pos_state[i] = d->qpos[i];
    vel_state[i] = d->actuator_velocity[i];
    eff_state[i] = d->actuator_force[i];
  }
}

} // namespace bolt_mujoco_simulation
