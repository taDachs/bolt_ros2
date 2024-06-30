#include "bolt_mujoco_simulation/mujoco_simulator.hpp"

#include <filesystem>
#include <iostream>
#include <memory>

namespace bolt_mujoco_simulation {
MuJoCoSimulator::MuJoCoSimulator() {}

void MuJoCoSimulator::keyboardCB(GLFWwindow *window, int key, int scancode,
                                 int act, int mods) {
  getInstance().keyboardCBImpl(window, key, scancode, act, mods);
}

void MuJoCoSimulator::keyboardCBImpl(GLFWwindow *window, int key, int scancode,
                                     int act, int mods) {
  (void)window;
  (void)scancode;
  (void)mods;

  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mju_copy(d->qpos, m->key_qpos, m->nq);  // initial states from xml
    mj_forward(m, d);
  }
}

void MuJoCoSimulator::mouseButtonCB(GLFWwindow *window, int button, int act,
                                    int mods) {
  getInstance().mouseButtonCBImpl(window, button, act, mods);
}

void MuJoCoSimulator::mouseButtonCBImpl(GLFWwindow *window, int button, int act,
                                        int mods) {
  (void)button;
  (void)act;
  (void)mods;

  // update button state
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

void MuJoCoSimulator::mouseMoveCB(GLFWwindow *window, double xpos,
                                  double ypos) {
  getInstance().mouseMoveCBImpl(window, xpos, ypos);
}

void MuJoCoSimulator::mouseMoveCBImpl(GLFWwindow *window, double xpos,
                                      double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void MuJoCoSimulator::scrollCB(GLFWwindow *window, double xoffset,
                               double yoffset) {
  getInstance().scrollCBImpl(window, xoffset, yoffset);
}

void MuJoCoSimulator::scrollCBImpl(GLFWwindow *window, double xoffset,
                                   double yoffset) {
  (void)window;
  (void)xoffset;

  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void MuJoCoSimulator::controlCB(const mjModel *m, mjData *d) {
  getInstance().controlCBImpl(m, d);
}

void MuJoCoSimulator::controlCBImpl([[maybe_unused]] const mjModel *m,
                                    mjData *d) {
  command_mutex.lock();

  for (int i = 0; i < m->nq; ++i) {
    // names is one big char array with multiple null terminated strings inside.
    // Take the one that starts at the given address
    std::string name = m->names + m->name_jntadr[i];

    d->ctrl[i] =
        k_p[name] * (pos_cmd[name] - d->qpos[i]) +             // stiffness
        k_d[name] * (vel_cmd[name] - d->actuator_velocity[i]) +  // damping
        k_t[name] * eff_cmd[name]; // feedforward torque
  }

  command_mutex.unlock();
}

int MuJoCoSimulator::simulate(const std::string &model_xml, const std::string &mesh_dir) {
  return getInstance().simulateImpl(model_xml, mesh_dir);
}

int MuJoCoSimulator::simulateImpl(const std::string &model_xml, const std::string &mesh_dir) {
  // Make sure that the ROS2-control system_interface only gets valid data in
  // read(). We lock until we are done with simulation setup.
  state_mutex.lock();

  node = rclcpp::Node::make_shared("mujoco_simulator");

  auto mj_vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(mj_vfs.get());

  for (const auto& entry : std::filesystem::directory_iterator(mesh_dir))
  {
    mj_addFileVFS(mj_vfs.get(),
                  // Append a forward slash for MuJoCo if non-existent
                  (std::string(mesh_dir).back() == '/') ? mesh_dir.c_str()
                                                        : std::string(mesh_dir + '/').c_str(),
                  entry.path().filename().c_str());
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  m                = mj_loadXML(model_xml.c_str(), mj_vfs.get(), error, 1000);
  if (!m) {
    mju_error_s("Load model error: %s", error);
    return 1;
  }

  // Set initial state with the keyframe mechanism from xml
  d = mj_makeData(m);
  mju_copy(d->qpos, m->key_qpos, m->nq);

  // Initialize buffers for ROS2-control.
  for (int i = 0; i < m->nq; ++i) {
    // names is one big char array with multiple null terminated strings inside.
    // Take the one that starts at the given address
    std::string name = m->names + m->name_jntadr[i];
    ;
    pos_state[name] = 0.0;
    vel_state[name] = 0.0;
    eff_state[name] = 0.0;
    pos_cmd[name] = 0.0;
    vel_cmd[name] = 0.0;
    eff_cmd[name] = 0.0;
    k_p[name] = 0.0;
    k_d[name] = 0.0;
    k_t[name] = 0.0;
  }

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
  glfwSetKeyCallback(window, keyboardCB);
  glfwSetMouseButtonCallback(window, mouseButtonCB);
  glfwSetCursorPosCallback(window, mouseMoveCB);
  glfwSetScrollCallback(window, scrollCB);

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

void MuJoCoSimulator::read(std::map<std::string, double> &pos,
                           std::map<std::string, double> &vel,
                           std::map<std::string, double> &eff) {
  // Realtime in ROS2-control is more important than fresh data exchange.
  if (state_mutex.try_lock()) {
    for (auto &it : pos) {
      pos[it.first] = pos_state[it.first];
      vel[it.first] = vel_state[it.first];
      eff[it.first] = eff_state[it.first];
    }
    state_mutex.unlock();
  }
}

void MuJoCoSimulator::write(const std::map<std::string, double> &pos,
                            const std::map<std::string, double> &vel,
                            const std::map<std::string, double> &eff,
                            const std::map<std::string, double> &k_p,
                            const std::map<std::string, double> &k_d,
                            const std::map<std::string, double> &k_t) {
  // Realtime in ROS2-control is more important than fresh data exchange.
  if (command_mutex.try_lock()) {
    pos_cmd = pos;
    vel_cmd = vel;
    eff_cmd = eff;
    this->k_p = k_p;
    this->k_d = k_d;
    this->k_d = k_t;
    command_mutex.unlock();
  }
}

void MuJoCoSimulator::syncStates() {
  for (auto i = 0; i < m->nu; ++i) {
    std::string name = m->names + m->name_jntadr[i];
    ;
    pos_state[name] = d->qpos[i];
    vel_state[name] = d->actuator_velocity[i];
    eff_state[name] = d->actuator_force[i];
  }
}

}  // namespace bolt_mujoco_simulation
