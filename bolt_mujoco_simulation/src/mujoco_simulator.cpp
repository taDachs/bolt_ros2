#include "bolt_mujoco_simulation/mujoco_simulator.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
#include <thread>

#include <sensor_msgs/msg/imu.hpp>

namespace bolt_mujoco_simulation {
MuJoCoSimulator::MuJoCoSimulator() {
  // Ensuring that our normal_dist and random_engine are
  // never undefined, such that setting normal_dist parameters
  // is possible before calling the simulate method.
  setNoiseParams(0.0, 0.0);
}

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

  for (int i = 0; i < m->nu; ++i) {
    // names is one big char array with multiple null terminated strings inside.
    // Take the one that starts at the given address
    std::string name = m->names + m->name_actuatoradr[i];

    d->ctrl[i] =
        k_p[name] * (pos_cmd[name] - d->qpos[i + freeflyer_nq]) +
        k_d[name] * (vel_cmd[name] - d->actuator_velocity[i]) +
        k_t[name] * eff_cmd[name];
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

  // Sensor IDs
  sensor_orient_offset = -1;
  sensor_angular_vel_offset = -1;
  sensor_linear_acc_offset = -1;


  for (int i_sensor = 0; i_sensor < m->nsensor; ++i_sensor) {
    char* sensor_name = m->names + m->name_sensoradr[i_sensor];

    if (strcmp(NAME_GYRO, sensor_name) == 0) {
      sensor_angular_vel_offset = m->sensor_adr[i_sensor];
    } else if (strcmp(NAME_ACCEL, sensor_name) == 0) {
      sensor_linear_acc_offset = m->sensor_adr[i_sensor];
    } else if (strcmp(NAME_ORIENT, sensor_name) == 0) {
      sensor_orient_offset = m->sensor_adr[i_sensor];
    }
  }

  if (sensor_orient_offset == -1
      || sensor_angular_vel_offset == -1
      || sensor_linear_acc_offset == -1) {
    mju_error("Load model error: Sensors seem to be missing or are misconfigured.");
    return 1;
  }

  // a depth of 0 seems to let it revert back to "system defaults", whatever that means
  // maybe we should set it to 1, if we have no need for any kind of history
  const size_t imu_qos_history_depth = 1;
  sensor_imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu", imu_qos_history_depth);

  // or something else other than 60HZ
  const std::chrono::milliseconds imu_publishing_speed(1000 / 60);
  auto sensor_imu_timer = node->create_wall_timer(imu_publishing_speed,
                                                  std::bind(&MuJoCoSimulator::publishImuData, this));

  // Set initial state with the keyframe mechanism from xml
  d = mj_makeData(m);
  mju_copy(d->qpos, m->key_qpos, m->nq);

  freeflyer_nq = m->nq - m->nu;

  // Initialize buffers for ROS2-control.
  for (int i = 0; i < m->nu; ++i) {
    // names is one big char array with multiple null terminated strings inside.
    // Take the one that starts at the given address
    std::string name = m->names + m->name_actuatoradr[i];

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

  sensor_orientation = {0.0, 0.0, 0.0, 0.0};
  sensor_angular_vel = {0.0, 0.0, 0.0};
  sensor_linear_acc = {0.0, 0.0, 0.0};

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

  std::thread imu_publisher_thread([this](){rclcpp::spin(node);});

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
    std::string name = m->names + m->name_actuatoradr[i];

    pos_state[name] = d->qpos[i + freeflyer_nq];
    vel_state[name] = d->actuator_velocity[i];
    eff_state[name] = d->actuator_force[i];
  }

  // sensor_data
  sensor_orientation = {
    d->sensordata[sensor_orient_offset],
    d->sensordata[sensor_orient_offset + 1],
    d->sensordata[sensor_orient_offset + 2],
    d->sensordata[sensor_orient_offset + 3],
  };
  sensor_angular_vel = {
    d->sensordata[sensor_angular_vel_offset],
    d->sensordata[sensor_angular_vel_offset + 1],
    d->sensordata[sensor_angular_vel_offset + 2]
  };
  sensor_linear_acc = {
    d->sensordata[sensor_linear_acc_offset],
    d->sensordata[sensor_linear_acc_offset + 1],
    d->sensordata[sensor_linear_acc_offset + 2]
  };
}

void MuJoCoSimulator::setNoiseParams(double mean, double dev) {
  std::random_device rd;
  random_engine = std::mt19937(rd());
  normal_dist = std::normal_distribution<double>(mean, dev);
}

double MuJoCoSimulator::computeNoise() {
  if (!enable_noise) {
    return 0.0;
  }

  return normal_dist(random_engine);
}

void MuJoCoSimulator::publishImuData() {
  auto now = node->get_clock()->now();
  sensor_msgs::msg::Imu msg;
  msg.header.frame_id = NAME_SENSOR_IMU_LINK;
  msg.header.stamp.sec = now.seconds();
  msg.header.stamp.nanosec = now.nanoseconds();

  msg.angular_velocity.x = computeNoise() + d->sensordata[sensor_angular_vel_offset];
  msg.angular_velocity.y = computeNoise() + d->sensordata[sensor_angular_vel_offset + 1];
  msg.angular_velocity.z = computeNoise() + d->sensordata[sensor_angular_vel_offset + 2];
  msg.linear_acceleration.x = computeNoise() + d->sensordata[sensor_linear_acc_offset];
  msg.linear_acceleration.y = computeNoise() + d->sensordata[sensor_linear_acc_offset + 1];
  msg.linear_acceleration.z = computeNoise() + d->sensordata[sensor_linear_acc_offset + 2];
  msg.orientation.w = computeNoise() + d->sensordata[sensor_orient_offset];
  msg.orientation.x = computeNoise() + d->sensordata[sensor_orient_offset + 1];
  msg.orientation.y = computeNoise() + d->sensordata[sensor_orient_offset + 2];
  msg.orientation.z = computeNoise() + d->sensordata[sensor_orient_offset + 3];

  sensor_imu_publisher->publish(msg);
}
}  // namespace bolt_mujoco_simulation
