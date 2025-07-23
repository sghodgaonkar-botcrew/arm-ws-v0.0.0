#include "cps_link.h"
#include "RemoteAPIClient.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <jsoncons/json.hpp>

// Static member initialization
RemoteAPIClient *CPSLink::g_client = nullptr;
void *CPSLink::g_joint_handles = nullptr;

void CPSLink::initJointHandles(RemoteAPIClient &client) {
  g_client = &client;

  // Get joint handles - assuming 6 joints for the robot arm
  auto *joint_handles = new jsoncons::json(jsoncons::json::array());
  for (int i = 0; i < 6; i++) {
    jsoncons::json joint_path = jsoncons::json::array();
    std::string joint_name = "/joint_" + std::to_string(i + 1);
    std::cout << "Joint path string: " << joint_name << std::endl;
    joint_path.push_back(joint_name);
    auto joint_handle = client.call("sim.getObject", joint_path);
    (*joint_handles).push_back(joint_handle[0]);
    std::cout << "Joint " << (i + 1) << " handle: " << joint_handle[0]
              << std::endl;
  }
  g_joint_handles = joint_handles;
}

void CPSLink::moveToJointConfig(const std::vector<double> &joint_config_degrees,
                                double max_vel, double max_accel,
                                double max_jerk) {
  if (g_client == nullptr || g_joint_handles == nullptr) {
    std::cerr << "Error: Client not initialized. Call initJointHandles() first."
              << std::endl;
    return;
  }

  auto *joint_handles = static_cast<jsoncons::json *>(g_joint_handles);

  // Convert degrees to radians
  jsoncons::json target_pos = jsoncons::json::array();
  for (double angle_deg : joint_config_degrees) {
    target_pos.push_back(angle_deg * M_PI / 180.0);
  }

  // Create parameters for sim.moveToConfig
  jsoncons::json params = jsoncons::json::object();
  params["joints"] = *joint_handles;
  params["targetPos"] = target_pos;

  // Create arrays for motion limits
  jsoncons::json max_vel_array = jsoncons::json::array();
  jsoncons::json max_accel_array = jsoncons::json::array();
  jsoncons::json max_jerk_array = jsoncons::json::array();

  for (int i = 0; i < 6; i++) {
    max_vel_array.push_back(max_vel);
    max_accel_array.push_back(max_accel);
    max_jerk_array.push_back(max_jerk);
  }

  params["maxVel"] = max_vel_array;
  params["maxAccel"] = max_accel_array;
  params["maxJerk"] = max_jerk_array;

  std::cout << "Moving to target configuration..." << std::endl;
  std::cout << "Parameters: " << params << std::endl;

  // Pass parameters as a single argument array
  jsoncons::json args = jsoncons::json::array();
  args.push_back(params);

  auto move_result = g_client->call("sim.moveToConfig", args);
  std::cout << "Move result: " << move_result << std::endl;
}

bool CPSLink::isInitialized() { return g_client != nullptr; }

int CPSLink::getJointCount() {
  return 6; // Default for 6-DOF robot arm
}

void CPSLink::reset() {
  g_client = nullptr;
  if (g_joint_handles != nullptr) {
    delete static_cast<jsoncons::json *>(g_joint_handles);
    g_joint_handles = nullptr;
  }
}

void exampleRobotControl() {
  try {
    RemoteAPIClient client;
    // Use direct function calls instead of the generated objects
    client.setStepping(true);
    client.call("sim.startSimulation");

    // Create a string array for the object path
    jsoncons::json path_array = jsoncons::json::array();
    path_array.push_back("/robot_base_respondable");

    std::cout << "Obtained handle of base = "
              << client.call("sim.getObject", path_array) << std::endl;

    // Initialize joint handles
    CPSLink::initJointHandles(client);

    // Target joint configuration in degrees
    std::vector<double> target_config_degrees = {-65.0534, 12.749,  112.186,
                                                 -72.93,   27.7451, 84.3221};

    // Move to target configuration
    CPSLink::moveToJointConfig(target_config_degrees);

    double start_sim_time =
        client.call("sim.getSimulationTime")[0].as<double>();
    double t = start_sim_time;
    do {
      t = (client.call("sim.getSimulationTime")[0]).as<double>();
      printf("Simulation time: %.2f [s]\n", t);
      client.step();
    } while (t < start_sim_time + 3.0);

    client.call("sim.stopSimulation");
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}