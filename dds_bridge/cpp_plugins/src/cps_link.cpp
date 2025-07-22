#include "RemoteAPIClient.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <jsoncons/json.hpp>

int main(int argc, char *argv[]) {
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

    // Get joint handles - assuming 6 joints for the robot arm
    jsoncons::json joint_handles = jsoncons::json::array();
    for (int i = 0; i < 6; i++) {
      jsoncons::json joint_path = jsoncons::json::array();
      std::string joint_name = "/joint_" + std::to_string(i + 1);
      std::cout << "Joint path string: " << joint_name << std::endl;
      joint_path.push_back(joint_name);
      auto joint_handle = client.call("sim.getObject", joint_path);
      joint_handles.push_back(joint_handle[0]);
      std::cout << "Joint " << (i + 1) << " handle: " << joint_handle[0]
                << std::endl;
    }

    // Target joint configuration (converting from degrees to radians)
    jsoncons::json target_pos = jsoncons::json::array();
    target_pos.push_back(-65.0534 * M_PI /
                         180.0);                  // -65.0534 degrees to radians
    target_pos.push_back(12.749 * M_PI / 180.0);  // 12.749 degrees to radians
    target_pos.push_back(112.186 * M_PI / 180.0); // 112.186 degrees to radians
    target_pos.push_back(-72.93 * M_PI / 180.0);  // -72.93 degrees to radians
    target_pos.push_back(27.7451 * M_PI / 180.0); // 27.7451 degrees to radians
    target_pos.push_back(84.3221 * M_PI / 180.0); // 84.3221 degrees to radians

    // Create parameters for sim.moveToConfig
    jsoncons::json params = jsoncons::json::object();
    params["joints"] = joint_handles;
    params["targetPos"] = target_pos;
    params["maxVel"] =
        jsoncons::json::array({785.4, 785.4, 785.4, 785.4, 785.4,
                               785.4}); // 785.4 rad/s for all joints
    params["maxAccel"] =
        jsoncons::json::array({60000.0, 60000.0, 60000.0, 60000.0, 60000.0,
                               60000.0}); // 60000 rad/s² for all joints
    params["maxJerk"] = jsoncons::json::array(
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}); // 1 rad/s³ for all joints

    std::cout << "Moving to target configuration..." << std::endl;
    std::cout << "Parameters: " << params << std::endl;

    // Pass parameters as a single argument array
    jsoncons::json args = jsoncons::json::array();
    args.push_back(params);
    auto move_result = client.call("sim.moveToConfig", args);
    std::cout << "Move result: " << move_result << std::endl;

    double start_sim_time =
        client.call("sim.getSimulationTime")[0].as<double>();
    double t = start_sim_time;
    do {
      t = (client.call("sim.getSimulationTime")[0]).as<double>();
      printf("Simulation time: %.2f [s]\n", t);
      client.step();
    } while (t < start_sim_time + 3.0);

    client.call("sim.stopSimulation");
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}