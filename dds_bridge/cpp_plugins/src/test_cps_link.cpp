#include "RemoteAPIClient.h"
#include "cps_link.h"
#include "ik_solver_w_rws.hpp"
#include "planner_v0.hpp"
#include "robot_model.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <jsoncons/json.hpp>
#include <ompl/config.h>
#include <vector>

int main(int argc, char *argv[]) {
  try {
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // Initialize robot model
    RobotModel robot_model(
        "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf",
        "connection_frame",
        "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.srdf");

    // Initialize IK solver with workspace data
    IKSolverWithRWS ik_solver("generated_workspaces", robot_model);

    if (!ik_solver.isWorkspaceLoaded()) {
      std::cerr << "Error: Failed to load workspace data for IK solver"
                << std::endl;
      return -1;
    }

    std::cout << "IK solver initialized with "
              << ik_solver.getNumWorkspacePoints() << " workspace points"
              << std::endl;

    // Create planner instance
    PlannerV0 planner(robot_model, 0.01);

    // Define start and goal poses as [x, y, z, qx, qy, qz, qw]
    XYZQuat start_pose;
    start_pose << -0.25716, 0, 1.428, 0, -0.7071068, 0, 0.7071068;

    XYZQuat goal_pose;
    goal_pose << 0.5, 0.5, 0.5, 0, 0, 0, 1;

    std::cout << "Start pose: [" << start_pose.transpose() << "]" << std::endl;
    std::cout << "Goal pose: [" << goal_pose.transpose() << "]" << std::endl;

    // Solve IK for start pose
    JointConfig start_config = ik_solver.solve(start_pose);
    std::cout << "Start IK solution: [" << start_config.transpose() << "]"
              << std::endl;

    // Print start configuration in degrees
    std::cout << "Start configuration (degrees): ";
    for (int i = 0; i < start_config.size(); ++i) {
      std::cout << (start_config[i] * 180.0 / M_PI);
      if (i < start_config.size() - 1)
        std::cout << ", ";
    }
    std::cout << std::endl;

    // Solve IK for goal pose
    JointConfig goal_config = ik_solver.solve(goal_pose);
    std::cout << "Goal IK solution: [" << goal_config.transpose() << "]"
              << std::endl;

    // Print goal configuration in degrees
    std::cout << "Goal configuration (degrees): ";
    for (int i = 0; i < goal_config.size(); ++i) {
      std::cout << (goal_config[i] * 180.0 / M_PI);
      if (i < goal_config.size() - 1)
        std::cout << ", ";
    }
    std::cout << std::endl;

    // Set configurations for the planner
    planner.setStartConfiguration(start_config);
    planner.setGoalConfiguration(goal_config);

    // Solve the planning problem
    bool solved = planner.solve(1.0);

    // If planning failed, try once more with longer time
    if (!solved) {
      std::cout << "Planning failed, retrying with 2.0 seconds..." << std::endl;
      solved = planner.solve(2.0);
    }

    if (!solved) {
      std::cout << "No solution found after retry" << std::endl;
      return -1;
    }

    std::cout << "Planning successful!" << std::endl;

    // Get solution as matrix for execution
    Eigen::MatrixXd solution_path = planner.getSolutionPath();
    std::cout << "Solution path has " << solution_path.rows() << " waypoints"
              << std::endl;

    // Initialize simulation
    RemoteAPIClient client;
    client.setStepping(true);
    client.call("sim.startSimulation");

    // Create a string array for the object path
    jsoncons::json path_array = jsoncons::json::array();
    path_array.push_back("/robot_base_respondable");

    std::cout << "Obtained handle of base = "
              << client.call("sim.getObject", path_array) << std::endl;

    // Initialize joint handles using CPSLink class
    CPSLink::initJointHandles(client);

    // Demonstrate additional CPSLink functionality
    std::cout << "CPSLink initialized: "
              << (CPSLink::isInitialized() ? "Yes" : "No") << std::endl;
    std::cout << "Number of joints: " << CPSLink::getJointCount() << std::endl;

    // Execute the solution path sequentially
    for (int i = 0; i < solution_path.rows(); ++i) {
      std::cout << "Moving to waypoint " << (i + 1) << "/"
                << solution_path.rows() << std::endl;

      // Convert from radians to degrees
      std::vector<double> waypoint_degrees;
      for (int j = 0; j < solution_path.cols(); ++j) {
        waypoint_degrees.push_back(solution_path(i, j) * 180.0 / M_PI);
      }

      // Print waypoint in degrees
      std::cout << "Waypoint " << (i + 1) << " (degrees): ";
      for (size_t j = 0; j < waypoint_degrees.size(); ++j) {
        std::cout << waypoint_degrees[j];
        if (j < waypoint_degrees.size() - 1)
          std::cout << ", ";
      }
      std::cout << std::endl;

      // Move to this waypoint
      CPSLink::moveToJointConfig(waypoint_degrees);

      // Wait for movement to complete (simulate for a few seconds)
      // double start_sim_time =
      //     client.call("sim.getSimulationTime")[0].as<double>();
      // double t = start_sim_time;
      // do {
      //   t = (client.call("sim.getSimulationTime")[0]).as<double>();
      //   client.step();
      // } while (t < start_sim_time + 2.0); // Wait 2 seconds per waypoint
    }

    std::cout << "Solution path execution completed!" << std::endl;

    // Wait a bit more to see the final position
    double start_sim_time =
        client.call("sim.getSimulationTime")[0].as<double>();
    double t = start_sim_time;
    do {
      t = (client.call("sim.getSimulationTime")[0]).as<double>();
      client.step();
    } while (t < start_sim_time + 5.0); // Wait 5 more seconds

    client.call("sim.stopSimulation");

    // Clean up CPSLink resources
    CPSLink::reset();
    std::cout << "CPSLink resources cleaned up." << std::endl;

    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}