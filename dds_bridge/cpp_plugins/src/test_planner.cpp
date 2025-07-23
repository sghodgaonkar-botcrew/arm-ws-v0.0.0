#include "ik_solver_w_rws.hpp"
#include "planner_v0.hpp"
#include "robot_model.h"
#include <cmath>
#include <iostream>
#include <ompl/config.h>

int main(int argc, char *argv[]) {
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

  if (solved) {
    std::cout << "Planning successful!" << std::endl;

    // Print solution path
    planner.printSolutionPath();

    // Save solution to file
    planner.saveSolutionPath("solution_path.txt");

    // Get solution as matrix for further processing
    Eigen::MatrixXd solution_path = planner.getSolutionPath();
    std::cout << "Solution path has " << solution_path.rows() << " waypoints"
              << std::endl;

  } else {
    std::cout << "No solution found" << std::endl;
  }

  return 0;
}