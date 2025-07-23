#ifndef PLANNER_V0_HPP
#define PLANNER_V0_HPP

#include "robot_model.h"
#include "workspace_types.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <string>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @brief Motion planner class using OMPL for 6-DOF robot arm
 *
 * This class encapsulates the motion planning functionality using OMPL's
 * RRT-Connect planner for a 6-DOF robot arm. It provides methods to set
 * start/goal configurations and solve motion planning problems.
 */
class PlannerV0 {
public:
  /**
   * @brief Constructor
   * @param robot_model Reference to the robot model for collision checking
   * @param ground_threshold Minimum distance from ground for valid
   * configurations
   */
  explicit PlannerV0(RobotModel &robot_model, double ground_threshold = 0.01);

  /**
   * @brief Destructor
   */
  ~PlannerV0() = default;

  /**
   * @brief Set the start joint configuration
   * @param start_config 6D joint configuration vector
   */
  void setStartConfiguration(const JointConfig &start_config);

  /**
   * @brief Set the goal joint configuration
   * @param goal_config 6D joint configuration vector
   */
  void setGoalConfiguration(const JointConfig &goal_config);

  /**
   * @brief Set joint limits for all joints
   * @param lower_bounds Lower bounds for each joint (in radians)
   * @param upper_bounds Upper bounds for each joint (in radians)
   */
  void setJointLimits(const JointConfig &lower_bounds,
                      const JointConfig &upper_bounds);

  /**
   * @brief Set default joint limits for UR10 robot
   */
  void setDefaultJointLimits();

  /**
   * @brief Solve the motion planning problem
   * @param planning_time Maximum planning time in seconds
   * @return true if solution found, false otherwise
   */
  bool solve(double planning_time = 1.0);

  /**
   * @brief Get the solution path as a matrix
   * @return Matrix where each row is a joint configuration
   */
  Eigen::MatrixXd getSolutionPath() const;

  /**
   * @brief Print the solution path to console
   */
  void printSolutionPath() const;

  /**
   * @brief Save the solution path to a file
   * @param filename Output file name
   * @return true if successful, false otherwise
   */
  bool saveSolutionPath(const std::string &filename) const;

  /**
   * @brief Check if a solution was found
   * @return true if solution exists, false otherwise
   */
  bool hasSolution() const;

  /**
   * @brief Get the planning setup for advanced configuration
   * @return Reference to the SimpleSetup object
   */
  og::SimpleSetup &getSimpleSetup();

  /**
   * @brief Get collision check statistics
   * @return Number of collision checks performed
   */
  int getCollisionCheckCount() const;

private:
  RobotModel &robot_model_;
  double ground_threshold_;
  std::shared_ptr<ob::RealVectorStateSpace> space_;
  std::unique_ptr<og::SimpleSetup> simple_setup_;
  std::atomic<bool> solution_found_;

  // Thread safety members
  mutable std::mutex robot_model_mutex_;
  mutable std::mutex collision_check_mutex_;
  mutable std::atomic<int> collision_check_count_{0};

  /**
   * @brief Initialize the planning space and setup
   */
  void initializePlanning();

  /**
   * @brief State validity checker function
   * @param state OMPL state to check
   * @return true if state is valid, false otherwise
   */
  bool isStateValid(const ob::State *state) const;

  /**
   * @brief Convert OMPL state to JointConfig
   * @param state OMPL state
   * @return JointConfig vector
   */
  JointConfig stateToJointConfig(const ob::State *state) const;

  /**
   * @brief Convert JointConfig to OMPL state
   * @param joint_config Joint configuration
   * @param state OMPL state to populate
   */
  void jointConfigToState(const JointConfig &joint_config,
                          ob::State *state) const;
};

#endif // PLANNER_V0_HPP
