#include "planner_v0.hpp"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <sstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

PlannerV0::PlannerV0(RobotModel &robot_model, double ground_threshold)
    : robot_model_(robot_model), ground_threshold_(ground_threshold),
      solution_found_(false) {
  initializePlanning();
  std::cout << "PlannerV0 initialized with thread safety features" << std::endl;
}

void PlannerV0::initializePlanning() {
  // Construct the state space (6D joint configuration)
  space_ = std::make_shared<ob::RealVectorStateSpace>(6);

  // Set default joint limits
  setDefaultJointLimits();

  // Create SimpleSetup
  simple_setup_ = std::make_unique<og::SimpleSetup>(space_);

  // Set state validity checker
  simple_setup_->setStateValidityChecker(
      [this](const ob::State *state) { return this->isStateValid(state); });

  // Set up pSBL planner
  auto psbl_planner =
      std::make_shared<og::pSBL>(simple_setup_->getSpaceInformation());
  simple_setup_->setPlanner(psbl_planner);

  std::cout << "Planner set to: pSBL (Parallel Single-Query Bi-directional "
               "Lazy collision checking)"
            << std::endl;
}

void PlannerV0::setStartConfiguration(const JointConfig &start_config) {
  ob::ScopedState<> start(space_);
  jointConfigToState(start_config, start.get());
  simple_setup_->setStartState(start);
}

void PlannerV0::setGoalConfiguration(const JointConfig &goal_config) {
  ob::ScopedState<> goal(space_);
  jointConfigToState(goal_config, goal.get());
  simple_setup_->setGoalState(goal);
}

void PlannerV0::setJointLimits(const JointConfig &lower_bounds,
                               const JointConfig &upper_bounds) {
  ob::RealVectorBounds bounds(6);
  for (int i = 0; i < 6; ++i) {
    bounds.setLow(i, lower_bounds[i]);
    bounds.setHigh(i, upper_bounds[i]);
  }
  space_->setBounds(bounds);
}

void PlannerV0::setDefaultJointLimits() {
  ob::RealVectorBounds bounds(6);
  for (int i = 0; i < 6; ++i) {
    bounds.setLow(i, robot_model_.getJointLimitsLower()[i]);
    bounds.setHigh(i, robot_model_.getJointLimitsUpper()[i]);
  }
  space_->setBounds(bounds);
}

bool PlannerV0::solve(double planning_time) {
  // Setup the planner
  simple_setup_->setup();

  // Attempt to solve the problem
  ob::PlannerStatus solved = simple_setup_->solve(planning_time);

  if (solved) {
    solution_found_.store(true, std::memory_order_release);
    // Simplify the solution
    simple_setup_->simplifySolution();
    std::cout << "Planning completed successfully with "
              << collision_check_count_.load() << " collision checks performed."
              << std::endl;
    return true;
  } else {
    solution_found_.store(false, std::memory_order_release);
    std::cout << "Planning failed after " << collision_check_count_.load()
              << " collision checks." << std::endl;
    return false;
  }
}

Eigen::MatrixXd PlannerV0::getSolutionPath() const {
  if (!solution_found_.load(std::memory_order_acquire)) {
    return Eigen::MatrixXd();
  }

  const og::PathGeometric &path = simple_setup_->getSolutionPath();
  int num_states = path.getStateCount();

  Eigen::MatrixXd solution_matrix(num_states, 6);

  for (int i = 0; i < num_states; ++i) {
    const ob::State *state = path.getState(i);
    JointConfig joint_config = stateToJointConfig(state);
    solution_matrix.row(i) = joint_config;
  }

  return solution_matrix;
}

void PlannerV0::printSolutionPath() const {
  if (!solution_found_.load(std::memory_order_acquire)) {
    std::cout << "No solution found" << std::endl;
    return;
  }

  std::cout << "Found solution:" << std::endl;
  simple_setup_->getSolutionPath().printAsMatrix(std::cout);
}

bool PlannerV0::saveSolutionPath(const std::string &filename) const {
  if (!solution_found_.load(std::memory_order_acquire)) {
    std::cerr << "No solution to save" << std::endl;
    return false;
  }

  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Error: Could not open " << filename << " for writing"
              << std::endl;
    return false;
  }

  simple_setup_->getSolutionPath().printAsMatrix(outFile);
  outFile.close();
  std::cout << "Solution path saved to " << filename << std::endl;
  return true;
}

bool PlannerV0::hasSolution() const {
  return solution_found_.load(std::memory_order_acquire);
}

og::SimpleSetup &PlannerV0::getSimpleSetup() { return *simple_setup_; }

int PlannerV0::getCollisionCheckCount() const {
  return collision_check_count_.load(std::memory_order_acquire);
}

bool PlannerV0::isStateValid(const ob::State *state) const {
  // Cast the abstract state type to the type we expect
  const auto *joint_state = state->as<ob::RealVectorStateSpace::StateType>();

  // Convert to JointConfig type
  JointConfig joint_config;
  joint_config << joint_state->values[0], joint_state->values[1],
      joint_state->values[2], joint_state->values[3], joint_state->values[4],
      joint_state->values[5];

  // Thread-safe collision checking with mutex protection
  bool is_valid = false;
  {
    std::lock_guard<std::mutex> lock(collision_check_mutex_);

    // Measure time taken for collision check in microseconds
    auto start_time = std::chrono::high_resolution_clock::now();

    is_valid =
        robot_model_.isValidConfiguration(joint_config, ground_threshold_);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                           end_time - start_time)
                           .count();

    // Increment collision check counter for monitoring
    collision_check_count_++;

    // Only print every 1000th collision check to avoid spam
    if (collision_check_count_ % 1000 == 0) {
      std::cout << "[PlannerV0::isStateValid] Collision check #"
                << collision_check_count_ << " took " << duration_us
                << " microseconds." << std::endl;
    }
  }

  return is_valid;
}

JointConfig PlannerV0::stateToJointConfig(const ob::State *state) const {
  const auto *joint_state = state->as<ob::RealVectorStateSpace::StateType>();

  JointConfig joint_config;
  joint_config << joint_state->values[0], joint_state->values[1],
      joint_state->values[2], joint_state->values[3], joint_state->values[4],
      joint_state->values[5];

  return joint_config;
}

void PlannerV0::jointConfigToState(const JointConfig &joint_config,
                                   ob::State *state) const {
  auto *joint_state = state->as<ob::RealVectorStateSpace::StateType>();

  for (int i = 0; i < 6; ++i) {
    joint_state->values[i] = joint_config[i];
  }
}
