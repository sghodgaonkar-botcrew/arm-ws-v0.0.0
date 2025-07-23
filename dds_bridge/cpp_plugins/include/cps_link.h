#ifndef CPS_LINK_H
#define CPS_LINK_H

#include <memory>
#include <string>
#include <vector>

// Forward declarations
class RemoteAPIClient;
// Note: jsoncons::json is a typedef, so we can't forward declare it
// It will be included in the implementation file

/**
 * @brief CoppeliaSim Robot Control Interface
 *
 * This header provides functions for controlling a 6-DOF robot arm
 * in CoppeliaSim simulation environment.
 */
class CPSLink {
private:
  static RemoteAPIClient *g_client;
  // Using void* to avoid including jsoncons in header
  static void *g_joint_handles;

public:
  /**
   * @brief Initialize joint handles for the robot arm
   * @param client Reference to the RemoteAPIClient instance
   *
   * This function retrieves handles for all 6 joints of the robot arm
   * and stores them for later use in motion control.
   */
  static void initJointHandles(RemoteAPIClient &client);

  /**
   * @brief Move robot to a specific joint configuration
   * @param joint_config_degrees Target joint angles in degrees
   * @param max_vel Maximum velocity for each joint (rad/s)
   * @param max_accel Maximum acceleration for each joint (rad/s²)
   * @param max_jerk Maximum jerk for each joint (rad/s³)
   *
   * This function converts the target configuration from degrees to radians
   * and executes the motion using CoppeliaSim's moveToConfig function.
   */
  static void moveToJointConfig(const std::vector<double> &joint_config_degrees,
                                double max_vel = 785.4,
                                double max_accel = 60000.0,
                                double max_jerk = 1.0);

  /**
   * @brief Check if the client is properly initialized
   * @return true if client is initialized, false otherwise
   */
  static bool isInitialized();

  /**
   * @brief Get the number of joints in the robot arm
   * @return Number of joints (default: 6)
   */
  static int getJointCount();

  /**
   * @brief Reset the internal state
   *
   * This function clears the stored client and joint handles.
   */
  static void reset();
};

/**
 * @brief Example usage function
 *
 * This function demonstrates how to use the CPSLink class
 * to control a robot arm in CoppeliaSim.
 */
void exampleRobotControl();

#endif // CPS_LINK_H