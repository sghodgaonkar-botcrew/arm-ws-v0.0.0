# C++ Plugins for Robot Arm Control

This directory contains a comprehensive C++ library for robot arm control, featuring inverse kinematics (IK) solving with reachable workspace support, motor control, and workspace generation capabilities. The library is designed for deployment on NVIDIA Jetson Orin Nano and provides efficient, real-time robot arm control.

## Table of Contents

- [Overview](#overview)
- [Headers](#headers)
- [Dependencies](#dependencies)
- [Build Instructions](#build-instructions)
- [Usage Examples](#usage-examples)
- [API Reference](#api-reference)
- [Performance Considerations](#performance-considerations)
- [Troubleshooting](#troubleshooting)

## Overview

The library consists of several key components:

- **Robot Model**: URDF-based robot representation with kinematics and collision detection
- **IK Solver with RWS**: Fast inverse kinematics using pre-computed reachable workspace data
- **Workspace Generator**: Tools for generating and sampling robot workspace data
- **Motor Controller**: Interface for controlling Moteus motor controllers
- **Workspace Types**: Common data structures for workspace representation

## Headers

### 1. `robot_model.h`

**Purpose**: Core robot model class that loads URDF/SRDF files and provides kinematics, collision detection, and utility functions.

**Key Features**:
- URDF/SRDF loading and parsing
- Forward kinematics computation
- Geometric Jacobian calculation
- Collision detection with ground clearance
- Joint limit management
- Pose conversion utilities (SE3 ↔ XYZQuat)

**Main Classes**:
- `RobotModel`: Main robot model class

**Key Methods**:
```cpp
// Constructor
RobotModel(const std::string& urdf_path, 
           const std::string& end_effector_name,
           const std::string& srdf_path);

// Forward kinematics
pinocchio::SE3 computeForwardKinematics(const JointConfig& q);
XYZQuat computeForwardKinematics(pinocchio::FrameIndex frame_id);

// Jacobian computation
Eigen::Matrix<double, 6, 6> computeGeometricJacobian(const JointConfig& q);
Eigen::Matrix<double, 7, 6> computeEndEffectorFullJacobian(const Eigen::VectorXd& q);

// Collision detection
bool checkCollision(const JointConfig& q);
bool isValidConfiguration(const JointConfig& q, double ground_threshold = 0.05);

// Utility functions
static XYZQuat homogeneousToXYZQuat(const pinocchio::SE3& homogeneous_transform);
static pinocchio::SE3 xyzQuatToHomogeneous(const XYZQuat& xyzquat);
```

### 2. `ik_solver_w_rws.hpp`

**Purpose**: High-performance inverse kinematics solver that uses pre-computed reachable workspace data for fast warm-start initialization.

**Key Features**:
- Two-stage IK solving: warm-start + iterative refinement
- Spatial indexing with k-d trees and voxel grids
- Quadratic programming optimization with line search
- Automatic workspace data loading
- Support for both XYZQuat and SE3 pose formats

**Main Classes**:
- `IKSolverWithRWS`: Main IK solver class
- `WorkspaceAdaptor`: Nanoflann adaptor for workspace points
- `KDTree3D`: 3D k-d tree for spatial indexing

**Key Methods**:
```cpp
// Constructor
IKSolverWithRWS(const std::string& workspace_dir, RobotModel& robot_model);

// Main solving methods
JointConfig solve(const Eigen::Vector<double, 7>& target_pose_xyzquat);
JointConfig solve(const pinocchio::SE3& target_pose);
JointConfig solve_ik(const Eigen::Vector<double, 7>& target_pose_xyzquat, 
                     const JointConfig* q_init = nullptr);

// Utility methods
bool isWorkspaceLoaded() const;
size_t getNumWorkspacePoints() const;
```

### 3. `reachable_workspace_generator.h`

**Purpose**: Tools for generating and sampling the reachable workspace of a robot arm.

**Key Features**:
- Random sampling within joint limits
- Collision detection and validation
- Duplicate configuration handling
- Multiple output formats (PLY, binary, voxel grid)
- Batch processing for efficiency

**Main Classes**:
- `ReachableWorkspaceGenerator`: Main workspace generator class
- `JointConfigHash`: Hash function for joint configurations
- `JointConfigEqual`: Equality function for joint configurations

**Key Methods**:
```cpp
// Constructor
ReachableWorkspaceGenerator(RobotModel& robot_model,
                           long unsigned int num_points = 100000,
                           int batch_size = 1000,
                           double ground_threshold = 0.05,
                           double joint_tolerance = 1e-3,
                           int max_attempts_per_sample = 10,
                           int max_distance = 50);

// Main generation methods
std::vector<WorkspacePoint> generateWorkspace();
void generateAndSaveWorkspace(const std::string& output_dir);

// Static utility methods
static void writeBinaryPLY(const std::string& filename, 
                          const std::vector<WorkspacePoint>& workspace_points);
static void writeAsciiPLY(const std::string& filename, 
                         const std::vector<WorkspacePoint>& workspace_points);
static void writeKDTreeData(const std::string& filename, 
                           const std::vector<WorkspacePoint>& workspace_points);
static void buildAndSaveVoxelGrid(const std::vector<WorkspacePoint>& workspace_points,
                                 const std::string& output_dir,
                                 const std::string& timestamp);
```

### 4. `motor_controller.h`

**Purpose**: Interface for controlling Moteus motor controllers with position control and torque management.

**Key Features**:
- Multi-motor position control
- Torque release functionality
- Threaded pose holding mechanism
- Configurable torque limits
- Thread-safe operations

**Main Classes**:
- `MotorController`: Main motor controller class

**Key Methods**:
```cpp
// Constructors
MotorController();
MotorController(const std::array<double, MOTORS>& max_torque);

// Control methods
void SetPosition(double pos, double accel_limit = 1.0, unsigned id = 0);
void SetArmPose(const double pos[MOTORS], const double accel_limit = 1.0);

// Torque control
void ReleaseMotorTorque(unsigned id);
void ReleaseArmTorque();

// Utility
static constexpr unsigned getMOTORS();
```

### 5. `workspace_types.h`

**Purpose**: Common data structures and type definitions used throughout the workspace system.

**Key Types and Structures**:
- `JointConfig`: 6-dimensional joint configuration vector
- `XYZQuat`: 7-dimensional pose vector [x, y, z, qx, qy, qz, qw]
- `WorkspacePoint`: Structure containing position, rotation, and joint config
- `NearestNeighbor`: Search result structure with index and distance
- `VoxelGrid`: Sparse voxel grid for spatial indexing

## Dependencies

### Required Libraries
- **Eigen3**: Linear algebra and geometry operations
- **Pinocchio**: Robot kinematics and dynamics
- **ProxSuite**: Quadratic programming solver
- **Nanoflann**: Efficient k-d tree implementation
- **Moteus**: Motor controller library
- **CycloneDDS-CXX**: DDS communication
- **Boost**: Random number generation and utilities
- **OpenMP**: Parallel processing (optional)

### System Requirements
- Ubuntu 22.04.5 LTS or compatible
- NVIDIA Jetson Orin Nano (target deployment platform)
- C++17 compiler
- CMake 3.16+

## Build Instructions

1. **Clone and navigate to the project**:
```bash
cd dds_bridge/cpp_plugins
```

2. **Create build directory**:
```bash
mkdir build && cd build
```

3. **Configure and build**:
```bash
cmake ..
make -j$(nproc)
```

4. **Install dependencies** (if not already installed):
```bash
# Install system dependencies
sudo apt update
sudo apt install libeigen3-dev libpinocchio-dev libboost-random-dev

# Install ProxSuite (if not available via package manager)
# Follow installation instructions from: https://github.com/Simple-Robotics/proxsuite
```

## Usage Examples

### 1. Basic Robot Model Setup

```cpp
#include "robot_model.h"
#include <iostream>

int main() {
    try {
        // Initialize robot model with URDF and SRDF files
        RobotModel robot_model(
            "/path/to/robot.urdf",
            "end_effector_frame",
            "/path/to/robot.srdf"
        );

        // Set initial joint configuration
        JointConfig q = RobotModel::NEUTRAL_JOINT_CONFIG;
        robot_model.setCurrentJointConfig(q);

        // Compute forward kinematics
        pinocchio::SE3 pose = robot_model.computeForwardKinematics(q);
        XYZQuat pose_xyzquat = RobotModel::homogeneousToXYZQuat(pose);

        std::cout << "End effector pose: " << pose_xyzquat.transpose() << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}
```

### 2. Workspace Generation

```cpp
#include "reachable_workspace_generator.h"
#include "robot_model.h"
#include <iostream>

int main() {
    try {
        // Initialize robot model
        RobotModel robot_model(
            "/path/to/robot.urdf",
            "end_effector_frame",
            "/path/to/robot.srdf"
        );

        // Create workspace generator with custom parameters
        ReachableWorkspaceGenerator generator(
            robot_model,    // Robot model reference
            50000,          // Number of points to attempt
            1000,           // Batch size
            0.05,           // Ground threshold
            1e-3,           // Joint tolerance
            10              // Max attempts per sample
        );

        // Generate and save workspace data
        generator.generateAndSaveWorkspace("generated_workspaces");

        std::cout << "Workspace generation completed!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}
```

### 3. IK Solving with Workspace Support

```cpp
#include "ik_solver_w_rws.hpp"
#include "robot_model.h"
#include <iostream>

int main() {
    try {
        // Initialize robot model
        RobotModel robot_model(
            "/path/to/robot.urdf",
            "end_effector_frame",
            "/path/to/robot.srdf"
        );

        // Initialize IK solver with workspace data
        IKSolverWithRWS ik_solver("generated_workspaces", robot_model);

        // Check if workspace data was loaded
        if (!ik_solver.isWorkspaceLoaded()) {
            std::cerr << "Failed to load workspace data!" << std::endl;
            return -1;
        }

        std::cout << "Loaded " << ik_solver.getNumWorkspacePoints() 
                  << " workspace points." << std::endl;

        // Define target pose [x, y, z, qx, qy, qz, qw]
        Eigen::Vector<double, 7> target_pose = {0.5, 0.3, 0.8, 0, 0, 0, 1};

        // Solve IK
        JointConfig solution = ik_solver.solve(target_pose);

        std::cout << "IK solution: " << solution.transpose() << std::endl;

        // Verify solution
        pinocchio::SE3 final_pose = robot_model.computeForwardKinematics(solution);
        XYZQuat final_pose_xyzquat = RobotModel::homogeneousToXYZQuat(final_pose);

        std::cout << "Final pose: " << final_pose_xyzquat.transpose() << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}
```

### 4. Motor Control

```cpp
#include "motor_controller.h"
#include <iostream>
#include <array>

int main() {
    try {
        // Initialize motor controller with torque limits
        std::array<double, 2> max_torque = {1.7, 1.7};
        MotorController mc(max_torque);

        std::cout << "MotorController initialized with " 
                  << MotorController::getMOTORS() << " motors" << std::endl;

        // Move individual motors
        mc.SetPosition(0.0, 1.0, 0);  // Motor 0 to 0 radians
        mc.SetPosition(0.0, 1.0, 1);  // Motor 1 to 0 radians

        // Move all motors to a pose
        double poses[2] = {3.14, -3.14};  // π and -π radians
        mc.SetArmPose(poses, 5.0);        // High acceleration

        // Hold pose for some time
        sleep(2);

        // Release torque
        mc.ReleaseArmTorque();

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}
```

### 5. Complete IK Control Loop

```cpp
#include "ik_solver_w_rws.hpp"
#include "motor_controller.h"
#include "robot_model.h"
#include <iostream>
#include <vector>

int main() {
    try {
        // Initialize components
        RobotModel robot_model("/path/to/robot.urdf", "end_effector_frame", "/path/to/robot.srdf");
        IKSolverWithRWS ik_solver("generated_workspaces", robot_model);
        MotorController mc;

        // Define target poses
        std::vector<Eigen::Vector<double, 7>> target_poses = {
            {0.5, 0.5, 0.5, 0, -0.7071068, 0, 0.7071068},
            {0.5, 0.5, 0.5, 0.5626401, 0, -0.5626401, -0.6056999},
            {0.5, 0.5, 0.5, 0, 0, 0, 1}
        };

        // Normalize quaternions
        for (auto& pose : target_poses) {
            Eigen::Quaterniond quat(pose[6], pose[3], pose[4], pose[5]);
            quat.normalize();
            pose[3] = quat.x(); pose[4] = quat.y(); 
            pose[5] = quat.z(); pose[6] = quat.w();
        }

        // Execute IK control loop
        for (size_t i = 0; i < target_poses.size(); ++i) {
            std::cout << "Moving to target " << (i + 1) << std::endl;

            // Solve IK
            JointConfig solution = ik_solver.solve(target_poses[i]);

            // Convert to degrees for motor control
            double poses_deg[6];
            for (int j = 0; j < 6; ++j) {
                poses_deg[j] = solution[j] * 180.0 / M_PI;
                if (poses_deg[j] > 180.0) poses_deg[j] -= 360.0;
            }

            // Move motors
            mc.SetArmPose(poses_deg, 2.0);
            sleep(1);  // Wait for movement to complete
        }

        // Release torque at the end
        mc.ReleaseArmTorque();

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}
```

## API Reference

### Data Types

```cpp
// Joint configuration (6 DOF robot)
using JointConfig = Eigen::Vector<double, 6>;

// Pose representation [x, y, z, qx, qy, qz, qw]
using XYZQuat = Eigen::Vector<double, 7>;

// Workspace point structure
struct WorkspacePoint {
    Eigen::Vector3d position;      // 3D position
    Eigen::Quaterniond rotation;   // Orientation
    JointConfig joint_config;      // Joint angles
};

// Nearest neighbor search result
struct NearestNeighbor {
    std::size_t index;             // Point index
    double distance;               // Distance to query
    const WorkspacePoint* point;   // Point pointer
};

// Voxel grid for spatial indexing
struct VoxelGrid {
    Eigen::Vector3d minB;          // Grid bounds
    double r;                      // Resolution
    Eigen::Vector3i dims;          // Dimensions
    int maxDistance;               // Search distance
    size_t numCells;               // Number of cells
    std::map<size_t, int> sparseNearestIdx; // Cell mapping
};
```

### Constants

```cpp
// Neutral joint configuration for UR10
static const JointConfig RobotModel::NEUTRAL_JOINT_CONFIG;

// Maximum IK iterations
static const int IKSolverWithRWS::MAX_IK_ITERATIONS = 1000;

// Number of motors in controller
static constexpr unsigned MotorController::MOTORS = 2;
```

## Performance Considerations

### IK Solver Performance
- **Warm-start initialization**: Uses spatial indexing for O(log n) neighbor search
- **Iterative refinement**: Quadratic programming with line search for robust convergence
- **Typical convergence**: 10-50 iterations for most poses
- **Memory usage**: ~100MB for 50K workspace points

### Workspace Generation Performance
- **Sampling efficiency**: 10-30% success rate depending on robot geometry
- **Batch processing**: Improves cache locality and reduces memory allocation
- **Duplicate detection**: O(1) hash-based lookup
- **Output formats**: Binary files for fast loading

### Motor Control Performance
- **Position control**: Non-blocking with acceleration limits
- **Thread safety**: Mutex-protected operations
- **Torque management**: Immediate release capability
- **Real-time operation**: Suitable for control loops

## Troubleshooting

### Common Issues

1. **Workspace data not found**:
   - Ensure workspace generation has been run
   - Check file paths and permissions
   - Verify binary file format compatibility

2. **IK solver not converging**:
   - Check if target pose is within reachable workspace
   - Verify joint limits in URDF
   - Try different initial configurations

3. **Motor communication errors**:
   - Check USB connections and permissions
   - Verify motor IDs and configuration
   - Ensure proper power supply

4. **Build errors**:
   - Install all required dependencies
   - Check CMake version compatibility
   - Verify compiler supports C++17

### Debug Tips

1. **Enable debug output**:
   ```bash
   cmake -DCMAKE_BUILD_TYPE=Debug ..
   ```

2. **Check workspace data integrity**:
   ```cpp
   if (!ik_solver.isWorkspaceLoaded()) {
       std::cerr << "Workspace data not loaded!" << std::endl;
   }
   ```

3. **Verify robot model loading**:
   ```cpp
   robot_model.printAllFrames();
   robot_model.findAndPrintFrame("end_effector_frame");
   ```

4. **Test individual components**:
   - Run `test_ik_solver_w_rws` for IK testing
   - Run `mc_tester` for motor control testing
   - Run `example_workspace_generator` for workspace generation

### Performance Optimization

1. **Workspace generation**:
   - Increase batch size for better cache locality
   - Adjust ground threshold based on robot mounting
   - Use appropriate joint tolerance for your application

2. **IK solving**:
   - Pre-generate workspace data for your specific robot
   - Use voxel grid for faster neighbor search
   - Tune QP solver parameters if needed

3. **Motor control**:
   - Use appropriate acceleration limits
   - Implement proper error handling
   - Consider using hold threads for continuous operation

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review the example code in the `src/` directory
3. Examine the test files for usage patterns
4. Check build logs for dependency issues 