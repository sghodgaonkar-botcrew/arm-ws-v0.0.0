#ifndef REACHABLE_WORKSPACE_GENERATOR_H
#define REACHABLE_WORKSPACE_GENERATOR_H

#include "robot_model.h"
#include <Eigen/Geometry>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

// Type definitions (reuse from ik_model.h)
using JointConfig = Eigen::Vector<double, 6>;

// Hash function for JointConfig to use with unordered_set (O(1) lookup)
struct JointConfigHash {
    std::size_t operator()(const JointConfig &config) const;
};

// Equality function for JointConfig to use with unordered_set
struct JointConfigEqual {
    bool operator()(const JointConfig &lhs, const JointConfig &rhs) const;
};

// Structure to hold workspace point data with position, rotation, and joint
// config
struct WorkspacePoint {
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    JointConfig joint_config;

    WorkspacePoint(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot,
                   const JointConfig &config);
};

// Main workspace generator class
class ReachableWorkspaceGenerator {
  public:
    // Constructor - takes IKModel by reference and parameters directly
    ReachableWorkspaceGenerator(RobotModel &robot_model,
                                long unsigned int num_points = 100000,
                                int batch_size = 1000,
                                double ground_threshold = 0.05,
                                double joint_tolerance = 1e-3,
                                int max_attempts_per_sample = 10);

    // Destructor
    ~ReachableWorkspaceGenerator();

    // Main generation function
    std::vector<WorkspacePoint> generateWorkspace();

    // Save functions
    void saveToPLY(const std::string &filename,
                   const std::vector<WorkspacePoint> &workspace_points);
    void saveToKDTree(const std::string &filename,
                      const std::vector<WorkspacePoint> &workspace_points);
    void saveToVoxelGrid(const std::string &output_dir,
                         const std::vector<WorkspacePoint> &workspace_points);

    // Generate and save all formats with timestamped filenames
    void generateAndSaveWorkspace(const std::string &output_dir);

  private:
    RobotModel &robot_model_; // Reference to external RobotModel
    std::mt19937 random_generator_;

    // Workspace generation parameters
    long unsigned int num_points_;
    int batch_size_;
    double ground_threshold_;
    double joint_tolerance_;
    int max_attempts_per_sample_;

    // Helper functions
    bool processSampleFast(const JointConfig &q, WorkspacePoint &result);

    std::vector<WorkspacePoint> processBatch(
        std::mt19937 &gen,
        std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
            &existing_configs,
        const int batch_size);

    std::string getTimestampedFilename(const std::string &base_dir,
                                       const std::string &prefix,
                                       const std::string &extension);
};

// Standalone helper functions (for backward compatibility)
namespace workspace_utils {

// Check if two joint configurations are approximately equal
bool isJointConfigEqual(const JointConfig &lhs, const JointConfig &rhs,
                        double tolerance = 1e-3);

// Write binary PLY file
void writeBinaryPLY(const std::string &filename,
                    const std::vector<WorkspacePoint> &workspace_points);

// Write k-d tree data to binary file
void writeKDTreeData(const std::string &filename,
                     const std::vector<WorkspacePoint> &workspace_points);

// Build and save sparse 3D voxel grid
void buildAndSaveVoxelGrid(const std::vector<WorkspacePoint> &workspace_points,
                           const std::string &output_dir);

// Get timestamped filename
std::string getTimestampedFilename(const std::string &base_dir,
                                   const std::string &prefix,
                                   const std::string &extension);
} // namespace workspace_utils

#endif // REACHABLE_WORKSPACE_GENERATOR_H