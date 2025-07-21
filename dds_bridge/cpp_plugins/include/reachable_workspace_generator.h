#ifndef REACHABLE_WORKSPACE_GENERATOR_H
#define REACHABLE_WORKSPACE_GENERATOR_H

#include "robot_model.h"
#include "workspace_types.h"
#include <Eigen/Geometry>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

// Hash function for JointConfig to use with unordered_set (O(1) lookup)
struct JointConfigHash {
    std::size_t operator()(const JointConfig &config) const;
};

// Equality function for JointConfig to use with unordered_set
struct JointConfigEqual {
    bool operator()(const JointConfig &lhs, const JointConfig &rhs) const;
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
                                int max_attempts_per_sample = 10,
                                int max_distance = 2);

    // Destructor
    ~ReachableWorkspaceGenerator();

    // Main generation function
    std::vector<WorkspacePoint> generateWorkspace();

    // Generate and save all formats with timestamped filenames
    void generateAndSaveWorkspace(const std::string &output_dir);

    // Static utility functions (can be called without class instance)
    static void
    writeBinaryPLY(const std::string &filename,
                   const std::vector<WorkspacePoint> &workspace_points);
    static void
    writeAsciiPLY(const std::string &filename,
                  const std::vector<WorkspacePoint> &workspace_points);
    static void
    writeKDTreeData(const std::string &filename,
                    const std::vector<WorkspacePoint> &workspace_points);
    // Static version with timestamp parameter
    static void
    buildAndSaveVoxelGrid(const std::vector<WorkspacePoint> &workspace_points,
                          const std::string &output_dir,
                          const std::string &timestamp);

    static std::string getTimestampedFilename(const std::string &base_dir,
                                              const std::string &prefix,
                                              const std::string &extension);

  private:
    RobotModel &robot_model_; // Reference to external RobotModel
    std::mt19937 random_generator_;

    // Workspace generation parameters
    long unsigned int num_points_;
    int batch_size_;
    double ground_threshold_;
    double joint_tolerance_;
    int max_attempts_per_sample_;
    int max_distance_;

    // Helper functions
    bool processSampleFast(const JointConfig &q, WorkspacePoint &result);

    std::vector<WorkspacePoint> processBatch(
        std::mt19937 &gen,
        std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
            &existing_configs,
        const int batch_size);
};

#endif // REACHABLE_WORKSPACE_GENERATOR_H