#ifndef IK_SOLVER_W_RWS_HPP
#define IK_SOLVER_W_RWS_HPP

#include "robot_model.h"
#include "workspace_types.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nanoflann.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <vector>

// Nanoflann adaptor for WorkspacePoint
struct WorkspaceAdaptor {
    using PointCloudT = std::vector<WorkspacePoint>;
    const PointCloudT &pts;

    WorkspaceAdaptor(const PointCloudT &points) : pts(points) {}

    // Must return the number of data points
    inline std::size_t kdtree_get_point_count() const noexcept {
        return pts.size();
    }

    // Returns the 'd'-th coordinate (0=x,1=y,2=z) of point #i
    inline double kdtree_get_pt(const std::size_t i,
                                const std::size_t d) const noexcept {
        switch (d) {
        case 0:
            return pts[i].position.x();
        case 1:
            return pts[i].position.y();
        case 2:
            return pts[i].position.z();
        default:
            return 0.0;
        }
    }

    // Optional bounding-box routine: not needed for most use cases
    template <class BBOX> bool kdtree_get_bbox(BBOX &) const noexcept {
        return false;
    }
};

// Define a convenient alias for a 3-D L2 tree
using KDTree3D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, WorkspaceAdaptor>, WorkspaceAdaptor,
    3 /* dimension */
    >;

/**
 * @brief Inverse Kinematics Solver with Reachable Workspace Support
 *
 * This class provides IK solving capabilities using pre-computed reachable
 * workspace data for efficient warm-start initialization. It loads workspace
 * data from binary files and uses nearest neighbor search to find good initial
 * configurations.
 */
class IKSolverWithRWS {
  public:
    // Constants
    static const int MAX_IK_ITERATIONS =
        1000; // Maximum iterations for IK optimization

    /**
     * @brief Constructor
     * @param workspace_dir Directory containing generated workspace data files
     * @param robot_model Reference to the robot model for IK solving
     */
    IKSolverWithRWS(const std::string &workspace_dir, RobotModel &robot_model);

    /**
     * @brief Solve IK for a target pose given as XYZQuat vector
     * @param target_pose_xyzquat Target pose as [x, y, z, qx, qy, qz, qw]
     * @return Joint configuration that achieves the target pose
     */
    JointConfig solve(const Eigen::Vector<double, 7> &target_pose_xyzquat);

    /**
     * @brief Solve IK for a target pose given as SE3 transform
     * @param target_pose Target pose as SE3 transform
     * @return Joint configuration that achieves the target pose
     */
    JointConfig solve(const pinocchio::SE3 &target_pose);

    /**
     * @brief Check if workspace data was successfully loaded
     * @return true if workspace data is available, false otherwise
     */
    bool isWorkspaceLoaded() const { return !workspace_points_.empty(); }

    /**
     * @brief Get the number of loaded workspace points
     * @return Number of workspace points
     */
    size_t getNumWorkspacePoints() const { return workspace_points_.size(); }

    JointConfig solve_ik(const Eigen::Vector<double, 7> &target_pose_xyzquat,
                         const JointConfig *q_init = nullptr);

  private:
    // Core IK solving functions
    JointConfig solve_ik(const pinocchio::SE3 &target_pose,
                         const JointConfig *q_init = nullptr);

    // JointConfig solve_ik(const Eigen::Vector<double, 7> &target_pose_xyzquat,
    //                      const JointConfig *q_init = nullptr);

    // Workspace data loading functions
    std::string findLatestKDTreeFile(const std::string &directory);
    std::string findLatestVoxelGridFile(const std::string &directory);
    std::vector<WorkspacePoint> loadKDTreeData(const std::string &filename);
    VoxelGrid loadVoxelGridData(const std::string &filename);

    // Nearest neighbor search functions
    std::vector<NearestNeighbor> findKNearestNeighborsVoxel(
        const std::vector<WorkspacePoint> &workspace_points,
        const VoxelGrid &grid, const Eigen::Vector3d &target_xyz, int k = 10);

    std::vector<NearestNeighbor>
    findKNearestNeighbors(const std::vector<WorkspacePoint> &workspace_points,
                          const Eigen::Vector3d &target_xyz, int k = 10);

    // Member variables
    std::string workspace_dir_;
    RobotModel &robot_model_;
    std::vector<WorkspacePoint> workspace_points_;
    VoxelGrid voxel_grid_;
    bool voxel_grid_loaded_;
};

#endif // IK_SOLVER_W_RWS_HPP