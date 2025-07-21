/**
 * @file ik_solver_w_rws.hpp
 * @brief Inverse Kinematics Solver with Reachable Workspace Support
 *
 * This header file defines the IKSolverWithRWS class, which provides efficient
 * inverse kinematics solving capabilities using pre-computed reachable
 * workspace data. The solver uses a two-stage approach:
 * 1. Fast nearest neighbor search to find good initial joint configurations
 * 2. Iterative optimization using quadratic programming to refine the solution
 *
 * The workspace data is stored in binary files containing:
 * - K-d tree data: sampled workspace points with positions, orientations, and
 * joint configs
 * - Voxel grid data: spatial indexing structure for fast neighbor search
 *
 * @author Shantanu Ghodgaonkar
 * @date 2025-07-21
 * @version 1.0
 */

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

/**
 * @brief Nanoflann adaptor for WorkspacePoint data structure
 *
 * This adaptor allows the nanoflann library to work with our custom
 * WorkspacePoint data structure. It provides the necessary interface functions
 * for building and querying a k-d tree over workspace points.
 *
 * The adaptor maps workspace points to 3D coordinates (x, y, z) for spatial
 * indexing, enabling fast nearest neighbor searches in the workspace.
 */
struct WorkspaceAdaptor {
    using PointCloudT = std::vector<WorkspacePoint>;
    const PointCloudT &pts; ///< Reference to the workspace points vector

    /**
     * @brief Constructor
     * @param points Reference to the workspace points vector
     */
    WorkspaceAdaptor(const PointCloudT &points) : pts(points) {}

    /**
     * @brief Get the total number of points in the workspace
     * @return Number of workspace points
     *
     * This function is required by nanoflann to know the size of the point
     * cloud.
     */
    inline std::size_t kdtree_get_point_count() const noexcept {
        return pts.size();
    }

    /**
     * @brief Get the d-th coordinate of point i
     * @param i Index of the point
     * @param d Dimension index (0=x, 1=y, 2=z)
     * @return Coordinate value at the specified dimension
     *
     * This function maps workspace points to 3D coordinates for k-d tree
     * indexing. Only position coordinates are used for spatial indexing;
     * orientation and joint configuration are not considered in the spatial
     * search.
     */
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

    /**
     * @brief Optional bounding box routine for nanoflann
     * @param bbox Bounding box structure (unused)
     * @return false (bounding box not implemented)
     *
     * This function is required by nanoflann but not used in our
     * implementation. Returning false indicates that bounding box optimization
     * is not available.
     */
    template <class BBOX> bool kdtree_get_bbox(BBOX &) const noexcept {
        return false;
    }
};

/**
 * @brief 3D k-d tree type for workspace point indexing
 *
 * This typedef creates a convenient alias for a 3-dimensional k-d tree that
 * uses L2 (Euclidean) distance metric for nearest neighbor searches.
 * The tree is built over workspace points using their position coordinates.
 */
using KDTree3D = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, WorkspaceAdaptor>, WorkspaceAdaptor,
    3 /* dimension */
    >;

/**
 * @brief Inverse Kinematics Solver with Reachable Workspace Support
 *
 * This class provides IK solving capabilities using pre-computed reachable
 * workspace data for efficient warm-start initialization. The solver implements
 * a sophisticated two-stage approach:
 *
 * 1. **Warm-start Initialization**: Uses nearest neighbor search in
 * pre-computed workspace data to find good initial joint configurations close
 * to the target pose. This significantly improves convergence speed and success
 * rate.
 *
 * 2. **Iterative Refinement**: Uses quadratic programming with line search to
 *    refine the initial configuration and achieve precise pose matching.
 *
 * The workspace data is loaded from binary files containing:
 * - K-d tree data: sampled workspace points with positions, orientations, and
 * joint configs
 * - Voxel grid data: spatial indexing structure for fast neighbor search
 *
 * **Control Flow Overview:**
 * 1. Constructor loads workspace data from files
 * 2. solve() method performs nearest neighbor search for warm-start
 * 3. solve_ik() method performs iterative optimization using QP
 * 4. Line search ensures convergence and prevents overshooting
 *
 * **Key Features:**
 * - Fast warm-start using spatial indexing
 * - Robust convergence with line search
 * - Joint limit enforcement
 * - Comprehensive error handling
 *
 * @see RobotModel
 * @see WorkspacePoint
 * @see VoxelGrid
 */
class IKSolverWithRWS {
  public:
    // Constants
    /**
     * @brief Maximum number of iterations for IK optimization
     *
     * This constant limits the number of iterations in the iterative IK solver
     * to prevent infinite loops and ensure reasonable computation time.
     * The solver typically converges in much fewer iterations (10-50).
     */
    static const int MAX_IK_ITERATIONS = 1000;

    /**
     * @brief Constructor
     * @param workspace_dir Directory containing generated workspace data files
     * @param robot_model Reference to the robot model for IK solving
     *
     * **Constructor Flow:**
     * 1. Store workspace directory and robot model references
     * 2. Find and load the latest k-d tree data file
     * 3. Find and load the latest voxel grid data file
     * 4. Verify data integrity and print loading statistics
     * 5. Initialize voxel grid loaded flag
     *
     * **Error Handling:**
     * - Throws runtime_error if no workspace data files are found
     * - Throws runtime_error if data loading fails
     * - Provides detailed error messages for debugging
     *
     * @throws std::runtime_error if workspace data cannot be loaded
     */
    IKSolverWithRWS(const std::string &workspace_dir, RobotModel &robot_model);

    /**
     * @brief Solve IK for a target pose given as XYZQuat vector
     * @param target_pose_xyzquat Target pose as [x, y, z, qx, qy, qz, qw]
     * @return Joint configuration that achieves the target pose
     *
     * **Method Flow:**
     * 1. Extract translation (xyz) and rotation (quaternion) from input
     * 2. Normalize quaternion to ensure unit length
     * 3. Perform nearest neighbor search using voxel grid for efficiency
     * 4. Select best initial configuration based on rotation error
     * 5. Normalize joint angles to [-π, π] range
     * 6. Call solve_ik() with warm-start configuration
     *
     * **Neighbor Selection Logic:**
     * - Uses geodesic distance for rotation error calculation
     * - Selects configuration with minimum rotation error
     * - Falls back to current joint config if no neighbors found
     *
     * @see solve_ik()
     */
    JointConfig solve(const Eigen::Vector<double, 7> &target_pose_xyzquat);

    /**
     * @brief Solve IK for a target pose given as SE3 transform
     * @param target_pose Target pose as SE3 transform
     * @return Joint configuration that achieves the target pose
     *
     * **Method Flow:**
     * 1. Convert SE3 transform to XYZQuat format using RobotModel utility
     * 2. Call the XYZQuat version of solve()
     *
     * This method provides a convenient interface for users working with
     * Pinocchio SE3 transforms.
     *
     * @see solve(const Eigen::Vector<double, 7>&)
     */
    JointConfig solve(const pinocchio::SE3 &target_pose);

    /**
     * @brief Check if workspace data was successfully loaded
     * @return true if workspace data is available, false otherwise
     *
     * This method can be used to verify that the solver was properly
     * initialized with workspace data before attempting IK solving.
     */
    bool isWorkspaceLoaded() const { return !workspace_points_.empty(); }

    /**
     * @brief Get the number of loaded workspace points
     * @return Number of workspace points
     *
     * This method provides information about the size of the loaded
     * workspace data, which can be useful for debugging and performance
     * analysis.
     */
    size_t getNumWorkspacePoints() const { return workspace_points_.size(); }

    /**
     * @brief Solve IK with explicit initial configuration
     * @param target_pose_xyzquat Target pose as [x, y, z, qx, qy, qz, qw]
     * @param q_init Optional initial joint configuration (nullptr for current
     * config)
     * @return Joint configuration that achieves the target pose
     *
     * This method allows users to provide their own initial configuration
     * instead of using the automatic warm-start selection.
     *
     * @see solve_ik(const pinocchio::SE3&, const JointConfig*)
     */
    JointConfig solve_ik(const Eigen::Vector<double, 7> &target_pose_xyzquat,
                         const JointConfig *q_init = nullptr);

  private:
    /**
     * @brief Core IK solving method using iterative optimization
     * @param target_pose Target pose as SE3 transform
     * @param q_init Optional initial joint configuration (nullptr for current
     * config)
     * @return Joint configuration that achieves the target pose
     *
     * **Core IK Algorithm Flow:**
     * 1. **Initialization**: Set up QP solver with joint velocity bounds
     * 2. **Iterative Loop** (up to MAX_IK_ITERATIONS):
     *    a. Compute forward kinematics and geometric Jacobian
     *    b. Calculate pose error using SE3 logarithm
     *    c. Check convergence (error < 1e-9)
     *    d. Build QP problem: H = J^T J + λI, g = -J^T err
     *    e. Update joint velocity bounds based on joint limits
     *    f. Solve QP to get joint velocity update
     *    g. Perform line search to find optimal step size
     *    h. Update joint configuration
     * 3. **Finalization**: Set robot model configuration and return result
     *
     * **Key Algorithm Features:**
     * - Uses quadratic programming for robust optimization
     * - Implements line search to ensure convergence
     * - Enforces joint limits through velocity bounds
     * - Uses regularization (λ) to handle singular configurations
     * - Monitors convergence using SE3 error metric
     *
     * **QP Problem Formulation:**
     * - Objective: minimize ||J*dq - err||² + λ||dq||²
     * - Constraints: joint velocity bounds and joint limit constraints
     * - Variables: joint velocity updates (dq)
     *
     * @see solve_ik(const Eigen::Vector<double, 7>&, const JointConfig*)
     */
    JointConfig solve_ik(const pinocchio::SE3 &target_pose,
                         const JointConfig *q_init = nullptr);

    // Workspace data loading functions
    /**
     * @brief Find the latest k-d tree data file in the workspace directory
     * @param directory Directory to search for k-d tree files
     * @return Path to the latest k-d tree file, or empty string if not found
     *
     * **Search Logic:**
     * - Looks for files starting with "kdtree_data_" and ending with ".bin"
     * - Compares file modification times to find the most recent
     * - Returns empty string if no matching files found
     *
     * This method enables automatic loading of the most recent workspace data
     * without manual file specification.
     */
    std::string findLatestKDTreeFile(const std::string &directory);

    /**
     * @brief Find the latest voxel grid data file in the workspace directory
     * @param directory Directory to search for voxel grid files
     * @return Path to the latest voxel grid file, or empty string if not found
     *
     * **Search Logic:**
     * - Looks for files starting with "workspace_grid_sparse_" and ending with
     * ".bin"
     * - Compares file modification times to find the most recent
     * - Returns empty string if no matching files found
     *
     * The voxel grid provides spatial indexing for efficient nearest neighbor
     * search.
     */
    std::string findLatestVoxelGridFile(const std::string &directory);

    /**
     * @brief Load workspace points from k-d tree binary file
     * @param filename Path to the k-d tree binary file
     * @return Vector of loaded workspace points
     *
     * **File Format:**
     * - Header: number of points (size_t)
     * - For each point:
     *   - Position: 3 doubles (x, y, z)
     *   - Quaternion: 4 doubles (w, x, y, z)
     *   - Joint config: 6 doubles
     *
     * **Loading Process:**
     * 1. Open binary file and read header
     * 2. Pre-allocate vector for efficiency
     * 3. Read each point's data sequentially
     * 4. Normalize quaternions to ensure unit length
     * 5. Create WorkspacePoint objects
     * 6. Handle read errors gracefully
     *
     * @throws std::runtime_error if file cannot be opened or read
     */
    std::vector<WorkspacePoint> loadKDTreeData(const std::string &filename);

    /**
     * @brief Load voxel grid data from binary file
     * @param filename Path to the voxel grid binary file
     * @return Loaded voxel grid structure
     *
     * **File Format:**
     * - Header: minB(3×double), r(double), dims(3×int), maxDistance(int),
     * numCells(size_t)
     * - Data: (cell_index, nearest_point_index) pairs
     *
     * **Loading Process:**
     * 1. Open binary file and read header information
     * 2. Read sparse data as (cell_index, nearest_point_index) pairs
     * 3. Store in sparseNearestIdx map for efficient lookup
     * 4. Handle read errors gracefully
     *
     * The voxel grid provides O(1) spatial indexing for nearest neighbor
     * search.
     *
     * @throws std::runtime_error if file cannot be opened or read
     */
    VoxelGrid loadVoxelGridData(const std::string &filename);

    // Nearest neighbor search functions
    /**
     * @brief Find k nearest neighbors using voxel grid spatial indexing
     * @param workspace_points Vector of workspace points to search
     * @param grid Voxel grid for spatial indexing
     * @param target_xyz Target position in 3D space
     * @param k Number of nearest neighbors to find (default: 10)
     * @return Vector of nearest neighbors with distances
     *
     * **Algorithm Flow:**
     * 1. **Grid Indexing**: Convert target position to voxel grid coordinates
     * 2. **Onion Ring Search**: Search in expanding rings around target cell
     *    - Start with center cell
     *    - Expand to 6-connected neighbors
     *    - Continue until k neighbors found or search exhausted
     * 3. **Distance Calculation**: Compute Euclidean distances to all
     * candidates
     * 4. **Selection**: Return k closest neighbors by distance
     *
     * **Performance Characteristics:**
     * - O(1) spatial indexing using voxel grid
     * - O(k) distance calculations
     * - Much faster than brute force search for large datasets
     *
     * **Search Strategy:**
     * - Uses "onion ring" expansion to find nearby points
     * - Ensures uniqueness of selected points
     * - Prioritizes spatial proximity over exact distance
     *
     * @see findKNearestNeighbors()
     */
    std::vector<NearestNeighbor> findKNearestNeighborsVoxel(
        const std::vector<WorkspacePoint> &workspace_points,
        const VoxelGrid &grid, const Eigen::Vector3d &target_xyz, int k = 10);

    /**
     * @brief Find k nearest neighbors using k-d tree spatial indexing
     * @param workspace_points Vector of workspace points to search
     * @param target_xyz Target position in 3D space
     * @param k Number of nearest neighbors to find (default: 10)
     * @return Vector of nearest neighbors with distances
     *
     * **Algorithm Flow:**
     * 1. **Tree Construction**: Build k-d tree from workspace points
     * 2. **Nearest Neighbor Search**: Use nanoflann library for efficient
     * search
     * 3. **Distance Conversion**: Convert squared distances to actual distances
     * 4. **Result Formatting**: Convert to NearestNeighbor format
     *
     * **Performance Characteristics:**
     * - O(log n) search time for balanced trees
     * - O(n log n) tree construction time
     * - More accurate than voxel grid method but potentially slower
     *
     * This method provides a fallback when voxel grid is not available or
     * when more precise nearest neighbor search is required.
     *
     * @see findKNearestNeighborsVoxel()
     */
    std::vector<NearestNeighbor>
    findKNearestNeighbors(const std::vector<WorkspacePoint> &workspace_points,
                          const Eigen::Vector3d &target_xyz, int k = 10);

    // Member variables
    std::string workspace_dir_; ///< Directory containing workspace data files
    RobotModel &robot_model_;   ///< Reference to robot model for IK solving
    std::vector<WorkspacePoint>
        workspace_points_; ///< Loaded workspace points for nearest neighbor
                           ///< search
    VoxelGrid
        voxel_grid_; ///< Spatial indexing structure for fast neighbor search
    bool voxel_grid_loaded_; ///< Flag indicating if voxel grid was successfully
                             ///< loaded
};

#endif // IK_SOLVER_W_RWS_HPP