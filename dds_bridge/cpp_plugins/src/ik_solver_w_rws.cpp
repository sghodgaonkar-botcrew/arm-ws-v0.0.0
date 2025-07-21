/**
 * @file ik_solver_w_rws.cpp
 * @brief Implementation of Inverse Kinematics Solver with Reachable Workspace
 * Support
 *
 * This file implements the IKSolverWithRWS class, providing efficient inverse
 * kinematics solving using pre-computed workspace data. The implementation
 * includes sophisticated nearest neighbor search algorithms and iterative
 * optimization using quadratic programming.
 *
 * @author Shantanu Ghodgaonkar
 * @date 2025-07-21
 * @version 1.0
 */

#include "ik_solver_w_rws.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <unordered_set>

/**
 * @brief Constructor implementation
 *
 * **Constructor Flow:**
 * 1. Initialize member variables and set voxel grid loaded flag to false
 * 2. Find and load the latest k-d tree data file
 * 3. Verify k-d tree data was loaded successfully
 * 4. Find and load the latest voxel grid data file
 * 5. Verify voxel grid data was loaded successfully
 * 6. Set voxel grid loaded flag to true
 * 7. Print loading statistics for verification
 *
 * **Error Handling:**
 * - Throws runtime_error if no k-d tree files found
 * - Throws runtime_error if k-d tree data loading fails
 * - Throws runtime_error if no voxel grid files found
 * - Throws runtime_error if voxel grid data loading fails
 *
 * **Data Loading Strategy:**
 * - Automatically finds the most recent data files by modification time
 * - Provides detailed error messages for debugging
 * - Ensures data integrity before proceeding
 */
IKSolverWithRWS::IKSolverWithRWS(const std::string &workspace_dir,
                                 RobotModel &robot_model)
    : workspace_dir_(workspace_dir), robot_model_(robot_model),
      voxel_grid_loaded_(false) {

    // Load the latest k-d tree data from workspace directory
    std::string latest_kdtree_file = findLatestKDTreeFile(workspace_dir_);

    if (latest_kdtree_file.empty()) {
        std::cerr << "No k-d tree files found in " << workspace_dir_
                  << std::endl;
        std::cerr << "Please run reachable_workspace first to generate "
                     "workspace data."
                  << std::endl;
        throw std::runtime_error("No workspace data found in directory: " +
                                 workspace_dir_);
    }

    std::cout << "Loading latest k-d tree file: " << latest_kdtree_file
              << std::endl;
    workspace_points_ = loadKDTreeData(latest_kdtree_file);

    if (workspace_points_.empty()) {
        std::cerr << "Failed to load workspace points from k-d tree file."
                  << std::endl;
        throw std::runtime_error("Failed to load workspace data from: " +
                                 latest_kdtree_file);
    }

    std::cout << "Successfully loaded " << workspace_points_.size()
              << " workspace points." << std::endl;

    // Load the latest voxel grid data
    std::string latest_voxel_grid_file =
        findLatestVoxelGridFile(workspace_dir_);

    if (latest_voxel_grid_file.empty()) {
        std::cerr << "No voxel grid files found in " << workspace_dir_
                  << std::endl;
        std::cerr << "Please run reachable_workspace first to generate "
                     "workspace data including voxel grid."
                  << std::endl;
        throw std::runtime_error("No voxel grid data found in directory: " +
                                 workspace_dir_);
    }

    std::cout << "Loading latest voxel grid file: " << latest_voxel_grid_file
              << std::endl;
    voxel_grid_ = loadVoxelGridData(latest_voxel_grid_file);

    if (voxel_grid_.numCells == 0) {
        std::cerr << "Failed to load sparse voxel grid data." << std::endl;
        throw std::runtime_error("Failed to load voxel grid data from: " +
                                 latest_voxel_grid_file);
    }

    std::cout << "Successfully loaded sparse voxel grid with "
              << voxel_grid_.numCells << " cells." << std::endl;
    voxel_grid_loaded_ = true;

    // Print first few points for verification
    // std::cout << "\nFirst 3 workspace points:" << std::endl;
    // for (int i = 0; i < std::min(3,
    // static_cast<int>(workspace_points_.size()));
    //      ++i) {
    //     const auto &point = workspace_points_[i];
    //     std::cout << "Point " << i << ": Position=["
    //               << point.position.transpose() << "], Joints=["
    //               << point.joint_config.transpose() << "]" << std::endl;
    // }
    // std::cout << std::endl;
}

/**
 * @brief Solve IK for target pose given as XYZQuat vector
 *
 * **Method Flow:**
 * 1. **Input Processing**: Extract translation and rotation from XYZQuat input
 * 2. **Quaternion Normalization**: Ensure unit quaternion for numerical
 * stability
 * 3. **Nearest Neighbor Search**: Use voxel grid for efficient spatial search
 * 4. **Warm-start Selection**: Choose best initial configuration based on
 * rotation error
 * 5. **Joint Angle Normalization**: Wrap angles to [-π, π] range
 * 6. **IK Solving**: Call core IK solver with warm-start configuration
 *
 * **Neighbor Selection Logic:**
 * - Uses geodesic distance (2*acos(|q1·q2|)) for rotation error
 * - Selects configuration with minimum rotation error
 * - Falls back to current joint config if no neighbors found
 * - Provides detailed logging of selection process
 *
 * **Performance Optimizations:**
 * - Uses voxel grid for O(1) spatial indexing
 * - Searches for 20 neighbors to ensure good coverage
 * - Normalizes quaternions to prevent numerical issues
 */
JointConfig
IKSolverWithRWS::solve(const Eigen::Vector<double, 7> &target_pose_xyzquat) {
    // Split target pose into translation and quaternion components
    Eigen::Vector3d xyz =
        target_pose_xyzquat.head<3>(); // Extract translation [x, y, z]
    // User provides [qx, qy, qz, qw], Eigen::Quaterniond expects [qw, qx, qy,
    // qz]
    Eigen::Quaterniond quat(target_pose_xyzquat[6], target_pose_xyzquat[3],
                            target_pose_xyzquat[4], target_pose_xyzquat[5]);
    quat.normalize(); // Ensure unit quaternion

    // Perform nearest neighbor search using voxel grid for efficiency
    std::vector<NearestNeighbor> nearest_neighbors_voxel =
        findKNearestNeighborsVoxel(workspace_points_, voxel_grid_, xyz, 20);

    // Pick the best initial configuration for IK warm-start
    JointConfig q_init = RobotModel::NEUTRAL_JOINT_CONFIG;

    if (!nearest_neighbors_voxel.empty()) {
        // Use voxel grid results for warm-start if available
        std::cout
            << "Selecting best initial configuration from voxel grid results..."
            << std::endl;

        // Find the best neighbor by full pose error (position + rotation)
        size_t bestIdx = nearest_neighbors_voxel[0].index;
        double bestPoseErr = std::numeric_limits<double>::infinity();

        // Iterate through all neighbors to find the one with minimum rotation
        // error
        for (const auto &neighbor : nearest_neighbors_voxel) {
            const auto &wp = workspace_points_[neighbor.index];

            // Calculate geodesic rotation error between target and neighbor
            // quaternions This measures the angular distance between two
            // rotations
            double ang = 2.0 * std::acos(std::abs(wp.rotation.dot(quat)));

            if (ang < bestPoseErr) {
                bestPoseErr = ang;
                bestIdx = neighbor.index;
            }
        }

        // Use the best neighbor's joint configuration as warm-start
        q_init = workspace_points_[bestIdx].joint_config;
        std::cout << "Selected initial configuration with rotation error: "
                  << bestPoseErr << " radians" << std::endl;
        std::cout << "Initial joint config: [" << q_init.transpose() << "]"
                  << std::endl;

        // Commented out code shows alternative selection strategies
        // that were considered but not used in the final implementation
    } else {
        std::cout << "No neighbors found, using current joint configuration as "
                     "warm-start."
                  << std::endl;
        q_init = robot_model_.getCurrentJointConfig();
    }

    // Normalize joint angles to [-π, π] range to prevent numerical issues
    for (uint8_t i = 0; i < robot_model_.getNumJoints(); i++) {
        if (q_init[i] > M_PI) {
            q_init[i] -= 2 * M_PI;
        } else if (q_init[i] < -M_PI) {
            q_init[i] += 2 * M_PI;
        }
    }

    // Solve IK with warm-start configuration
    return solve_ik(target_pose_xyzquat, &q_init);
}

/**
 * @brief Solve IK for target pose given as SE3 transform
 *
 * **Method Flow:**
 * 1. Convert SE3 transform to XYZQuat format using RobotModel utility
 * 2. Call the XYZQuat version of solve()
 *
 * This method provides a convenient interface for users working with
 * Pinocchio SE3 transforms, automatically handling the conversion.
 */
JointConfig IKSolverWithRWS::solve(const pinocchio::SE3 &target_pose) {
    // Convert SE3 to XYZQuat format
    XYZQuat target_xyzquat = RobotModel::homogeneousToXYZQuat(target_pose);
    return solve(target_xyzquat);
}

/**
 * @brief Core IK solving method using iterative optimization
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
 * **Line Search Parameters:**
 * - Initial step size: 1.0
 * - Reduction factor: 0.5
 * - Minimum step size: 1e-6
 * - Armijo condition: err_new < err_old
 */
JointConfig IKSolverWithRWS::solve_ik(const pinocchio::SE3 &target_pose,
                                      const JointConfig *q_init) {
    const int nx = robot_model_.getNumVelocityVariables();
    const int n_eq = 0;
    const int n_in = 0;

    // Initialize joint configuration
    Eigen::VectorXd q;
    if (q_init == nullptr)
        q = robot_model_.getCurrentJointConfig();
    else
        q = *q_init;

    // Preallocate matrices and vectors for efficiency
    pinocchio::SE3 fk_pose;
    Eigen::MatrixXd J(6, nx);
    Eigen::Vector<double, 6> err;
    double error_norm = 0.0;

    // Set joint velocity bounds for QP solver
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(nx, -0.1);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(nx, 0.1);

    // Initialize QP solver with box constraints
    proxsuite::proxqp::dense::QP<double> qp(nx, n_eq, n_in, true);
    qp.init(Eigen::MatrixXd::Identity(nx, nx), Eigen::VectorXd::Zero(nx),
            Eigen::MatrixXd(),
            Eigen::VectorXd(), // no equality constraints (A_eq, b_eq)
            Eigen::MatrixXd(),
            Eigen::VectorXd(), // no general inequalities (C, l)
            Eigen::VectorXd(), // no general upper bounds (u)
            lb, ub             // box constraints
    );

    // Line search parameters for convergence
    const double alpha_init = 1.0;
    const double rho = 0.5;
    const double alpha_min = 1e-6;

    // Main IK iteration loop
    for (int iter = 0; iter < MAX_IK_ITERATIONS; ++iter) {
        // 1) Compute forward kinematics and geometric Jacobian
        fk_pose = robot_model_.computeForwardKinematics(q);
        J = robot_model_.computeGeometricJacobian(q); // size 6×nx

        // 2) Compute task error using SE3 logarithm
        err = -(pinocchio::log6(target_pose.inverse() * fk_pose)).toVector();
        error_norm = err.norm();

        // Check for convergence
        if (error_norm < 1e-9) {
            std::cout << "Converged in " << iter
                      << " iterations \nq = " << q.transpose() * (180 / 3.14159)
                      << "\nerror norm = " << error_norm << std::endl;
            break;
        }

        // 3) Build QP problem: H = JᵀJ + λI, g = -Jᵀ err
        const double lambda = 1e-3; // Regularization parameter
        Eigen::MatrixXd H =
            J.transpose() * J + lambda * Eigen::MatrixXd::Identity(nx, nx);

        Eigen::VectorXd g = -J.transpose() * err;

        // 4) Compute per-iteration delta-bounds to enforce joint limits
        Eigen::VectorXd dq_lb =
            (robot_model_.getJointLimitsLower() - q).cwiseMax(lb);
        Eigen::VectorXd dq_ub =
            (robot_model_.getJointLimitsUpper() - q).cwiseMin(ub);

        // Update and solve QP
        qp.update(
            /*H=*/H,
            /*g=*/g,
            /*A=*/std::nullopt,
            /*b=*/std::nullopt,
            /*C=*/std::nullopt,
            /*l=*/std::nullopt,
            /*u=*/std::nullopt,
            /*l_box=*/dq_lb,
            /*u_box=*/dq_ub,
            /*recond=*/false // skip expensive preconditioner rebuild
        );
        qp.solve();

        Eigen::VectorXd dq = qp.results.x;

        // 5) Backtracking line search along dq
        double alpha = alpha_init;
        const double err0 = error_norm;
        pinocchio::SE3 fk_tmp;
        Eigen::Vector<double, 6> err_tmp;
        double err_tmp_norm;

        // Line search to find optimal step size
        while (alpha > alpha_min) {
            Eigen::VectorXd q_test = q + alpha * dq;
            // evaluate new error
            fk_tmp = robot_model_.computeForwardKinematics(q_test);
            err_tmp =
                -(pinocchio::log6(target_pose.inverse() * fk_tmp)).toVector();
            err_tmp_norm = err_tmp.norm();
            if (err_tmp_norm < err0)
                break;
            alpha *= rho;
        }

        // 6) Update joint configuration
        q += alpha * dq;
    }

    // Print the final pose after IK
    pinocchio::SE3 final_pose = robot_model_.computeForwardKinematics(q);
    std::cout << "Final end-effector pose after IK (SE3):" << std::endl;
    std::cout << final_pose << std::endl;
    robot_model_.setCurrentJointConfig(q);
    return q;
}

/**
 * @brief Solve IK with explicit initial configuration for XYZQuat input
 *
 * **Method Flow:**
 * 1. Convert XYZQuat input to SE3 transform
 * 2. Normalize quaternion to ensure unit length
 * 3. Call the SE3 version of solve_ik()
 * 4. Print final pose in XYZQuat format for verification
 *
 * This method provides a convenient interface for users who prefer
 * XYZQuat format and want to specify their own initial configuration.
 */
JointConfig
IKSolverWithRWS::solve_ik(const Eigen::Vector<double, 7> &target_pose_xyzquat,
                          const JointConfig *q_init) {
    // Target pose
    pinocchio::SE3 target_pose_se3 = pinocchio::SE3::Identity();
    target_pose_se3.translation() = target_pose_xyzquat.head<3>();
    Eigen::Quaterniond quat(target_pose_xyzquat[6], target_pose_xyzquat[3],
                            target_pose_xyzquat[4], target_pose_xyzquat[5]);
    quat.normalize(); // *** ensure it's unit length ***
    target_pose_se3.rotation() = quat.toRotationMatrix();

    JointConfig q = solve_ik(target_pose_se3, q_init);
    std::cout << "Final end-effector pose after IK (XYZQuat):" << std::endl;
    XYZQuat final_pose_xyzquat = RobotModel::homogeneousToXYZQuat(
        robot_model_.computeForwardKinematics(q));
    std::cout << final_pose_xyzquat.transpose() << std::endl;
    return q;
}

/**
 * @brief Find the latest k-d tree data file in the workspace directory
 *
 * **Search Logic:**
 * 1. Check if directory exists
 * 2. Iterate through all files in directory
 * 3. Filter files by name pattern: "kdtree_data_*.bin"
 * 4. Compare modification times to find most recent
 * 5. Return path to latest file or empty string if none found
 *
 * **File Naming Convention:**
 * - Files must start with "kdtree_data_"
 * - Files must end with ".bin"
 * - Modification time determines "latest"
 *
 * This method enables automatic loading of the most recent workspace data
 * without manual file specification.
 */
std::string
IKSolverWithRWS::findLatestKDTreeFile(const std::string &directory) {
    if (!std::filesystem::exists(directory)) {
        std::cerr << "Directory does not exist: " << directory << std::endl;
        return "";
    }

    std::string latest_file;
    std::filesystem::file_time_type latest_time;

    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            // Check if it's a k-d tree file (starts with "kdtree_data_")
            if (filename.find("kdtree_data_") == 0 &&
                filename.find(".bin") != std::string::npos) {
                auto file_time = entry.last_write_time();
                if (latest_file.empty() || file_time > latest_time) {
                    latest_file = entry.path().string();
                    latest_time = file_time;
                }
            }
        }
    }

    return latest_file;
}

/**
 * @brief Find the latest voxel grid data file in the workspace directory
 *
 * **Search Logic:**
 * 1. Check if directory exists
 * 2. Iterate through all files in directory
 * 3. Filter files by name pattern: "workspace_grid_sparse_*.bin"
 * 4. Compare modification times to find most recent
 * 5. Return path to latest file or empty string if none found
 *
 * **File Naming Convention:**
 * - Files must start with "workspace_grid_sparse_"
 * - Files must end with ".bin"
 * - Modification time determines "latest"
 *
 * The voxel grid provides spatial indexing for efficient nearest neighbor
 * search.
 */
std::string
IKSolverWithRWS::findLatestVoxelGridFile(const std::string &directory) {
    if (!std::filesystem::exists(directory)) {
        std::cerr << "Directory does not exist: " << directory << std::endl;
        return "";
    }

    std::string latest_file;
    std::filesystem::file_time_type latest_time;

    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            // Check if it's a sparse voxel grid file (starts with
            // "workspace_grid_sparse_")
            if (filename.find("workspace_grid_sparse_") == 0 &&
                filename.find(".bin") != std::string::npos) {
                auto file_time = entry.last_write_time();
                if (latest_file.empty() || file_time > latest_time) {
                    latest_file = entry.path().string();
                    latest_time = file_time;
                }
            }
        }
    }

    return latest_file;
}

/**
 * @brief Load workspace points from k-d tree binary file
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
 * **Error Handling:**
 * - Returns empty vector if file cannot be opened
 * - Returns empty vector if header read fails
 * - Stops loading if any point read fails
 * - Provides detailed error messages
 *
 * **Performance Optimizations:**
 * - Pre-allocates vector to avoid reallocation
 * - Uses binary I/O for efficiency
 * - Normalizes quaternions during loading
 */
std::vector<WorkspacePoint>
IKSolverWithRWS::loadKDTreeData(const std::string &filename) {
    std::vector<WorkspacePoint> workspace_points;

    std::ifstream kdtree_file(filename, std::ios::binary);
    if (!kdtree_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename
                  << " for reading." << std::endl;
        return workspace_points;
    }

    // Read header information
    size_t num_points;
    kdtree_file.read(reinterpret_cast<char *>(&num_points), sizeof(size_t));

    if (kdtree_file.fail()) {
        std::cerr << "Error: Failed to read number of points from file."
                  << std::endl;
        return workspace_points;
    }

    std::cout << "Loading " << num_points
              << " workspace points from: " << filename << std::endl;

    workspace_points.reserve(num_points);

    // Read data for each point
    for (size_t i = 0; i < num_points; ++i) {
        // Read position (3 doubles)
        double x, y, z;
        kdtree_file.read(reinterpret_cast<char *>(&x), sizeof(double));
        kdtree_file.read(reinterpret_cast<char *>(&y), sizeof(double));
        kdtree_file.read(reinterpret_cast<char *>(&z), sizeof(double));

        // Read quaternion (4 doubles: w, x, y, z)
        double qw, qx, qy, qz;
        kdtree_file.read(reinterpret_cast<char *>(&qw), sizeof(double));
        kdtree_file.read(reinterpret_cast<char *>(&qx), sizeof(double));
        kdtree_file.read(reinterpret_cast<char *>(&qy), sizeof(double));
        kdtree_file.read(reinterpret_cast<char *>(&qz), sizeof(double));

        // Read joint configuration (6 doubles)
        JointConfig joint_config;
        for (int j = 0; j < joint_config.size(); ++j) {
            double joint_val;
            kdtree_file.read(reinterpret_cast<char *>(&joint_val),
                             sizeof(double));
            joint_config[j] = joint_val;
        }

        // Check for read errors
        if (kdtree_file.fail()) {
            std::cerr << "Error: Failed to read point " << i << " from file."
                      << std::endl;
            break;
        }

        // Create WorkspacePoint
        Eigen::Vector3d position(x, y, z);
        Eigen::Quaterniond rotation(qw, qx, qy, qz);
        rotation.normalize(); // Ensure unit quaternion

        workspace_points.emplace_back(position, rotation, joint_config);
    }

    kdtree_file.close();
    std::cout << "Successfully loaded " << workspace_points.size()
              << " workspace points." << std::endl;

    return workspace_points;
}

/**
 * @brief Load voxel grid data from binary file
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
 * **Voxel Grid Structure:**
 * - minB: minimum bounds of the grid (3D point)
 * - r: resolution (voxel size)
 * - dims: grid dimensions (3D vector)
 * - maxDistance: maximum search distance
 * - numCells: number of occupied cells
 * - sparseNearestIdx: map from cell index to nearest point index
 *
 * The voxel grid provides O(1) spatial indexing for nearest neighbor search.
 *
 * **Error Handling:**
 * - Returns empty grid if file cannot be opened
 * - Returns empty grid if header read fails
 * - Stops loading if any data read fails
 * - Provides detailed error messages
 */
VoxelGrid IKSolverWithRWS::loadVoxelGridData(const std::string &filename) {
    VoxelGrid grid;

    std::ifstream in(filename, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "Error: Could not open sparse voxel grid file " << filename
                  << " for reading." << std::endl;
        return grid;
    }

    // Read header: minB(3×double), r(double), dims(3×int),
    // maxDistance(int), numCells(size_t)
    in.read(reinterpret_cast<char *>(grid.minB.data()), sizeof(double) * 3);
    in.read(reinterpret_cast<char *>(&grid.r), sizeof(double));
    in.read(reinterpret_cast<char *>(grid.dims.data()), sizeof(int) * 3);
    in.read(reinterpret_cast<char *>(&grid.maxDistance), sizeof(int));
    in.read(reinterpret_cast<char *>(&grid.numCells), sizeof(size_t));

    if (in.fail()) {
        std::cerr << "Error: Failed to read sparse voxel grid header from file."
                  << std::endl;
        return grid;
    }

    // Read sparse data: (cell_index, nearest_point_index) pairs
    for (size_t i = 0; i < grid.numCells; ++i) {
        size_t cellIndex;
        int nearestPointIndex;

        in.read(reinterpret_cast<char *>(&cellIndex), sizeof(size_t));
        in.read(reinterpret_cast<char *>(&nearestPointIndex), sizeof(int));

        if (in.fail()) {
            std::cerr << "Error: Failed to read sparse voxel grid data at cell "
                      << i << "." << std::endl;
            return grid;
        }

        grid.sparseNearestIdx[cellIndex] = nearestPointIndex;
    }

    in.close();

    std::cout << "Loaded sparse voxel grid: dims=[" << grid.dims.transpose()
              << "], r=" << grid.r << ", maxDistance=" << grid.maxDistance
              << ", cells=" << grid.numCells << std::endl;

    return grid;
}

/**
 * @brief Find k nearest neighbors using voxel grid spatial indexing
 *
 * **Algorithm Flow:**
 * 1. **Grid Indexing**: Convert target position to voxel grid coordinates
 * 2. **Onion Ring Search**: Search in expanding rings around target cell
 *    - Start with center cell
 *    - Expand to 6-connected neighbors
 *    - Continue until k neighbors found or search exhausted
 * 3. **Distance Calculation**: Compute Euclidean distances to all candidates
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
 * **Grid Coordinate Calculation:**
 * - idx = floor((target_xyz - minB) / r)
 * - Clamped to grid bounds [0, dims-1]
 * - Flat index: c0 = idx.x * dims.y * dims.z + idx.y * dims.z + idx.z
 *
 * **6-Connected Neighbors:**
 * - Offsets: (±1,0,0), (0,±1,0), (0,0,±1)
 * - Ensures complete coverage of nearby cells
 * - Avoids revisiting cells using seenCells set
 */
std::vector<NearestNeighbor> IKSolverWithRWS::findKNearestNeighborsVoxel(
    const std::vector<WorkspacePoint> &workspace_points, const VoxelGrid &grid,
    const Eigen::Vector3d &target_xyz, int k) {

    std::vector<NearestNeighbor> neighbors;

    if (workspace_points.empty() || grid.numCells == 0) {
        return neighbors;
    }

    // Clamp target position into grid bounds
    Eigen::Vector3i idx = ((target_xyz - grid.minB) / grid.r)
                              .cast<int>()
                              .cwiseMax(Eigen::Vector3i::Zero())
                              .cwiseMin(grid.dims - Eigen::Vector3i::Ones());

    // Compute flat index for the target cell
    size_t c0 = static_cast<size_t>(idx.x()) * grid.dims.y() * grid.dims.z() +
                static_cast<size_t>(idx.y()) * grid.dims.z() +
                static_cast<size_t>(idx.z());

    // Gather candidate sample indices in "onion rings"
    std::vector<size_t> candidates;
    candidates.reserve(16);

    // Keep a set of which samples we've already added to ensure uniqueness
    std::unordered_set<size_t> uniqueSamples;

    // Helper to push unique sample index
    auto pushSample = [&](int sampleIdx) {
        if (sampleIdx >= 0 &&
            static_cast<size_t>(sampleIdx) < workspace_points.size() &&
            uniqueSamples.insert(static_cast<size_t>(sampleIdx))
                .second) { // only true if newly inserted
            candidates.push_back(static_cast<size_t>(sampleIdx));
        }
    };

    // Start with the center cell's representative (if it exists in sparse
    // grid)
    auto centerIt = grid.sparseNearestIdx.find(c0);
    if (centerIt != grid.sparseNearestIdx.end()) {
        pushSample(centerIt->second);
    }

    // 6-connected neighbor offsets
    static const int offs6[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                    {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

    std::vector<size_t> seenCells; // to avoid revisiting
    seenCells.reserve(k * 2);
    seenCells.push_back(c0);

    std::vector<Eigen::Vector3i> frontier = {idx};
    while (uniqueSamples.size() < static_cast<size_t>(k) && !frontier.empty()) {
        std::vector<Eigen::Vector3i> nextFrontier;
        for (const auto &cell : frontier) {
            for (const auto &d : offs6) {
                Eigen::Vector3i n = cell + Eigen::Vector3i(d[0], d[1], d[2]);
                if ((n.array() < 0).any() ||
                    (n.array() >= grid.dims.array()).any()) {
                    continue;
                }

                size_t nc =
                    static_cast<size_t>(n.x()) * grid.dims.y() * grid.dims.z() +
                    static_cast<size_t>(n.y()) * grid.dims.z() +
                    static_cast<size_t>(n.z());

                // Avoid re-expanding this cell
                if (std::find(seenCells.begin(), seenCells.end(), nc) !=
                    seenCells.end()) {
                    continue;
                }

                seenCells.push_back(nc);

                // Add the nearest sample for this neighbor cell (if it
                // exists in sparse grid)
                auto neighborIt = grid.sparseNearestIdx.find(nc);
                if (neighborIt != grid.sparseNearestIdx.end()) {
                    pushSample(neighborIt->second);
                }

                nextFrontier.push_back(n);
            }
        }
        frontier.swap(nextFrontier);
    }

    // No need to remove duplicates since we guaranteed uniqueness during
    // insertion candidates.size() == uniqueSamples.size() and is at most k

    // Trim to the k closest by position
    struct DistIdx {
        double d2;
        size_t idx;

        bool operator<(const DistIdx &other) const { return d2 < other.d2; }
    };

    std::vector<DistIdx> distList;
    distList.reserve(candidates.size());

    for (auto si : candidates) {
        const auto &wp = workspace_points[si];
        double d2 = (wp.position - target_xyz).squaredNorm();
        distList.push_back({d2, si});
    }

    // Get top-k by distance squared
    if (distList.size() > static_cast<size_t>(k)) {
        std::nth_element(distList.begin(), distList.begin() + k,
                         distList.end());
        distList.resize(k);
    }

    // Convert to NearestNeighbor format
    neighbors.reserve(distList.size());
    for (const auto &di : distList) {
        double distance = std::sqrt(di.d2);
        neighbors.emplace_back(di.idx, distance, &workspace_points[di.idx]);
    }

    // Print the nearest neighbors with the distance between the target and
    // the neighbors
    std::cout << "Nearest neighbors to target (" << target_xyz.x() << ", "
              << target_xyz.y() << ", " << target_xyz.z() << "):" << std::endl;
    for (const auto &neighbor : neighbors) {
        const auto &wp = *neighbor.point;
        std::cout << "  Index: " << neighbor.index << ", Position: ("
                  << wp.position.x() << ", " << wp.position.y() << ", "
                  << wp.position.z() << ")"
                  << ", Distance: " << neighbor.distance << std::endl;
    }

    return neighbors;
}

/**
 * @brief Find k nearest neighbors using k-d tree spatial indexing
 *
 * **Algorithm Flow:**
 * 1. **Tree Construction**: Build k-d tree from workspace points
 * 2. **Nearest Neighbor Search**: Use nanoflann library for efficient search
 * 3. **Distance Conversion**: Convert squared distances to actual distances
 * 4. **Result Formatting**: Convert to NearestNeighbor format
 *
 * **Performance Characteristics:**
 * - O(log n) search time for balanced trees
 * - O(n log n) tree construction time
 * - More accurate than voxel grid method but potentially slower
 *
 * **Nanoflann Integration:**
 * - Uses WorkspaceAdaptor to interface with custom data structure
 * - L2 distance metric for Euclidean distance calculation
 * - Returns both indices and squared distances
 *
 * This method provides a fallback when voxel grid is not available or
 * when more precise nearest neighbor search is required.
 *
 * **Search Process:**
 * - Constructs adaptor from workspace points
 * - Builds k-d tree with specified parameters
 * - Performs k-nearest neighbor search
 * - Converts results to standard format
 */
std::vector<NearestNeighbor> IKSolverWithRWS::findKNearestNeighbors(
    const std::vector<WorkspacePoint> &workspace_points,
    const Eigen::Vector3d &target_xyz, int k) {

    std::vector<NearestNeighbor> neighbors;

    if (workspace_points.empty()) {
        return neighbors;
    }

    // Construct the adaptor & the index
    WorkspaceAdaptor wc_adaptor(workspace_points);
    KDTree3D kdtree(3, wc_adaptor,
                    nanoflann::KDTreeSingleIndexAdaptorParams(10));

    // Build the index
    kdtree.buildIndex();

    // Prepare query point
    double query_pt[3] = {target_xyz.x(), target_xyz.y(), target_xyz.z()};

    // Prepare output arrays
    std::vector<unsigned int> ret_idx(k);
    std::vector<double> out_dist2(k);

    // Perform the search
    const std::size_t found =
        kdtree.knnSearch(&query_pt[0],    // query data
                         k,               // num neighbors
                         ret_idx.data(),  // output: indices
                         out_dist2.data() // output: squared distances
        );

    // Convert results to our format
    neighbors.reserve(found);
    for (std::size_t i = 0; i < found; ++i) {
        std::size_t idx = static_cast<std::size_t>(ret_idx[i]);
        double distance = std::sqrt(
            out_dist2[i]); // Convert squared distance to actual distance
        neighbors.emplace_back(idx, distance, &workspace_points[idx]);
    }

    return neighbors;
}