#include "ik_solver_w_rws.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <unordered_set>

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

        for (const auto &neighbor : nearest_neighbors_voxel) {
            const auto &wp = workspace_points_[neighbor.index];

            // Geodesic rotation error
            double ang = 2.0 * std::acos(std::abs(wp.rotation.dot(quat)));

            if (ang < bestPoseErr) {
                bestPoseErr = ang;
                bestIdx = neighbor.index;
            }
        }

        q_init = workspace_points_[bestIdx].joint_config;
        std::cout << "Selected initial configuration with rotation error: "
                  << bestPoseErr << " radians" << std::endl;
        std::cout << "Initial joint config: [" << q_init.transpose() << "]"
                  << std::endl;

        // // Measure error between best nearest neighbor and target
        // pinocchio::SE3 target_pose;
        // target_pose.translation() = xyz;
        // target_pose.rotation() = quat.toRotationMatrix();

        // pinocchio::SE3 best_neighbor_pose;
        // best_neighbor_pose.translation() =
        // workspace_points_[bestIdx].position; best_neighbor_pose.rotation() =
        //     workspace_points_[bestIdx].rotation.toRotationMatrix();

        // double best_neighbor_error =
        //     (pinocchio::log6(target_pose.inverse() * best_neighbor_pose))
        //         .toVector()
        //         .norm();

        // // Measure error between current end effector pose and target
        // pinocchio::SE3 current_pose = robot_model_.computeForwardKinematics(
        //     robot_model_.getCurrentJointConfig());
        // double current_pose_error =
        //     (pinocchio::log6(target_pose.inverse() * current_pose))
        //         .toVector()
        //         .norm();

        // // Measure error between neutral joint config pose and target
        // pinocchio::SE3 neutral_pose = robot_model_.computeForwardKinematics(
        //     RobotModel::NEUTRAL_JOINT_CONFIG);
        // double neutral_pose_error =
        //     (pinocchio::log6(target_pose.inverse() * neutral_pose))
        //         .toVector()
        //         .norm();

        // std::cout << "Best neighbor error: " << best_neighbor_error
        //           << ", Current pose error: " << current_pose_error
        //           << ", Neutral config error: " << neutral_pose_error
        //           << std::endl;

        // // If best_neighbor_error > 2.0, default to current joint config
        // if (best_neighbor_error > 2.0) {
        //     q_init = robot_model_.getCurrentJointConfig();
        //     std::cout << "Best neighbor error exceeds threshold (2.0). "
        //                  "Defaulting to current joint config."
        //               << std::endl;
        // } else {
        //     // Choose the joint config with the least error as q_init
        //     double min_error = best_neighbor_error;
        //     JointConfig min_config = workspace_points_[bestIdx].joint_config;

        //     if (current_pose_error < min_error) {
        //         min_error = current_pose_error;
        //         min_config = robot_model_.getCurrentJointConfig();
        //     }
        //     if (neutral_pose_error < min_error) {
        //         min_error = neutral_pose_error;
        //         min_config = RobotModel::NEUTRAL_JOINT_CONFIG;
        //     }

        //     q_init = min_config;
        //     std::cout << "Selected initial configuration with minimum error:
        //     "
        //               << min_error << std::endl;
        //     std::cout << "Initial joint config: [" << q_init.transpose() <<
        //     "]"
        //               << std::endl;
        // }
    } else {
        std::cout << "No neighbors found, using current joint configuration as "
                     "warm-start."
                  << std::endl;
        q_init = robot_model_.getCurrentJointConfig();
    }
    for (uint8_t i = 0; i < robot_model_.getNumJoints(); i++) {
        if (q_init[i] > M_PI) {
            q_init[i] -= 2 * M_PI;
        } else if (q_init[i] < -M_PI) {
            q_init[i] += 2 * M_PI;
        }
    }
    // Solve IK with warm-start
    return solve_ik(target_pose_xyzquat, &q_init);
}

JointConfig IKSolverWithRWS::solve(const pinocchio::SE3 &target_pose) {
    // Convert SE3 to XYZQuat format
    XYZQuat target_xyzquat = RobotModel::homogeneousToXYZQuat(target_pose);
    return solve(target_xyzquat);
}

JointConfig IKSolverWithRWS::solve_ik(const pinocchio::SE3 &target_pose,
                                      const JointConfig *q_init) {
    const int nx = robot_model_.getNumVelocityVariables();
    const int n_eq = 0;
    const int n_in = 0;

    Eigen::VectorXd q;
    if (q_init == nullptr)
        q = robot_model_.getCurrentJointConfig();
    else
        q = *q_init;

    // Preallocate
    pinocchio::SE3 fk_pose;
    Eigen::MatrixXd J(6, nx);
    Eigen::Vector<double, 6> err;
    double error_norm = 0.0;

    // Joint velocity bounds
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(nx, -0.1);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(nx, 0.1);

    // QP solver (box constraints = true)
    proxsuite::proxqp::dense::QP<double> qp(nx, n_eq, n_in, true);
    qp.init(Eigen::MatrixXd::Identity(nx, nx), Eigen::VectorXd::Zero(nx),
            Eigen::MatrixXd(),
            Eigen::VectorXd(), // no equality constraints (A_eq, b_eq)
            Eigen::MatrixXd(),
            Eigen::VectorXd(), // no general inequalities (C, l)
            Eigen::VectorXd(), // no general upper bounds (u)
            lb, ub             // box constraints
    );

    // Line search parameters
    const double alpha_init = 1.0;
    const double rho = 0.5;
    const double alpha_min = 1e-6;

    for (int iter = 0; iter < MAX_IK_ITERATIONS; ++iter) {
        // 1) Forward kinematics & Jacobian
        fk_pose = robot_model_.computeForwardKinematics(q);
        J = robot_model_.computeGeometricJacobian(q); // size 6×nx

        // 2) Compute task error
        err = -(pinocchio::log6(target_pose.inverse() * fk_pose)).toVector();
        error_norm = err.norm();
        if (error_norm < 1e-9) {
            std::cout << "Converged in " << iter
                      << " iterations \nq = " << q.transpose() * (180 / 3.14159)
                      << "\nerror norm = " << error_norm << std::endl;
            break;
        }

        // 3) Build QP: H = JᵀJ, g = -Jᵀ err
        // Eigen::MatrixXd H = J.transpose() * J;
        const double lambda = 1e-3;
        Eigen::MatrixXd H =
            J.transpose() * J + lambda * Eigen::MatrixXd::Identity(nx, nx);

        Eigen::VectorXd g = -J.transpose() * err;

        // 4) Initialize and solve QP with box constraints

        // A) Compute per‐iteration delta‐bounds so that (q + dq) ∈ [q_min,
        // q_max]
        Eigen::VectorXd dq_lb =
            (robot_model_.getJointLimitsLower() - q).cwiseMax(lb);
        Eigen::VectorXd dq_ub =
            (robot_model_.getJointLimitsUpper() - q).cwiseMin(ub);

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