#include "ik_model.h"
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

JointConfig solve_ik(IKModel &ik_model, const pinocchio::SE3 &target_pose,
                     const JointConfig *q_init = nullptr) {
    const int nx = ik_model.getNumVelocityVariables();
    const int n_eq = 0;
    const int n_in = 0;

    Eigen::VectorXd q;
    if (q_init == nullptr)
        q = ik_model.getCurrentJointConfig();
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
    // std::cout << "Box‐constrained? " << std::boolalpha
    //           << qp.is_box_constrained() << std::endl;
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
    const double alpha_min = 1e-4;

    // Timing variables for per-iteration measurements
    long long total_fk_jacobian_time = 0;
    long long total_qp_time = 0;
    long long total_line_search_time = 0;
    int iterations_completed = 0;

    for (int iter = 0; iter < 1000; ++iter) {
        // 1) Forward kinematics & Jacobian
        auto start_fk_jacobian = std::chrono::high_resolution_clock::now();
        fk_pose = ik_model.computeForwardKinematics(q);
        J = ik_model.computeGeometricJacobian(q); // size 6×nx
        auto end_fk_jacobian = std::chrono::high_resolution_clock::now();
        total_fk_jacobian_time +=
            std::chrono::duration_cast<std::chrono::microseconds>(
                end_fk_jacobian - start_fk_jacobian)
                .count();

        // 2) Compute task error
        err = -(pinocchio::log6(target_pose.inverse() * fk_pose)).toVector();
        error_norm = err.norm();
        if (error_norm < 1e-8) {
            std::cout << "Converged in " << iter
                      << " iterations \nq = " << q.transpose() * (180 / 3.14159)
                      << "\nerror norm = " << error_norm << std::endl;
            iterations_completed = iter + 1;
            break;
        }
        // std::cout << "iter " << iter << "  err = " << error_norm << "\r";

        // 3) Build QP: H = JᵀJ, g = -Jᵀ err
        Eigen::MatrixXd H = J.transpose() * J;
        Eigen::VectorXd g = -J.transpose() * err;

        // 4) Initialize and solve QP with box constraints
        auto start_qp = std::chrono::high_resolution_clock::now();
        // Note: we supply 9 args: H, g, A_eq, b_eq, C, l, u, lb, ub

        // A) Compute per‐iteration delta‐bounds so that (q + dq) ∈ [q_min,
        // q_max]
        Eigen::VectorXd dq_lb =
            (ik_model.getJointLimitsLower() - q).cwiseMax(lb);
        Eigen::VectorXd dq_ub =
            (ik_model.getJointLimitsUpper() - q).cwiseMin(ub);

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
        auto end_qp = std::chrono::high_resolution_clock::now();
        total_qp_time += std::chrono::duration_cast<std::chrono::microseconds>(
                             end_qp - start_qp)
                             .count();

        Eigen::VectorXd dq = qp.results.x;

        // 5) Backtracking line search along dq
        auto start_line_search = std::chrono::high_resolution_clock::now();
        double alpha = alpha_init;
        const double err0 = error_norm;
        pinocchio::SE3 fk_tmp;
        Eigen::Vector<double, 6> err_tmp;
        double err_tmp_norm;

        while (alpha > alpha_min) {
            Eigen::VectorXd q_test = q + alpha * dq;
            // evaluate new error
            fk_tmp = ik_model.computeForwardKinematics(q_test);
            err_tmp =
                -(pinocchio::log6(target_pose.inverse() * fk_tmp)).toVector();
            err_tmp_norm = err_tmp.norm();
            if (err_tmp_norm < err0)
                break;
            alpha *= rho;
        }
        auto end_line_search = std::chrono::high_resolution_clock::now();
        total_line_search_time +=
            std::chrono::duration_cast<std::chrono::microseconds>(
                end_line_search - start_line_search)
                .count();

        // 6) Update joint configuration
        q += alpha * dq;
        iterations_completed = iter + 1;
    }

    // Print average per-iteration timings
    if (iterations_completed > 0) {
        std::cout << "\nPer-iteration timing averages (" << iterations_completed
                  << " iterations):" << std::endl;
        std::cout << "  Forward kinematics + Jacobian: "
                  << (total_fk_jacobian_time / iterations_completed) << " us"
                  << std::endl;
        std::cout << "  QP setup & solve: "
                  << (total_qp_time / iterations_completed) << " us"
                  << std::endl;
        std::cout << "  Backtracking line search: "
                  << (total_line_search_time / iterations_completed) << " us"
                  << std::endl;
    }
    // Print the final pose after IK
    pinocchio::SE3 final_pose = ik_model.computeForwardKinematics(q);
    std::cout << "Final end-effector pose after IK (SE3):" << std::endl;
    std::cout << final_pose << std::endl;
    ik_model.setCurrentJointConfig(q);
    return q;
}

JointConfig solve_ik(IKModel &ik_model,
                     const Eigen::Vector<double, 7> &target_pose_xyzquat,
                     const JointConfig *q_init = nullptr) {

    // Target pose
    pinocchio::SE3 target_pose_se3 = pinocchio::SE3::Identity();
    target_pose_se3.translation() = target_pose_xyzquat.head<3>();
    target_pose_se3.rotation() =
        Eigen::Quaterniond(target_pose_xyzquat[6], target_pose_xyzquat[3],
                           target_pose_xyzquat[4], target_pose_xyzquat[5])
            .toRotationMatrix();
    JointConfig q = solve_ik(ik_model, target_pose_se3, q_init);
    std::cout << "Final end-effector pose after IK (XYZQuat):" << std::endl;
    XYZQuat final_pose_xyzquat =
        IKModel::homogeneousToXYZQuat(ik_model.computeForwardKinematics(q));
    std::cout << final_pose_xyzquat.transpose() << std::endl;
    return q;
}

JointConfig solve_ik_dls(IKModel &ik_model, const pinocchio::SE3 &target_pose,
                         const JointConfig *q_init) {
    const int nx = ik_model.getNumVelocityVariables();
    Eigen::VectorXd q = (q_init ? *q_init : ik_model.getCurrentJointConfig());

    // Joint limits (user‐provided)
    Eigen::VectorXd q_min = ik_model.getJointLimitsLower();
    Eigen::VectorXd q_max = ik_model.getJointLimitsUpper();

    const double vel_limit = 0.1; // rad per iteration
    const double lambda = 1e-3;   // Tikhonov damping

    for (int iter = 0; iter < 1000; ++iter) {
        // 1) FK & error
        auto fk = ik_model.computeForwardKinematics(q);
        Eigen::Vector<double, 6> err =
            -(pinocchio::log6(target_pose.inverse() * fk)).toVector();
        if (err.norm() < 1e-8)
            break;

        // 2) Jacobian
        Eigen::MatrixXd J = ik_model.computeGeometricJacobian(q);

        // 3) DLS step: (JᵀJ + λI) dq = -Jᵀ err
        Eigen::MatrixXd H_reg =
            J.transpose() * J + lambda * Eigen::MatrixXd::Identity(nx, nx);
        Eigen::VectorXd b = -J.transpose() * err;
        Eigen::VectorXd dq = H_reg.ldlt().solve(b);

        // 4) Clip to both vel‐ and joint‐limits
        for (int i = 0; i < nx; ++i) {
            double dq_min = std::max(-vel_limit, q_min[i] - q[i]);
            double dq_max = std::min(vel_limit, q_max[i] - q[i]);
            dq[i] = std::clamp(dq[i], dq_min, dq_max);
        }

        // 5) Update
        q += dq;
    }

    ik_model.setCurrentJointConfig(q);
    return q;
}

// Structure to hold workspace point data with position, rotation, and joint
// config
struct WorkspacePoint {
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    JointConfig joint_config;

    WorkspacePoint(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot,
                   const JointConfig &config)
        : position(pos), rotation(rot), joint_config(config) {}
};

// Helper function to find the latest k-d tree file
std::string findLatestKDTreeFile(const std::string &directory) {
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

// Helper function to find the latest sparse voxel grid file
std::string findLatestVoxelGridFile(const std::string &directory) {
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

// Helper function to load k-d tree data from binary file
std::vector<WorkspacePoint> loadKDTreeData(const std::string &filename) {
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

// Structure to hold nearest neighbor results
struct NearestNeighbor {
    std::size_t index;
    double distance;
    const WorkspacePoint *point;

    NearestNeighbor(std::size_t idx, double dist, const WorkspacePoint *pt)
        : index(idx), distance(dist), point(pt) {}

    // Comparison operator for sorting (closest first)
    bool operator<(const NearestNeighbor &other) const {
        return distance < other.distance;
    }
};

// Structure to hold sparse voxel grid data
struct VoxelGrid {
    Eigen::Vector3d minB;
    double r;
    Eigen::Vector3i dims;
    int maxDistance;
    size_t numCells;
    std::map<size_t, int> sparseNearestIdx; // cell_index -> nearest_point_index

    VoxelGrid() : r(0.0), maxDistance(0), numCells(0) {}
};

// Helper function to load sparse voxel grid data from binary file
VoxelGrid loadVoxelGridData(const std::string &filename) {
    VoxelGrid grid;

    std::ifstream in(filename, std::ios::binary);
    if (!in.is_open()) {
        std::cerr << "Error: Could not open sparse voxel grid file " << filename
                  << " for reading." << std::endl;
        return grid;
    }

    // Read header: minB(3×double), r(double), dims(3×int), maxDistance(int),
    // numCells(size_t)
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

// Helper function to find k nearest neighbors using sparse voxel grid
std::vector<NearestNeighbor>
findKNearestNeighborsVoxel(const std::vector<WorkspacePoint> &workspace_points,
                           const VoxelGrid &grid,
                           const Eigen::Vector3d &target_xyz, int k = 10) {

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

    // Helper to push unique sample index
    auto pushSample = [&](int sampleIdx) {
        if (sampleIdx >= 0 &&
            static_cast<size_t>(sampleIdx) < workspace_points.size()) {
            candidates.push_back(static_cast<size_t>(sampleIdx));
        }
    };

    // Start with the center cell's representative (if it exists in sparse grid)
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
    while (candidates.size() < static_cast<size_t>(k) && !frontier.empty()) {
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

                // Add the nearest sample for this neighbor cell (if it exists
                // in sparse grid)
                auto neighborIt = grid.sparseNearestIdx.find(nc);
                if (neighborIt != grid.sparseNearestIdx.end()) {
                    pushSample(neighborIt->second);
                }

                nextFrontier.push_back(n);
                if (candidates.size() >= static_cast<size_t>(k))
                    break;
            }
            if (candidates.size() >= static_cast<size_t>(k))
                break;
        }
        frontier.swap(nextFrontier);
    }

    // Remove duplicates from candidates
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()),
                     candidates.end());

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

    return neighbors;
}

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

// Helper function to find k nearest neighbors using nanoflann k-d tree
std::vector<NearestNeighbor>
findKNearestNeighbors(const std::vector<WorkspacePoint> &workspace_points,
                      const Eigen::Vector3d &target_xyz, int k = 10) {

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

// Helper function to find k nearest neighbors using brute force search (SLOW -
// for comparison)
std::vector<NearestNeighbor>
findKNearestNeighbors_old(const std::vector<WorkspacePoint> &workspace_points,
                          const Eigen::Vector3d &target_xyz, int k = 10) {

    std::vector<NearestNeighbor> neighbors;
    neighbors.reserve(k);

    // Calculate distances to all points
    for (std::size_t i = 0; i < workspace_points.size(); ++i) {
        const auto &point = workspace_points[i];
        double distance = (point.position - target_xyz).norm();

        // If we have fewer than k neighbors, add this one
        if (neighbors.size() < static_cast<std::size_t>(k)) {
            neighbors.emplace_back(i, distance, &point);
        } else {
            // If this point is closer than the farthest neighbor, replace it
            if (distance < neighbors.back().distance) {
                neighbors.back() = NearestNeighbor(i, distance, &point);
            }
        }

        // Keep the vector sorted (closest first)
        std::sort(neighbors.begin(), neighbors.end());
    }

    return neighbors;
}

// Helper function to process a single sample
bool processSample(const pinocchio::Model &model, pinocchio::Data &data,
                   const pinocchio::GeometryModel &geom_model,
                   pinocchio::GeometryData &geom_data,
                   const Eigen::VectorXd &joint_limits_lower,
                   const Eigen::VectorXd &joint_limits_upper,
                   const int frame_id, boost::random::sobol &seq,
                   std::vector<double> &u, JointConfig &q,
                   WorkspacePoint &result) {

    // Safety checks
    if (frame_id < 0 || frame_id >= model.frames.size()) {
        return false;
    }

    if (u.size() != model.nq) {
        return false;
    }

    // Generate Sobol sequence values for the entire point at once
    seq.generate(u.begin(), u.end());

    // Map to joint configuration with bounds checking
    for (int j = 0; j < model.nq; ++j) {
        if (j < joint_limits_lower.size() && j < joint_limits_upper.size()) {
            q[j] = joint_limits_lower[j] +
                   ((joint_limits_upper[j] - joint_limits_lower[j]) * u[j]);
        } else {
            return false; // Invalid joint limits
        }
    }

    try {
        // Compute forward kinematics
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);

        // Check for collisions
        bool collision_detected =
            pinocchio::computeCollisions(model, data, geom_model, geom_data, q);

        // Only return collision-free configurations above ground
        if (!collision_detected) {
            Eigen::Vector3d position = data.oMf[frame_id].translation();
            // if (position[2] > 0.0) {
            // Extract rotation matrix and convert to quaternion
            Eigen::Matrix3d rotation_matrix = data.oMf[frame_id].rotation();
            Eigen::Quaterniond rotation(rotation_matrix);
            rotation.normalize(); // Ensure unit quaternion

            result = WorkspacePoint(position, rotation, q);
            return true;
            // }
        }
    } catch (const std::exception &e) {
        // Log error and return false
        return false;
    } catch (...) {
        // Catch any other exceptions
        return false;
    }

    return false;
}

// Helper function to write binary PLY file
void writeBinaryPLY(const std::string &filename,
                    const std::vector<WorkspacePoint> &workspace_points) {
    std::ofstream ply_file(filename, std::ios::binary);

    if (!ply_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename
                  << " for writing." << std::endl;
        return;
    }

    // Write PLY header
    ply_file << "ply" << std::endl;
    ply_file << "format binary_little_endian 1.0" << std::endl;
    ply_file << "element vertex " << workspace_points.size() << std::endl;
    ply_file << "property float x" << std::endl;
    ply_file << "property float y" << std::endl;
    ply_file << "property float z" << std::endl;
    ply_file << "property float joint1" << std::endl;
    ply_file << "property float joint2" << std::endl;
    ply_file << "property float joint3" << std::endl;
    ply_file << "property float joint4" << std::endl;
    ply_file << "property float joint5" << std::endl;
    ply_file << "property float joint6" << std::endl;
    ply_file << "end_header" << std::endl;

    // Write binary data
    for (const auto &point : workspace_points) {
        // Write position (3 floats)
        float x = static_cast<float>(point.position[0]);
        float y = static_cast<float>(point.position[1]);
        float z = static_cast<float>(point.position[2]);
        ply_file.write(reinterpret_cast<const char *>(&x), sizeof(float));
        ply_file.write(reinterpret_cast<const char *>(&y), sizeof(float));
        ply_file.write(reinterpret_cast<const char *>(&z), sizeof(float));

        // Write joint configuration (6 floats)
        for (int j = 0; j < point.joint_config.size(); ++j) {
            float joint_val = static_cast<float>(point.joint_config[j]);
            ply_file.write(reinterpret_cast<const char *>(&joint_val),
                           sizeof(float));
        }
    }

    ply_file.close();
    std::cout << "\nWorkspace points saved to: " << filename << std::endl;
    std::cout << "You can now load this file in CloudCompare to visualize the "
                 "workspace."
              << std::endl;
}

// Helper function to write k-d tree data to binary file
void writeKDTreeData(const std::string &filename,
                     const std::vector<WorkspacePoint> &workspace_points) {
    std::ofstream kdtree_file(filename, std::ios::binary);

    if (!kdtree_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename
                  << " for writing." << std::endl;
        return;
    }

    // Write header information
    size_t num_points = workspace_points.size();
    kdtree_file.write(reinterpret_cast<const char *>(&num_points),
                      sizeof(size_t));

    // Write data for each point
    for (const auto &point : workspace_points) {
        // Write position (3 doubles)
        double x = point.position[0];
        double y = point.position[1];
        double z = point.position[2];
        kdtree_file.write(reinterpret_cast<const char *>(&x), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&y), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&z), sizeof(double));

        // Write quaternion (4 doubles: w, x, y, z)
        double qw = point.rotation.w();
        double qx = point.rotation.x();
        double qy = point.rotation.y();
        double qz = point.rotation.z();
        kdtree_file.write(reinterpret_cast<const char *>(&qw), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&qx), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&qy), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&qz), sizeof(double));

        // Write joint configuration (6 doubles)
        for (int j = 0; j < point.joint_config.size(); ++j) {
            double joint_val = point.joint_config[j];
            kdtree_file.write(reinterpret_cast<const char *>(&joint_val),
                              sizeof(double));
        }
    }

    kdtree_file.close();
    std::cout << "K-d tree data saved to: " << filename << std::endl;
    std::cout << "File contains " << workspace_points.size()
              << " points with position, rotation (quaternion), and joint "
                 "configuration."
              << std::endl;
}

// Helper function to get timestamped filename
std::string getTimestampedFilename(const std::string &base_dir,
                                   const std::string &prefix,
                                   const std::string &extension) {
    // Create output directory if it doesn't exist
    if (!std::filesystem::exists(base_dir)) {
        std::filesystem::create_directory(base_dir);
    }

    // Get current time for timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &now_c);
#else
    localtime_r(&now_c, &tm_buf);
#endif
    std::ostringstream oss;
    oss << base_dir << "/" << prefix << "_"
        << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << extension;
    return oss.str();
}

void computeReachableWorkspace(IKModel &ik_model, const int num_points) {
    // Compute reachable workspace
    // 1. Get the joint limits
    // 2. Generate random joint configurations
    // 3. Compute forward kinematics for each configuration
    // 4. Check for collisions
}

int main() {
    // Load the latest k-d tree data from generated_workspaces
    const std::string workspace_dir = "generated_workspaces";
    std::string latest_kdtree_file = findLatestKDTreeFile(workspace_dir);

    if (latest_kdtree_file.empty()) {
        std::cerr << "No k-d tree files found in " << workspace_dir
                  << std::endl;
        std::cerr << "Please run reachable_workspace first to generate "
                     "workspace data."
                  << std::endl;
        return -1;
    }

    std::cout << "Loading latest k-d tree file: " << latest_kdtree_file
              << std::endl;
    std::vector<WorkspacePoint> workspace_points =
        loadKDTreeData(latest_kdtree_file);

    if (workspace_points.empty()) {
        std::cerr << "Failed to load workspace points from k-d tree file."
                  << std::endl;
        return -1;
    }

    std::cout << "Successfully loaded " << workspace_points.size()
              << " workspace points." << std::endl;

    // Load the latest voxel grid data
    std::string latest_voxel_grid_file = findLatestVoxelGridFile(workspace_dir);
    VoxelGrid voxel_grid;

    if (latest_voxel_grid_file.empty()) {
        std::cout << "No voxel grid files found in " << workspace_dir
                  << ". Voxel grid search will be disabled." << std::endl;
    } else {
        std::cout << "Loading latest voxel grid file: "
                  << latest_voxel_grid_file << std::endl;
        voxel_grid = loadVoxelGridData(latest_voxel_grid_file);

        if (voxel_grid.numCells == 0) {
            std::cerr << "Failed to load sparse voxel grid data." << std::endl;
        } else {
            std::cout << "Successfully loaded sparse voxel grid with "
                      << voxel_grid.numCells << " cells." << std::endl;
        }
    }

    // Print first few points for verification
    std::cout << "\nFirst 3 workspace points:" << std::endl;
    for (int i = 0; i < std::min(3, static_cast<int>(workspace_points.size()));
         ++i) {
        const auto &point = workspace_points[i];
        std::cout << "Point " << i << ": Position=["
                  << point.position.transpose() << "], Joints=["
                  << point.joint_config.transpose() << "]" << std::endl;
    }
    std::cout << std::endl;

    // Initialize IK model
    IKModel ik_model(
        "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf",
        "connection_frame",
        "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.srdf");

    // // Target pose
    // pinocchio::SE3 target_pose = pinocchio::SE3::Identity();
    // target_pose.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
    // target_pose.rotation() =
    //     Eigen::Quaterniond(0.7071068, 0, -0.7071068, 0).toRotationMatrix();

    Eigen::Vector<double, 7> target_pose_xyzquat;
    // target_pose_xyzquat << 0.5, 0.5, 0.5, 0, -0.7071068, 0, 0.7071068;
    std::cout << "Enter target pose as 7 values [x y z qx qy qz qw]: ";
    for (int i = 0; i < 7; ++i) {
        std::cin >> target_pose_xyzquat[i];
    }

    // Start timing the entire IK solving process
    auto start_time_total_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();

    // Time the pose extraction step
    auto start_time_pose_extraction_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();

    // Split target pose into translation and quaternion components
    Eigen::Vector3d xyz =
        target_pose_xyzquat.head<3>(); // Extract translation [x, y, z]
    // User provides [qx, qy, qz, qw], Eigen::Quaterniond expects [qw, qx, qy,
    // qz]
    Eigen::Quaterniond quat(target_pose_xyzquat[6], target_pose_xyzquat[3],
                            target_pose_xyzquat[4], target_pose_xyzquat[5]);
    quat.normalize(); // Ensure unit quaternion

    auto end_time_pose_extraction_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    std::cout << "Time for pose extraction: "
              << (end_time_pose_extraction_us - start_time_pose_extraction_us)
              << " us" << std::endl;

    // // Find 10 nearest neighbors to the target xyz position
    // std::cout << "Finding 10 nearest neighbors to target position..."
    //           << std::endl;

    // // Test brute force method
    // auto start_time_us =
    //     std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now().time_since_epoch())
    //         .count();

    // std::vector<NearestNeighbor> nearest_neighbors_old =
    //     findKNearestNeighbors_old(workspace_points, xyz, 10);

    // auto end_time_us_old =
    //     std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now().time_since_epoch())
    //         .count();
    // std::cout << "Time taken for brute force: "
    //           << (end_time_us_old - start_time_us) << " us" << std::endl;

    // // Test nanoflann method
    // auto start_time_us_nano =
    //     std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now().time_since_epoch())
    //         .count();

    // std::vector<NearestNeighbor> nearest_neighbors =
    //     findKNearestNeighbors(workspace_points, xyz, 10);

    // auto end_time_us =
    //     std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now().time_since_epoch())
    //         .count();
    // std::cout << "Time taken for nanoflann: "
    //           << (end_time_us - start_time_us_nano) << " us" << std::endl;

    // Test voxel grid method (if available)
    std::vector<NearestNeighbor> nearest_neighbors_voxel;
    if (voxel_grid.numCells > 0) {
        auto start_time_us_voxel =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count();

        nearest_neighbors_voxel =
            findKNearestNeighborsVoxel(workspace_points, voxel_grid, xyz, 10);

        auto end_time_us_voxel =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count();
        std::cout << "Time taken for voxel grid: "
                  << (end_time_us_voxel - start_time_us_voxel) << " us"
                  << std::endl;
    }

    // std::cout << "10 nearest neighbors found (nanoflann):" << std::endl;
    // for (size_t i = 0; i < nearest_neighbors.size(); ++i) {
    //     const auto &neighbor = nearest_neighbors[i];
    //     std::cout << "  " << (i + 1) << ". Distance: " << neighbor.distance
    //               << "m, Position: [" << neighbor.point->position.transpose()
    //               << "]"
    //               << ", Joints: [" <<
    //               neighbor.point->joint_config.transpose()
    //               << "]" << std::endl;
    // }
    // std::cout << std::endl;

    // Show voxel grid results if available
    if (!nearest_neighbors_voxel.empty()) {
        std::cout << "10 nearest neighbors found (voxel grid):" << std::endl;
        for (size_t i = 0; i < nearest_neighbors_voxel.size(); ++i) {
            const auto &neighbor = nearest_neighbors_voxel[i];
            std::cout << "  " << (i + 1) << ". Distance: " << neighbor.distance
                      << "m, Position: ["
                      << neighbor.point->position.transpose() << "]"
                      << ", Joints: ["
                      << neighbor.point->joint_config.transpose() << "]"
                      << std::endl;
        }
        std::cout << std::endl;
    }

    // Time the best initial configuration selection
    auto start_time_config_selection_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();

    // Pick the best initial configuration for IK warm-start
    JointConfig q_init = IKModel::NEUTRAL_JOINT_CONFIG;

    if (!nearest_neighbors_voxel.empty()) {
        // Use voxel grid results for warm-start if available
        std::cout
            << "Selecting best initial configuration from voxel grid results..."
            << std::endl;

        // Find the best neighbor by full pose error (position + rotation)
        size_t bestIdx = nearest_neighbors_voxel[0].index;
        double bestPoseErr = std::numeric_limits<double>::infinity();

        for (const auto &neighbor : nearest_neighbors_voxel) {
            const auto &wp = workspace_points[neighbor.index];

            // Geodesic rotation error
            double ang = 2.0 * std::acos(std::abs(wp.rotation.dot(quat)));

            if (ang < bestPoseErr) {
                bestPoseErr = ang;
                bestIdx = neighbor.index;
            }
        }

        q_init = workspace_points[bestIdx].joint_config;
        std::cout << "Selected initial configuration with rotation error: "
                  << bestPoseErr << " radians" << std::endl;
        std::cout << "Initial joint config: [" << q_init.transpose() << "]"
                  << std::endl;
    }
    //     else if (!nearest_neighbors.empty()) {
    //         // Fall back to nanoflann results
    //         std::cout << "Using nanoflann results for warm-start..." <<
    //         std::endl; q_init = nearest_neighbors[0].point->joint_config;
    //         std::cout << "Initial joint config: [" << q_init.transpose() <<
    //         "]"
    //                   << std::endl;
    // }
    else {
        std::cout
            << "No neighbors found, using neutral configuration as warm-start."
            << std::endl;
    }

    auto end_time_config_selection_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    std::cout << "Time for configuration selection: "
              << (end_time_config_selection_us - start_time_config_selection_us)
              << " us" << std::endl;

    // Sanity check: if the closest neighbor is too far, warn the user
    if (!nearest_neighbors_voxel.empty() &&
        nearest_neighbors_voxel[0].distance > 0.1) {
        std::cout << "WARNING: Closest neighbor is "
                  << nearest_neighbors_voxel[0].distance
                  << "m away. Target may be unreachable." << std::endl;
    }

    // Time the solve_ik function
    auto start_time_solve_ik_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();

    // Solve IK with warm-start
    solve_ik(ik_model, target_pose_xyzquat, &q_init);

    auto end_time_solve_ik_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    std::cout << "Time for solve_ik: "
              << (end_time_solve_ik_us - start_time_solve_ik_us) << " us ("
              << (end_time_solve_ik_us - start_time_solve_ik_us) / 1000.0
              << " ms)" << std::endl;

    // End timing and print total time
    auto end_time_total_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    std::cout << "\nTotal time for IK solving process: "
              << (end_time_total_us - start_time_total_us) << " us ("
              << (end_time_total_us - start_time_total_us) / 1000.0 << " ms)"
              << std::endl;

    auto start_time_solve_ik_no_warmstart_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    ik_model.setCurrentJointConfig(IKModel::NEUTRAL_JOINT_CONFIG);
    solve_ik(ik_model, target_pose_xyzquat);

    auto end_time_solve_ik_no_warmstart_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    std::cout << "Time for solve_ik (no warm-start): "
              << (end_time_solve_ik_no_warmstart_us -
                  start_time_solve_ik_no_warmstart_us)
              << " us ("
              << (end_time_solve_ik_no_warmstart_us -
                  start_time_solve_ik_no_warmstart_us) /
                     1000.0
              << " ms)" << std::endl;

    // Test DLS solver with warm start
    auto start_time_solve_ik_dls_warmstart_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();

    ik_model.setCurrentJointConfig(IKModel::NEUTRAL_JOINT_CONFIG);

    // Convert target pose to SE3 format for DLS solver
    pinocchio::SE3 target_pose_se3 = pinocchio::SE3::Identity();
    target_pose_se3.translation() = target_pose_xyzquat.head<3>();
    target_pose_se3.rotation() =
        Eigen::Quaterniond(target_pose_xyzquat[6], target_pose_xyzquat[3],
                           target_pose_xyzquat[4], target_pose_xyzquat[5])
            .toRotationMatrix();

    solve_ik_dls(ik_model, target_pose_se3, &q_init);

    auto end_time_solve_ik_dls_warmstart_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    std::cout << "Time for solve_ik_dls (with warm-start): "
              << (end_time_solve_ik_dls_warmstart_us -
                  start_time_solve_ik_dls_warmstart_us)
              << " us ("
              << (end_time_solve_ik_dls_warmstart_us -
                  start_time_solve_ik_dls_warmstart_us) /
                     1000.0
              << " ms)" << std::endl;

    return 0;
}
