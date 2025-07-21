#include "ik_model.h"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <Eigen/Geometry>
#include <boost/random/sobol.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <algorithm>
#include <mutex>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <vector>

const std::string robots_model_path =
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/";
const std::string urdf_filename =
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf";
const std::string srdf_filename =
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.srdf";
const std::string end_effector_name = "connection_frame";

const long unsigned int num_points = 100000;
const int batch_size =
    1000; // Process samples in batches for better cache locality

using JointConfig = Eigen::Vector<double, 6>;

// Hash function for JointConfig to use with unordered_set (O(1) lookup)
struct JointConfigHash {
    std::size_t operator()(const JointConfig &config) const {
        std::size_t hash = 0;
        for (int i = 0; i < config.size(); ++i) {
            // Simple hash combination
            hash ^= std::hash<double>{}(config[i]) + 0x9e3779b9 + (hash << 6) +
                    (hash >> 2);
        }
        return hash;
    }
};

// Equality function for JointConfig to use with unordered_set
struct JointConfigEqual {
    bool operator()(const JointConfig &lhs, const JointConfig &rhs) const {
        const double tolerance = 1e-3;
        for (int i = 0; i < lhs.size(); ++i) {
            if (std::abs(lhs[i] - rhs[i]) > tolerance) {
                return false;
            }
        }
        return true;
    }
};

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

// Optimized function to process a single sample without duplicate checking
bool processSampleFast(const pinocchio::Model &model, pinocchio::Data &data,
                       const pinocchio::GeometryModel &geom_model,
                       pinocchio::GeometryData &geom_data,
                       const Eigen::VectorXd &joint_limits_lower,
                       const Eigen::VectorXd &joint_limits_upper,
                       const int frame_id, const std::vector<double> &u,
                       JointConfig &q, WorkspacePoint &result,
                       const double ground_threshold = 0.05) {

    // Safety checks
    if (frame_id < 0 || frame_id >= model.frames.size()) {
        return false;
    }

    if (u.size() != model.nq) {
        return false;
    }

    // Map Sobol sequence values to joint configuration with bounds checking
    for (int j = 0; j < model.nq; ++j) {
        double clamped_u = std::max(0.0, std::min(1.0, u[j]));
        q[j] = joint_limits_lower[j] +
               (joint_limits_upper[j] - joint_limits_lower[j]) * clamped_u;

        if (q[j] < joint_limits_lower[j] || q[j] > joint_limits_upper[j]) {
            return false;
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

        if (!collision_detected) {
            // Check that frames from link2 onwards are above ground
            bool frames_above_ground = true;
            const int link2_frame_id = model.getFrameId("link_2");

            if (link2_frame_id >= 0) {
                for (pinocchio::FrameIndex i = link2_frame_id;
                     i < model.nframes; ++i) {
                    if (data.oMf[i].translation()[2] <= ground_threshold) {
                        frames_above_ground = false;
                        break;
                    }
                }
            } else {
                if (data.oMf[frame_id].translation()[2] <= ground_threshold) {
                    frames_above_ground = false;
                }
            }

            if (frames_above_ground) {
                Eigen::Vector3d position = data.oMf[frame_id].translation();
                Eigen::Matrix3d rotation_matrix = data.oMf[frame_id].rotation();
                Eigen::Quaterniond rotation(rotation_matrix);
                rotation.normalize();

                result = WorkspacePoint(position, rotation, q);
                return true;
            }
        }
    } catch (...) {
        return false;
    }

    return false;
}

// Batch processing function for better cache locality
std::vector<WorkspacePoint>
processBatch(const pinocchio::Model &model,
             const pinocchio::GeometryModel &geom_model,
             const Eigen::VectorXd &joint_limits_lower,
             const Eigen::VectorXd &joint_limits_upper, const int frame_id,
             std::mt19937 &gen,
             std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
                 &existing_configs,
             const int batch_size) {

    std::vector<WorkspacePoint> batch_results;
    batch_results.reserve(batch_size / 10); // Conservative estimate

    // Thread-local Pinocchio data structures
    pinocchio::Data data_local(model);
    pinocchio::GeometryData geom_data_local(geom_model);

    std::uniform_real_distribution<double> dis(0.0, 1.0);
    std::vector<double> u(model.nq);
    JointConfig q_local;
    WorkspacePoint result(Eigen::Vector3d::Zero(),
                          Eigen::Quaterniond::Identity(), JointConfig::Zero());

    int attempts = 0;
    const int max_attempts_per_sample =
        10; // Limit attempts to avoid infinite loops

    for (int i = 0; i < batch_size; ++i) {
        attempts = 0;
        bool valid_config_found = false;

        // Keep trying until we find a unique configuration or hit max attempts
        while (attempts < max_attempts_per_sample && !valid_config_found) {
            // Generate random values
            for (int j = 0; j < model.nq; ++j) {
                u[j] = dis(gen);
            }

            // Map to joint configuration
            for (int j = 0; j < model.nq; ++j) {
                double clamped_u = std::max(0.0, std::min(1.0, u[j]));
                q_local[j] =
                    joint_limits_lower[j] +
                    (joint_limits_upper[j] - joint_limits_lower[j]) * clamped_u;
            }

            // Check if configuration already exists
            if (existing_configs.find(q_local) != existing_configs.end()) {
                attempts++;
                continue; // Try again with new random values
            }

            // Configuration is unique, process it
            if (processSampleFast(model, data_local, geom_model,
                                  geom_data_local, joint_limits_lower,
                                  joint_limits_upper, frame_id, u, q_local,
                                  result)) {
                batch_results.push_back(result);
                existing_configs.insert(q_local);
                valid_config_found = true;
            } else {
                // Configuration is unique but failed processing (collision,
                // etc.) Still mark it as seen to avoid infinite loops
                existing_configs.insert(q_local);
                attempts++;
            }
        }

        // If we couldn't find a valid unique configuration after max attempts,
        // we'll skip this iteration and continue with the next one
        if (!valid_config_found) {
            // This could happen if the workspace is very constrained
            // We'll just continue to the next iteration
        }
    }

    return batch_results;
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
        float x = static_cast<float>(point.position[0]);
        float y = static_cast<float>(point.position[1]);
        float z = static_cast<float>(point.position[2]);
        ply_file.write(reinterpret_cast<const char *>(&x), sizeof(float));
        ply_file.write(reinterpret_cast<const char *>(&y), sizeof(float));
        ply_file.write(reinterpret_cast<const char *>(&z), sizeof(float));

        for (int j = 0; j < point.joint_config.size(); ++j) {
            float joint_val = static_cast<float>(point.joint_config[j]);
            ply_file.write(reinterpret_cast<const char *>(&joint_val),
                           sizeof(float));
        }
    }

    ply_file.close();
    std::cout << "Workspace points saved to: " << filename << std::endl;
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
    if (!std::filesystem::exists(base_dir)) {
        std::filesystem::create_directory(base_dir);
    }

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

// Helper function to build and save sparse 3D voxel grid
void buildAndSaveVoxelGrid(const std::vector<WorkspacePoint> &workspace_points,
                           const std::string &output_dir) {
    if (workspace_points.empty()) {
        std::cerr << "No workspace points to build voxel grid from."
                  << std::endl;
        return;
    }

    std::cout << "Building sparse 3D voxel grid from "
              << workspace_points.size() << " workspace points..." << std::endl;

    // A. Quantize point-cloud into a 3D grid
    // 1. Compute axis-aligned bounding box
    Eigen::Vector3d minB(+1e9, +1e9, +1e9), maxB(-1e9, -1e9, -1e9);
    for (const auto &wp : workspace_points) {
        minB = minB.cwiseMin(wp.position);
        maxB = maxB.cwiseMax(wp.position);
    }

    std::cout << "Bounding box: min=[" << minB.transpose() << "], max=["
              << maxB.transpose() << "]" << std::endl;

    // 2. Choose a voxel size (e.g. 1 cm)
    double r = 0.01; // 1cm voxel size

    // 3. Compute grid dimensions
    Eigen::Vector3i dims = ((maxB - minB) / r).cast<int>().array() + 1;

    std::cout << "Grid dimensions: [" << dims.transpose() << "]" << std::endl;

    // B. Bin each sample into its grid cell and collect sparse data
    auto flatIndex = [&](const Eigen::Vector3i &idx) {
        return static_cast<size_t>(idx.x()) * dims.y() * dims.z() +
               static_cast<size_t>(idx.y()) * dims.z() +
               static_cast<size_t>(idx.z());
    };

    // Use a map to store only non-empty cells
    std::map<size_t, std::vector<size_t>> sparseGrid;

    for (size_t i = 0; i < workspace_points.size(); ++i) {
        Eigen::Vector3i idx = ((workspace_points[i].position - minB) / r)
                                  .cast<int>()
                                  .cwiseMax(Eigen::Vector3i::Zero())
                                  .cwiseMin(dims - Eigen::Vector3i::Ones());
        size_t cellIndex = flatIndex(idx);
        sparseGrid[cellIndex].push_back(i);
    }

    // Count non-empty cells
    size_t nonEmptyCells = sparseGrid.size();
    std::cout << "Non-empty cells: " << nonEmptyCells << std::endl;

    // C. Create sparse nearest neighbor data with limited BFS
    std::map<size_t, int> sparseNearestIdx;
    std::queue<std::pair<size_t, int>> q; // (cell_index, distance)
    const int maxDistance =
        3; // Only fill cells within 3 voxels of workspace points

    // Seed the queue with every non-empty cell
    for (const auto &cell : sparseGrid) {
        sparseNearestIdx[cell.first] = static_cast<int>(
            cell.second[0]);     // Pick first sample as representative
        q.push({cell.first, 0}); // Distance 0 for actual workspace cells
    }

    // BFS flood-fill with distance limit
    // 6-connected neighbors
    const int offs[6][3] = {{+1, 0, 0}, {-1, 0, 0}, {0, +1, 0},
                            {0, -1, 0}, {0, 0, +1}, {0, 0, -1}};

    while (!q.empty()) {
        auto [c, distance] = q.front();
        q.pop();

        // Stop if we've reached the maximum distance
        if (distance >= maxDistance) {
            continue;
        }

        // Decode c → (x,y,z)
        int x = static_cast<int>(c / (dims.y() * dims.z()));
        int y = static_cast<int>((c / dims.z()) % dims.y());
        int z = static_cast<int>(c % dims.z());

        for (const auto &o : offs) {
            int nx = x + o[0], ny = y + o[1], nz = z + o[2];
            if (nx < 0 || ny < 0 || nz < 0 || nx >= dims.x() ||
                ny >= dims.y() || nz >= dims.z()) {
                continue;
            }

            size_t nc = static_cast<size_t>(nx) * dims.y() * dims.z() +
                        static_cast<size_t>(ny) * dims.z() +
                        static_cast<size_t>(nz);

            if (sparseNearestIdx.find(nc) == sparseNearestIdx.end()) {
                // Inherit the same representative point
                sparseNearestIdx[nc] = sparseNearestIdx[c];
                q.push({nc, distance + 1});
            }
        }
    }

    // Count filled cells
    size_t filledCells = sparseNearestIdx.size();
    std::cout << "Filled cells after limited BFS: " << filledCells << std::endl;

    // D. Serialize sparse data for runtime
    std::string grid_filename =
        getTimestampedFilename(output_dir, "workspace_grid_sparse", ".bin");
    std::ofstream out(grid_filename, std::ios::binary);

    if (!out.is_open()) {
        std::cerr << "Error: Could not open file " << grid_filename
                  << " for writing." << std::endl;
        return;
    }

    // 1) Header: minB(3×double), r(double), dims(3×int), maxDistance(int),
    // numCells(size_t)
    out.write(reinterpret_cast<const char *>(minB.data()), sizeof(double) * 3);
    out.write(reinterpret_cast<const char *>(&r), sizeof(double));
    out.write(reinterpret_cast<const char *>(dims.data()), sizeof(int) * 3);
    out.write(reinterpret_cast<const char *>(&maxDistance), sizeof(int));

    size_t numCells = sparseNearestIdx.size();
    out.write(reinterpret_cast<const char *>(&numCells), sizeof(size_t));

    // 2) Sparse data: (cell_index, nearest_point_index) pairs
    for (const auto &cell : sparseNearestIdx) {
        out.write(reinterpret_cast<const char *>(&cell.first), sizeof(size_t));
        out.write(reinterpret_cast<const char *>(&cell.second), sizeof(int));
    }

    out.close();

    std::cout << "Sparse voxel grid saved to: " << grid_filename << std::endl;
    std::cout << "Grid parameters: voxel_size=" << r << "m, "
              << "dimensions=[" << dims.transpose() << "], "
              << "max_distance=" << maxDistance << " voxels, "
              << "total_cells=" << numCells << std::endl;
    std::cout << "Memory usage: " << (numCells * (sizeof(size_t) + sizeof(int)))
              << " bytes (vs " << (dims.x() * dims.y() * dims.z() * sizeof(int))
              << " bytes for dense grid)" << std::endl;
}

int main() {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (!std::filesystem::exists(urdf_filename)) {
        std::cerr << "ERROR: URDF file not found: " << urdf_filename
                  << std::endl;
        return -1;
    }
    if (!std::filesystem::exists(srdf_filename)) {
        std::cerr << "ERROR: SRDF file not found: " << srdf_filename
                  << std::endl;
        return -1;
    }

    using namespace pinocchio;

    // Load the robot model
    Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    std::cout << "Loaded model with " << model.nq << " joints" << std::endl;

    // Build the data associated to the model
    Data data(model);

    // Set joint limits
    const auto joint_limits_lower = model.lowerPositionLimit.cast<double>();
    const auto joint_limits_upper = model.upperPositionLimit.cast<double>();

    std::cout << "Model has " << model.nq << " joints" << std::endl;
    std::cout << "Joint limits lower: " << joint_limits_lower.transpose()
              << std::endl;
    std::cout << "Joint limits upper: " << joint_limits_upper.transpose()
              << std::endl;

    // Verify that model has exactly 6 joints
    if (model.nq != 6) {
        std::cerr << "ERROR: Expected 6 joints for UR10, but model has "
                  << model.nq << " joints!" << std::endl;
        return -1;
    }

    const int frame_id = model.getFrameId(end_effector_name);

    // Load the geometries
    GeometryModel geom_model;
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION,
                               geom_model, robots_model_path);
    std::cout << "Loaded " << geom_model.ngeoms << " geometry objects"
              << std::endl;
    geom_model.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf_filename);
    GeometryData geom_data(geom_model);

    std::cout << "Generating " << num_points
              << " random joint configurations..." << std::endl;

    // Hash set for O(1) duplicate checking
    std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
        existing_configs;
    std::vector<WorkspacePoint> all_workspace_points;
    all_workspace_points.reserve(num_points / 10); // Conservative estimate

    long unsigned int duplicate_count = 0;
    long unsigned int processed_count = 0;
    long unsigned int total_attempts = 0;

    // Progress reporting interval
    const long unsigned int progress_interval = std::max(1UL, num_points / 100);

    // Random number generator
    std::mt19937 gen(42); // Fixed seed for reproducible results

    std::cout << "Starting optimized batch processing with duplicate retry..."
              << std::endl;

    // Process in batches for better cache locality
    for (long unsigned int batch_start = 0; batch_start < num_points;
         batch_start += batch_size) {
        int current_batch_size =
            std::min(static_cast<int>(batch_size),
                     static_cast<int>(num_points - batch_start));

        auto batch_results = processBatch(model, geom_model, joint_limits_lower,
                                          joint_limits_upper, frame_id, gen,
                                          existing_configs, current_batch_size);

        all_workspace_points.insert(all_workspace_points.end(),
                                    batch_results.begin(), batch_results.end());

        processed_count += current_batch_size;

        // Progress reporting
        if (processed_count % progress_interval < current_batch_size) {
            std::cout << "Progress: " << (processed_count * 100 / num_points)
                      << "% (" << processed_count << "/" << num_points << ")\r";
        }
    }
    std::cout << std::endl;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    std::cout << "\nOptimized processing completed in " << duration.count()
              << " seconds" << std::endl;
    std::cout << "Generated " << all_workspace_points.size()
              << " workspace points." << std::endl;
    std::cout << "Total unique joint configurations: "
              << existing_configs.size() << std::endl;
    std::cout << "Processing rate: " << (num_points / duration.count())
              << " samples/second" << std::endl;
    std::cout << "Success rate: "
              << (all_workspace_points.size() * 100.0 / num_points) << "% ("
              << all_workspace_points.size() << "/" << num_points << ")"
              << std::endl;
    std::cout
        << "Note: Duplicate configurations were retried with new random values"
        << std::endl;

    // Example: Print first few entries
    std::cout << "\nFirst 5 workspace points:" << std::endl;
    int count = 0;
    for (const auto &point : all_workspace_points) {
        if (count >= 5)
            break;
        std::cout << "Position: " << point.position.transpose()
                  << " | Rotation (quat): [" << point.rotation.w() << ", "
                  << point.rotation.x() << ", " << point.rotation.y() << ", "
                  << point.rotation.z() << "]"
                  << " | Joint config: " << point.joint_config.transpose()
                  << std::endl;
        count++;
    }

    // Save workspace points to binary PLY file
    const std::string output_dir = "generated_workspaces";
    std::string ply_filename = getTimestampedFilename(
        output_dir, "workspace_points_optimized", ".ply");
    writeBinaryPLY(ply_filename, all_workspace_points);

    // Save k-d tree data to binary file
    std::string kdtree_filename =
        getTimestampedFilename(output_dir, "kdtree_data_optimized", ".bin");
    writeKDTreeData(kdtree_filename, all_workspace_points);

    // Build and save 3D voxel grid
    buildAndSaveVoxelGrid(all_workspace_points, output_dir);

    return 0;
}