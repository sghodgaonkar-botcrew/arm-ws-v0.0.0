#include "reachable_workspace_generator.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>

// Hash function implementation
std::size_t JointConfigHash::operator()(const JointConfig &config) const {
    std::size_t hash = 0;
    for (int i = 0; i < config.size(); ++i) {
        // Simple hash combination
        hash ^= std::hash<double>{}(config[i]) + 0x9e3779b9 + (hash << 6) +
                (hash >> 2);
    }
    return hash;
}

// Equality function implementation
bool JointConfigEqual::operator()(const JointConfig &lhs,
                                  const JointConfig &rhs) const {
    const double tolerance = 1e-3;
    for (int i = 0; i < lhs.size(); ++i) {
        if (std::abs(lhs[i] - rhs[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

// ReachableWorkspaceGenerator implementation
ReachableWorkspaceGenerator::ReachableWorkspaceGenerator(
    RobotModel &robot_model, long unsigned int num_points, int batch_size,
    double ground_threshold, double joint_tolerance,
    int max_attempts_per_sample, int max_distance)
    : robot_model_(robot_model), random_generator_(42), num_points_(num_points),
      batch_size_(batch_size), ground_threshold_(ground_threshold),
      joint_tolerance_(joint_tolerance),
      max_attempts_per_sample_(max_attempts_per_sample),
      max_distance_(max_distance) {
    std::cout << "Initialized workspace generator with external RobotModel"
              << std::endl;
    std::cout << "Workspace parameters: " << num_points_ << " points, "
              << batch_size_ << " batch size, " << ground_threshold_
              << " ground threshold, " << max_distance_ << " max distance"
              << std::endl;
}

ReachableWorkspaceGenerator::~ReachableWorkspaceGenerator() {
    // No manual cleanup needed - RobotModel handles its own resources
}

bool ReachableWorkspaceGenerator::processSampleFast(const JointConfig &q,
                                                    WorkspacePoint &result) {
    try {
        // Use RobotModel to check if configuration is valid
        if (!robot_model_.isValidConfiguration(q, ground_threshold_)) {
            return false;
        }

        // Compute forward kinematics using RobotModel
        pinocchio::SE3 end_effector_pose =
            robot_model_.computeForwardKinematics(q);

        // Extract position and rotation
        Eigen::Vector3d position = end_effector_pose.translation();
        Eigen::Matrix3d rotation_matrix = end_effector_pose.rotation();
        Eigen::Quaterniond rotation(rotation_matrix);
        rotation.normalize();

        result = WorkspacePoint(position, rotation, q);
        return true;
    } catch (const std::exception &e) {
        std::cerr << "Error processing sample: " << e.what() << std::endl;
        return false;
    }
}

std::vector<WorkspacePoint> ReachableWorkspaceGenerator::processBatch(
    std::mt19937 &gen,
    std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
        &existing_configs,
    const int batch_size) {

    std::vector<WorkspacePoint> batch_results;
    batch_results.reserve(batch_size / 10); // Conservative estimate

    std::uniform_real_distribution<double> dis(0.0, 1.0);
    JointConfig q_local;
    WorkspacePoint result(Eigen::Vector3d::Zero(),
                          Eigen::Quaterniond::Identity(), JointConfig::Zero());

    int attempts = 0;
    const int max_attempts_per_sample = max_attempts_per_sample_;

    for (int i = 0; i < batch_size; ++i) {
        attempts = 0;
        bool valid_config_found = false;

        // Keep trying until we find a unique configuration or hit max attempts
        while (attempts < max_attempts_per_sample && !valid_config_found) {
            // Generate random joint configuration
            for (int j = 0; j < q_local.size(); ++j) {
                double u = dis(gen);
                double clamped_u = std::max(0.0, std::min(1.0, u));
                q_local[j] = robot_model_.getJointLimitsLower()[j] +
                             (robot_model_.getJointLimitsUpper()[j] -
                              robot_model_.getJointLimitsLower()[j]) *
                                 clamped_u;
            }

            // Check if configuration already exists
            if (existing_configs.find(q_local) != existing_configs.end()) {
                attempts++;
                continue; // Try again with new random values
            }

            // Configuration is unique, process it
            if (processSampleFast(q_local, result)) {
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
    }

    return batch_results;
}

std::vector<WorkspacePoint> ReachableWorkspaceGenerator::generateWorkspace() {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "Generating " << num_points_
              << " random joint configurations..." << std::endl;

    // Hash set for O(1) duplicate checking
    std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
        existing_configs;
    std::vector<WorkspacePoint> all_workspace_points;
    all_workspace_points.reserve(num_points_ / 10); // Conservative estimate

    long unsigned int processed_count = 0;

    // Progress reporting interval
    const long unsigned int progress_interval =
        std::max(1UL, num_points_ / 100);

    std::cout << "Starting optimized batch processing with duplicate retry..."
              << std::endl;

    // Process in batches for better cache locality
    for (long unsigned int batch_start = 0; batch_start < num_points_;
         batch_start += batch_size_) {
        int current_batch_size =
            std::min(static_cast<int>(batch_size_),
                     static_cast<int>(num_points_ - batch_start));

        auto batch_results = processBatch(random_generator_, existing_configs,
                                          current_batch_size);

        all_workspace_points.insert(all_workspace_points.end(),
                                    batch_results.begin(), batch_results.end());

        processed_count += current_batch_size;

        // Progress reporting
        if (processed_count % progress_interval < current_batch_size) {
            std::cout << "Progress: " << (processed_count * 100 / num_points_)
                      << "% (" << processed_count << "/" << num_points_ << ")"
                      << std::endl;
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
    std::cout << "Processing rate: " << (num_points_ / duration.count())
              << " samples/second" << std::endl;
    std::cout << "Success rate: "
              << (all_workspace_points.size() * 100.0 / num_points_) << "% ("
              << all_workspace_points.size() << "/" << num_points_ << ")"
              << std::endl;
    std::cout
        << "Note: Duplicate configurations were retried with new random values"
        << std::endl;

    // Example: Print first few entries
    // std::cout << "\nFirst 5 workspace points:" << std::endl;
    // int count = 0;
    // for (const auto &point : all_workspace_points) {
    //     if (count >= 5)
    //         break;
    //     std::cout << "Position: " << point.position.transpose()
    //               << " | Rotation (quat): [" << point.rotation.w() << ", "
    //               << point.rotation.x() << ", " << point.rotation.y() << ", "
    //               << point.rotation.z() << "]"
    //               << " | Joint config: " << point.joint_config.transpose()
    //               << std::endl;
    //     count++;
    // }

    return all_workspace_points;
}

void ReachableWorkspaceGenerator::generateAndSaveWorkspace(
    const std::string &output_dir) {
    // Generate the workspace
    std::cout << "Generating and saving workspace..." << std::endl;
    auto workspace_points = generateWorkspace();

    // Generate timestamp once for all files
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &now_c);
#else
    localtime_r(&now_c, &tm_buf);
#endif
    std::ostringstream timestamp_oss;
    timestamp_oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    std::string timestamp = timestamp_oss.str();

    // Create output directory if it doesn't exist
    if (!std::filesystem::exists(output_dir)) {
        std::filesystem::create_directory(output_dir);
    }

    // Save workspace points to binary PLY file
    std::string ply_filename =
        output_dir + "/workspace_points_optimized_" + timestamp + ".ply";
    writeBinaryPLY(ply_filename, workspace_points);

    // Save workspace points to ASCII PLY file (human-readable)
    std::string ascii_ply_filename =
        output_dir + "/workspace_points_ascii_" + timestamp + ".ply";
    writeAsciiPLY(ascii_ply_filename, workspace_points);

    // Save k-d tree data to binary file
    std::string kdtree_filename =
        output_dir + "/kdtree_data_optimized_" + timestamp + ".bin";
    writeKDTreeData(kdtree_filename, workspace_points);

    // Build and save 3D voxel grid
    ReachableWorkspaceGenerator::buildAndSaveVoxelGrid(workspace_points,
                                                       output_dir, timestamp);

    std::cout << "Workspace generation and saving completed successfully!"
              << std::endl;
    std::cout << "All files saved to: " << output_dir << std::endl;
    std::cout << "Timestamp used: " << timestamp << std::endl;
}

void ReachableWorkspaceGenerator::writeBinaryPLY(
    const std::string &filename,
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

void ReachableWorkspaceGenerator::writeAsciiPLY(
    const std::string &filename,
    const std::vector<WorkspacePoint> &workspace_points) {
    std::ofstream ply_file(filename, std::ios::out);

    if (!ply_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename
                  << " for writing." << std::endl;
        return;
    }

    // Write PLY header
    ply_file << "ply" << std::endl;
    ply_file << "format ascii 1.0" << std::endl;
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

    // Write ASCII data
    for (const auto &point : workspace_points) {
        float x = static_cast<float>(point.position[0]);
        float y = static_cast<float>(point.position[1]);
        float z = static_cast<float>(point.position[2]);

        ply_file << x << " " << y << " " << z;

        // Write joint configuration values
        for (int j = 0; j < point.joint_config.size(); ++j) {
            float joint_val = static_cast<float>(point.joint_config[j]);
            ply_file << " " << joint_val;
        }

        ply_file << std::endl;
    }

    ply_file.close();
    std::cout << "Workspace points saved to ASCII PLY: " << filename
              << std::endl;
}

void ReachableWorkspaceGenerator::writeKDTreeData(
    const std::string &filename,
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

// Static version with timestamp parameter
void ReachableWorkspaceGenerator::buildAndSaveVoxelGrid(
    const std::vector<WorkspacePoint> &workspace_points,
    const std::string &output_dir, const std::string &timestamp) {
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
    int maxDistance = 5; // Default max distance for static version

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
        output_dir + "/workspace_grid_sparse_" + timestamp + ".bin";
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

std::string ReachableWorkspaceGenerator::getTimestampedFilename(
    const std::string &base_dir, const std::string &prefix,
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
