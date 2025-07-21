#ifndef WORKSPACE_TYPES_H
#define WORKSPACE_TYPES_H

#include <Eigen/Geometry>
#include <map>
#include <vector>

// Type aliases for fixed-size vectors
using JointConfig = Eigen::Vector<double, 6>;
using XYZQuat = Eigen::Vector<double, 7>;

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

#endif // WORKSPACE_TYPES_H