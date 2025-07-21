/**
 * @file workspace_types.h
 * @brief Workspace data types and structures for IK solver
 *
 * This header file defines the fundamental data structures used by the
 * IK solver with reachable workspace support. It includes:
 * - Type aliases for fixed-size vectors
 * - WorkspacePoint structure for storing sampled workspace data
 * - NearestNeighbor structure for search results
 * - VoxelGrid structure for spatial indexing
 *
 * These types are used throughout the IK solver to represent workspace
 * data, search results, and spatial indexing structures.
 *
 * @author Shantanu Ghodgaonkar
 * @date 2025-07-21
 * @version 1.0
 */

#ifndef WORKSPACE_TYPES_H
#define WORKSPACE_TYPES_H

#include <Eigen/Geometry>
#include <map>
#include <vector>

/**
 * @brief Type alias for 6-dimensional joint configuration vector
 *
 * This type represents a joint configuration for a 6-DOF robot arm.
 * The vector contains 6 joint angles in radians, typically in the range [-π,
 * π].
 *
 * @see XYZQuat
 */
using JointConfig = Eigen::Vector<double, 6>;

/**
 * @brief Type alias for 7-dimensional pose vector [x, y, z, qx, qy, qz, qw]
 *
 * This type represents a 3D pose as position (x, y, z) and quaternion (qx, qy,
 * qz, qw). The quaternion represents rotation and should be normalized to unit
 * length.
 *
 * **Format:**
 * - Elements 0-2: Position (x, y, z) in meters
 * - Elements 3-6: Quaternion (qx, qy, qz, qw) for rotation
 *
 * @see JointConfig
 */
using XYZQuat = Eigen::Vector<double, 7>;

/**
 * @brief Structure to hold workspace point data with position, rotation, and
 * joint config
 *
 * This structure represents a single point in the robot's reachable workspace.
 * Each point contains:
 * - Position: 3D Cartesian coordinates of the end effector
 * - Rotation: Quaternion representing end effector orientation
 * - Joint configuration: Joint angles that achieve this pose
 *
 * **Usage:**
 * - Used to store pre-computed workspace samples
 * - Enables fast nearest neighbor search for IK warm-start
 * - Provides mapping from workspace poses to joint configurations
 *
 * **Data Integrity:**
 * - Quaternion should be normalized to unit length
 * - Joint angles should be in valid ranges
 * - Position should be within robot's reachable workspace
 */
struct WorkspacePoint {
    Eigen::Vector3d position; ///< 3D position of end effector in meters
    Eigen::Quaterniond
        rotation; ///< Quaternion representing end effector orientation
    JointConfig joint_config; ///< Joint configuration that achieves this pose

    /**
     * @brief Constructor
     * @param pos 3D position vector
     * @param rot Quaternion for rotation (will be normalized)
     * @param config Joint configuration vector
     *
     * The constructor automatically normalizes the quaternion to ensure
     * numerical stability and proper rotation representation.
     */
    WorkspacePoint(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot,
                   const JointConfig &config)
        : position(pos), rotation(rot), joint_config(config) {
        rotation.normalize(); // Ensure unit quaternion
    }
};

/**
 * @brief Structure to hold nearest neighbor search results
 *
 * This structure represents the result of a nearest neighbor search,
 * containing information about a nearby workspace point and its
 * distance to the query point.
 *
 * **Usage:**
 * - Return type for nearest neighbor search functions
 * - Used for IK warm-start selection
 * - Enables sorting and filtering of search results
 *
 * **Distance Metric:**
 * - Distance is typically Euclidean distance in 3D space
 * - Can be extended to include rotation distance
 * - Used for selecting best initial configuration for IK
 */
struct NearestNeighbor {
    std::size_t index; ///< Index of the neighbor in workspace points array
    double distance;   ///< Distance to the query point
    const WorkspacePoint *point; ///< Pointer to the workspace point

    /**
     * @brief Constructor
     * @param idx Index of the neighbor
     * @param dist Distance to query point
     * @param pt Pointer to workspace point
     */
    NearestNeighbor(std::size_t idx, double dist, const WorkspacePoint *pt)
        : index(idx), distance(dist), point(pt) {}

    /**
     * @brief Comparison operator for sorting (closest first)
     * @param other Another nearest neighbor result
     * @return true if this neighbor is closer than the other
     *
     * This operator enables sorting of search results by distance,
     * with closest neighbors appearing first.
     */
    bool operator<(const NearestNeighbor &other) const {
        return distance < other.distance;
    }
};

/**
 * @brief Structure to hold sparse voxel grid data for spatial indexing
 *
 * This structure implements a sparse voxel grid for efficient spatial
 * indexing of workspace points. It provides O(1) spatial lookup for
 * nearest neighbor search.
 *
 * **Grid Structure:**
 * - Regular 3D grid with specified resolution
 * - Only stores occupied cells (sparse representation)
 * - Maps cell indices to nearest workspace point indices
 *
 * **Spatial Indexing:**
 * - Converts 3D positions to grid cell indices
 * - Enables fast neighbor search using "onion ring" expansion
 * - Reduces search complexity from O(n) to O(k) where k is number of neighbors
 *
 * **Memory Efficiency:**
 * - Only stores occupied cells to save memory
 * - Uses std::map for sparse storage
 * - Scales well with workspace size
 */
struct VoxelGrid {
    Eigen::Vector3d minB; ///< Minimum bounds of the grid (3D point)
    double r;             ///< Resolution (voxel size) in meters
    Eigen::Vector3i
        dims;        ///< Grid dimensions (number of cells in each dimension)
    int maxDistance; ///< Maximum search distance for neighbor finding
    size_t numCells; ///< Number of occupied cells in the grid
    std::map<size_t, int>
        sparseNearestIdx; ///< Map from cell index to nearest point index

    /**
     * @brief Default constructor
     *
     * Initializes all members to safe default values:
     * - r = 0.0 (invalid resolution)
     * - maxDistance = 0 (no search distance)
     * - numCells = 0 (empty grid)
     * - Other members default to zero vectors
     */
    VoxelGrid() : r(0.0), maxDistance(0), numCells(0) {}
};

#endif // WORKSPACE_TYPES_H