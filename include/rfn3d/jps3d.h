#ifndef RFN3D_JPS3D_H
#define RFN3D_JPS3D_H

#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <gcopter/voxel_map.hpp>

// 3D Jump Point Search front-end over a dilated voxel_map::VoxelMap.
//
// The search core (neighbor / forced-neighbor tables, jump, successor rules) is
// adapted from KumarRobotics/jps3d (BSD 3-Clause, Copyright (c) 2017 sikang).
// Differences from upstream: no boost (std::priority_queue with lazy deletion),
// node bookkeeping is a hash map keyed by voxel id so per-plan memory scales
// with the nodes the search touches rather than the grid size, and the
// straight-line part of jump() is a loop instead of tail recursion.
//
// Any non-zero voxel (Occupied or Dilated) is blocked, and voxels outside the
// map are blocked for the search. The map is expected to be a local window
// around the vehicle, so a goal outside it is clipped to the window edge.
class JPS3D
{
public:
    // Plan from start to goal (world coordinates) on `map`. On success `path`
    // holds line-of-sight-simplified waypoints from `start`; it ends at `goal`
    // when the goal is inside the map or reachable in a straight line from the
    // clipped window-edge goal, and at the window edge otherwise. Returns false
    // if start is outside the map or blocked, or no path exists.
    bool plan(const voxel_map::VoxelMap &map,
              const Eigen::Vector3d &start,
              const Eigen::Vector3d &goal,
              std::vector<Eigen::Vector3d> &path);

    // Jump points of the last successful search, start -> goal, as voxel indices.
    const std::vector<Eigen::Vector3i> &getJumpCells() const { return _cells; }

private:
    // For each direction id (dx+1)+3(dy+1)+9(dz+1):
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    struct Neib
    {
        Neib();
        int ns[27][3][26];
        int f1[27][3][12];
        int f2[27][3][12];
        // {always-added, forced-candidate} counts per move norm1:
        // none (start node), straight, 2D diagonal, 3D diagonal.
        static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};

    private:
        static void neib(int dx, int dy, int dz, int norm1, int dev,
                         int &tx, int &ty, int &tz);
        static void fneib(int dx, int dy, int dz, int norm1, int dev,
                          int &fx, int &fy, int &fz, int &nx, int &ny, int &nz);
    };

    struct Node
    {
        int x, y, z;
        int dx, dy, dz; // direction of arrival, each in {-1, 0, 1}
        int parent = -1;
        double g = 0.0;
        bool closed = false;
    };

    bool search(const Eigen::Vector3i &start, const Eigen::Vector3i &goal);
    void getSucc(const Node &curr, std::vector<int> &succ_ids, std::vector<double> &succ_costs);
    bool jump(int x, int y, int z, int dx, int dy, int dz, int &nx, int &ny, int &nz) const;
    bool hasForced(int x, int y, int z, int id, int norm1) const;

    // Clip the start->goal segment to the map and return a free voxel at (or
    // near) the exit point. Returns false if no free voxel is close by.
    bool clipGoal(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                  Eigen::Vector3i &goal_id) const;
    // Exact voxel-traversal segment test; outside the map counts as free.
    bool lineFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;

    inline int toId(int x, int y, int z) const { return x + y * _sx + z * _sx * _sy; }
    inline bool inMap(int x, int y, int z) const
    {
        return x >= 0 && x < _sx && y >= 0 && y < _sy && z >= 0 && z < _sz;
    }
    inline bool isFree(int x, int y, int z) const
    {
        return inMap(x, y, z) && (*_voxels)[toId(x, y, z)] == voxel_map::Unoccupied;
    }
    // Out-of-map is neither free nor occupied, so map edges never force neighbors.
    inline bool isOccupied(int x, int y, int z) const
    {
        return inMap(x, y, z) && (*_voxels)[toId(x, y, z)] != voxel_map::Unoccupied;
    }
    double heur(int x, int y, int z) const;

    static const Neib &neibTable();

    const voxel_map::VoxelMap *_map = nullptr;
    const std::vector<uint8_t> *_voxels = nullptr;
    int _sx = 0, _sy = 0, _sz = 0;
    int _gx = 0, _gy = 0, _gz = 0;

    std::unordered_map<int, Node> _nodes;
    std::vector<Eigen::Vector3i> _cells; // jump points of the last search, start -> goal
};

#endif // RFN3D_JPS3D_H
