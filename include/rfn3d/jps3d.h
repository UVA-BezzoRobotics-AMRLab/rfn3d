#ifndef RFN3D_JPS3D_H
#define RFN3D_JPS3D_H

#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <gcopter/voxel_map.hpp>

// Adapted from KumarRobotics/jps3d (BSD 3-Clause, Copyright (c) 2017 sikang).
class JPS3D
{
public:
    bool plan(const voxel_map::VoxelMap &map,
              const Eigen::Vector3d &start,
              const Eigen::Vector3d &goal,
              std::vector<Eigen::Vector3d> &path);

    const std::vector<Eigen::Vector3i> &getJumpCells() const { return _jump_cells; }

private:
    struct Neib
    {
        struct Counts
        {
            int always;
            int force_checks;
        };
        static constexpr Counts kCountsByNorm1[4] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};

        Neib();
        int always[27][3][26];
        int force_check[27][3][12];
        int forced[27][3][12];

    private:
        static void neib(int dx, int dy, int dz, int norm1, int dev,
                         int &tx, int &ty, int &tz);
        static void fneib(int dx, int dy, int dz, int norm1, int dev,
                          int &fx, int &fy, int &fz, int &nx, int &ny, int &nz);
    };

    struct Node
    {
        int x, y, z;
        int dx, dy, dz;
        int parent = -1;
        double g = 0.0;
        bool closed = false;
    };

    bool search(const Eigen::Vector3i &start, const Eigen::Vector3i &goal);
    void getSucc(const Node &curr, std::vector<int> &succ_ids, std::vector<double> &succ_costs);
    bool jump(int x, int y, int z, int dx, int dy, int dz, int &nx, int &ny, int &nz) const;
    bool hasForced(int x, int y, int z, int id, int move_norm1) const;

    bool clipGoal(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                  Eigen::Vector3i &goal_id) const;
    void shortcut(const std::vector<Eigen::Vector3d> &raw, std::vector<Eigen::Vector3d> &path) const;
    bool nearestFreeCell(const Eigen::Vector3i &c, int max_r, Eigen::Vector3i &out) const;
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

    // A hash map, not a grid array, so per-plan memory scales with the search, not the map.
    std::unordered_map<int, Node> _nodes;
    std::vector<Eigen::Vector3i> _jump_cells;
};

#endif // RFN3D_JPS3D_H
