#include <rfn3d/jps3d.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace
{

// How far from the clipped window-edge goal to look for a free voxel (m).
constexpr double kGoalSnapRadius = 1.0;

struct OpenEntry
{
    double f;
    double g;
    int id;
};

// Min-heap on f; ties go to the larger g (deeper node), as in jps3d.
struct OpenCompare
{
    bool operator()(const OpenEntry &a, const OpenEntry &b) const
    {
        if (std::abs(a.f - b.f) < 1e-6)
        {
            return a.g < b.g;
        }
        return a.f > b.f;
    }
};

inline int sign(int v)
{
    return (v > 0) - (v < 0);
}

// World position -> voxel index. floor (not VoxelMap::posD2I's truncation) so
// points just below the origin map to -1 and read as outside the map.
inline Eigen::Vector3i toCell(const voxel_map::VoxelMap &map, const Eigen::Vector3d &p)
{
    return ((p - map.getOrigin()) / map.getScale()).array().floor().cast<int>().matrix();
}

} // namespace

bool JPS3D::plan(const voxel_map::VoxelMap &map,
                 const Eigen::Vector3d &start,
                 const Eigen::Vector3d &goal,
                 std::vector<Eigen::Vector3d> &path)
{
    path.clear();
    _cells.clear();

    _map = &map;
    _voxels = &map.getVoxels();
    const Eigen::Vector3i size = map.getSize();
    _sx = size(0);
    _sy = size(1);
    _sz = size(2);

    const Eigen::Vector3i start_id = toCell(map, start);
    if (!isFree(start_id(0), start_id(1), start_id(2)))
    {
        return false;
    }

    const Eigen::Vector3i goal_cell = toCell(map, goal);
    const bool goal_in_map = inMap(goal_cell(0), goal_cell(1), goal_cell(2));

    Eigen::Vector3i goal_id;
    if (goal_in_map)
    {
        goal_id = goal_cell;
        if (!isFree(goal_id(0), goal_id(1), goal_id(2)))
        {
            return false;
        }
    }
    else if (!clipGoal(start, goal, goal_id))
    {
        return false;
    }

    if (!search(start_id, goal_id))
    {
        return false;
    }

    // Jump points as voxel centres, anchored to the exact start (and goal when
    // it is in the map) rather than the centres of their voxels.
    std::vector<Eigen::Vector3d> raw;
    raw.reserve(_cells.size() + 1);
    raw.push_back(start);
    for (size_t i = 1; i < _cells.size(); ++i)
    {
        raw.push_back(map.posI2D(_cells[i]));
    }
    const Eigen::Vector3d end = goal_in_map ? goal : map.posI2D(goal_id);
    if (raw.size() == 1)
    {
        raw.push_back(end);
    }
    else
    {
        raw.back() = end;
    }

    // Greedy line-of-sight shortcutting: from each kept point, jump to the
    // farthest later point that is directly visible.
    path.push_back(raw.front());
    size_t i = 0;
    while (i + 1 < raw.size())
    {
        size_t j = raw.size() - 1;
        while (j > i + 1 && !lineFree(raw[i], raw[j]))
        {
            --j;
        }
        path.push_back(raw[j]);
        i = j;
    }

    // Beyond the window is unknown and treated as free (as the RRT front-end
    // does), so continue straight on to the real goal when nothing in the map
    // blocks that segment. The horizon truncation downstream trims it.
    if (!goal_in_map && lineFree(path.back(), goal))
    {
        path.push_back(goal);
    }

    return true;
}

bool JPS3D::search(const Eigen::Vector3i &start, const Eigen::Vector3i &goal)
{
    _nodes.clear();
    _cells.clear();

    _gx = goal(0);
    _gy = goal(1);
    _gz = goal(2);

    const int start_id = toId(start(0), start(1), start(2));
    const int goal_id = toId(_gx, _gy, _gz);

    Node &s = _nodes[start_id];
    s.x = start(0);
    s.y = start(1);
    s.z = start(2);
    s.dx = s.dy = s.dz = 0;
    s.g = 0.0;

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenCompare> open;
    open.push({heur(s.x, s.y, s.z), 0.0, start_id});

    std::vector<int> succ_ids;
    std::vector<double> succ_costs;

    while (!open.empty())
    {
        const OpenEntry top = open.top();
        open.pop();

        // unordered_map references survive rehashing, so `curr` stays valid
        // while successors are inserted below.
        Node &curr = _nodes[top.id];
        if (curr.closed || top.g > curr.g)
        {
            continue; // stale heap entry
        }
        curr.closed = true;

        if (top.id == goal_id)
        {
            for (int id = goal_id; id != -1; id = _nodes[id].parent)
            {
                const Node &n = _nodes[id];
                _cells.emplace_back(n.x, n.y, n.z);
            }
            std::reverse(_cells.begin(), _cells.end());
            return true;
        }

        succ_ids.clear();
        succ_costs.clear();
        getSucc(curr, succ_ids, succ_costs);

        for (size_t k = 0; k < succ_ids.size(); ++k)
        {
            const int id = succ_ids[k];
            auto [it, inserted] = _nodes.try_emplace(id);
            Node &child = it->second;
            if (inserted)
            {
                child.x = id % _sx;
                child.y = (id / _sx) % _sy;
                child.z = id / (_sx * _sy);
                child.g = std::numeric_limits<double>::infinity();
            }
            if (child.closed)
            {
                continue;
            }

            const double g = curr.g + succ_costs[k];
            if (g < child.g)
            {
                child.g = g;
                child.parent = top.id;
                child.dx = sign(child.x - curr.x);
                child.dy = sign(child.y - curr.y);
                child.dz = sign(child.z - curr.z);
                open.push({g + heur(child.x, child.y, child.z), g, id});
            }
        }
    }

    return false;
}

void JPS3D::getSucc(const Node &curr, std::vector<int> &succ_ids, std::vector<double> &succ_costs)
{
    const Neib &nb = neibTable();
    const int norm1 = std::abs(curr.dx) + std::abs(curr.dy) + std::abs(curr.dz);
    const int num_neib = Neib::nsz[norm1][0];
    const int num_fneib = Neib::nsz[norm1][1];
    const int id = (curr.dx + 1) + 3 * (curr.dy + 1) + 9 * (curr.dz + 1);

    for (int dev = 0; dev < num_neib + num_fneib; ++dev)
    {
        int nx, ny, nz;
        if (dev < num_neib)
        {
            if (!jump(curr.x, curr.y, curr.z,
                      nb.ns[id][0][dev], nb.ns[id][1][dev], nb.ns[id][2][dev],
                      nx, ny, nz))
            {
                continue;
            }
        }
        else
        {
            const int f = dev - num_neib;
            if (!isOccupied(curr.x + nb.f1[id][0][f], curr.y + nb.f1[id][1][f], curr.z + nb.f1[id][2][f]))
            {
                continue;
            }
            if (!jump(curr.x, curr.y, curr.z,
                      nb.f2[id][0][f], nb.f2[id][1][f], nb.f2[id][2][f],
                      nx, ny, nz))
            {
                continue;
            }
        }

        succ_ids.push_back(toId(nx, ny, nz));
        const double ddx = nx - curr.x;
        const double ddy = ny - curr.y;
        const double ddz = nz - curr.z;
        succ_costs.push_back(std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz));
    }
}

bool JPS3D::jump(int x, int y, int z, int dx, int dy, int dz, int &nx, int &ny, int &nz) const
{
    const Neib &nb = neibTable();
    const int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
    const int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
    const int num_neib = Neib::nsz[norm1][0];

    while (true)
    {
        x += dx;
        y += dy;
        z += dz;
        if (!isFree(x, y, z))
        {
            return false;
        }
        if ((x == _gx && y == _gy && z == _gz) || hasForced(x, y, z, id, norm1))
        {
            break;
        }

        // A diagonal move is a jump point if any of its component directions
        // (all but the last ns entry, which is the move itself) reaches one.
        bool found = false;
        for (int k = 0; k < num_neib - 1 && !found; ++k)
        {
            int jx, jy, jz;
            found = jump(x, y, z, nb.ns[id][0][k], nb.ns[id][1][k], nb.ns[id][2][k], jx, jy, jz);
        }
        if (found)
        {
            break;
        }
    }

    nx = x;
    ny = y;
    nz = z;
    return true;
}

bool JPS3D::hasForced(int x, int y, int z, int id, int norm1) const
{
    // Only the first 8 / 8 / 6 forced candidates are checked for straight / 2D /
    // 3D moves; the remaining f1 entries are successor extras.
    static constexpr int kNumChecks[4] = {0, 8, 8, 6};
    const Neib &nb = neibTable();
    for (int fn = 0; fn < kNumChecks[norm1]; ++fn)
    {
        if (isOccupied(x + nb.f1[id][0][fn], y + nb.f1[id][1][fn], z + nb.f1[id][2][fn]))
        {
            return true;
        }
    }
    return false;
}

double JPS3D::heur(int x, int y, int z) const
{
    const double dx = x - _gx;
    const double dy = y - _gy;
    const double dz = z - _gz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool JPS3D::clipGoal(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                     Eigen::Vector3i &goal_id) const
{
    // Clip to the box of voxel centres; start is inside it, so the segment
    // leaves exactly once.
    const double s = _map->getScale();
    const Eigen::Vector3d lo = _map->getOrigin().array() + 0.5 * s;
    const Eigen::Vector3d hi = _map->getCorner().array() - 0.5 * s;
    const Eigen::Vector3d d = goal - start;

    double t_exit = 1.0;
    for (int i = 0; i < 3; ++i)
    {
        if (d(i) > 0)
        {
            t_exit = std::min(t_exit, (hi(i) - start(i)) / d(i));
        }
        else if (d(i) < 0)
        {
            t_exit = std::min(t_exit, (lo(i) - start(i)) / d(i));
        }
    }

    const Eigen::Vector3i size(_sx, _sy, _sz);
    const Eigen::Vector3i c =
        toCell(*_map, start + std::max(t_exit, 0.0) * d)
            .cwiseMax(Eigen::Vector3i::Zero())
            .cwiseMin(size - Eigen::Vector3i::Ones());

    if (isFree(c(0), c(1), c(2)))
    {
        goal_id = c;
        return true;
    }

    // Exit voxel is blocked: take the closest free voxel in growing cube shells.
    const int max_r = std::max(1, static_cast<int>(std::ceil(kGoalSnapRadius / s)));
    for (int r = 1; r <= max_r; ++r)
    {
        int best_d2 = std::numeric_limits<int>::max();
        for (int i = -r; i <= r; ++i)
        {
            for (int j = -r; j <= r; ++j)
            {
                for (int k = -r; k <= r; ++k)
                {
                    if (std::max({std::abs(i), std::abs(j), std::abs(k)}) != r)
                    {
                        continue;
                    }
                    const int d2 = i * i + j * j + k * k;
                    if (d2 < best_d2 && isFree(c(0) + i, c(1) + j, c(2) + k))
                    {
                        best_d2 = d2;
                        goal_id = c + Eigen::Vector3i(i, j, k);
                    }
                }
            }
        }
        if (best_d2 != std::numeric_limits<int>::max())
        {
            return true;
        }
    }

    return false;
}

bool JPS3D::lineFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const
{
    // Exact voxel traversal (Amanatides & Woo) in voxel units, so no voxel the
    // segment passes through is skipped. On exact corner crossings it steps one
    // axis first, i.e. it is conservative there.
    const double s = _map->getScale();
    const Eigen::Vector3d pa = (a - _map->getOrigin()) / s;
    const Eigen::Vector3d pb = (b - _map->getOrigin()) / s;
    const Eigen::Vector3d d = pb - pa;
    Eigen::Vector3i c = pa.array().floor().cast<int>().matrix();
    const Eigen::Vector3i end = pb.array().floor().cast<int>().matrix();

    constexpr double kInf = std::numeric_limits<double>::infinity();
    int step[3];
    double t_max[3], t_delta[3];
    for (int i = 0; i < 3; ++i)
    {
        if (d(i) > 0)
        {
            step[i] = 1;
            t_max[i] = (c(i) + 1 - pa(i)) / d(i);
            t_delta[i] = 1.0 / d(i);
        }
        else if (d(i) < 0)
        {
            step[i] = -1;
            t_max[i] = (pa(i) - c(i)) / -d(i);
            t_delta[i] = -1.0 / d(i);
        }
        else
        {
            step[i] = 0;
            t_max[i] = kInf;
            t_delta[i] = kInf;
        }
    }

    while (true)
    {
        // Outside the map is unknown and counts as free.
        if (isOccupied(c(0), c(1), c(2)))
        {
            return false;
        }
        if (c == end)
        {
            return true;
        }
        const int ax = t_max[0] < t_max[1] ? (t_max[0] < t_max[2] ? 0 : 2)
                                           : (t_max[1] < t_max[2] ? 1 : 2);
        if (t_max[ax] > 1.0)
        {
            return true; // reached b (guards against float drift past `end`)
        }
        c(ax) += step[ax];
        t_max[ax] += t_delta[ax];
    }
}

const JPS3D::Neib &JPS3D::neibTable()
{
    static const Neib table;
    return table;
}

JPS3D::Neib::Neib()
{
    for (int dz = -1; dz <= 1; ++dz)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                const int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
                const int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
                for (int dev = 0; dev < nsz[norm1][0]; ++dev)
                {
                    neib(dx, dy, dz, norm1, dev, ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
                }
                for (int dev = 0; dev < nsz[norm1][1]; ++dev)
                {
                    fneib(dx, dy, dz, norm1, dev,
                          f1[id][0][dev], f1[id][1][dev], f1[id][2][dev],
                          f2[id][0][dev], f2[id][1][dev], f2[id][2][dev]);
                }
            }
        }
    }
}

// Neighbor and forced-neighbor tables below are transcribed from jps3d's
// JPS3DNeib::Neib / JPS3DNeib::FNeib.
void JPS3D::Neib::neib(int dx, int dy, int dz, int norm1, int dev, int &tx, int &ty, int &tz)
{
    switch (norm1)
    {
    case 0:
    {
        // All 26 neighbors, in jps3d's order.
        static constexpr int all[26][3] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {-1, 1, 0}, {0, -1, 0}, {1, -1, 0}, {-1, -1, 0},
            {0, 0, 1}, {1, 0, 1}, {-1, 0, 1}, {0, 1, 1}, {1, 1, 1}, {-1, 1, 1}, {0, -1, 1}, {1, -1, 1},
            {-1, -1, 1}, {0, 0, -1}, {1, 0, -1}, {-1, 0, -1}, {0, 1, -1}, {1, 1, -1}, {-1, 1, -1},
            {0, -1, -1}, {1, -1, -1}, {-1, -1, -1}};
        tx = all[dev][0];
        ty = all[dev][1];
        tz = all[dev][2];
        return;
    }
    case 1:
        tx = dx; ty = dy; tz = dz;
        return;
    case 2:
        switch (dev)
        {
        case 0:
            if (dz == 0) { tx = 0; ty = dy; tz = 0; }
            else         { tx = 0; ty = 0;  tz = dz; }
            return;
        case 1:
            if (dx == 0) { tx = 0;  ty = dy; tz = 0; }
            else         { tx = dx; ty = 0;  tz = 0; }
            return;
        case 2:
            tx = dx; ty = dy; tz = dz;
            return;
        }
        return;
    case 3:
        switch (dev)
        {
        case 0: tx = dx; ty = 0;  tz = 0;  return;
        case 1: tx = 0;  ty = dy; tz = 0;  return;
        case 2: tx = 0;  ty = 0;  tz = dz; return;
        case 3: tx = dx; ty = dy; tz = 0;  return;
        case 4: tx = dx; ty = 0;  tz = dz; return;
        case 5: tx = 0;  ty = dy; tz = dz; return;
        case 6: tx = dx; ty = dy; tz = dz; return;
        }
        return;
    }
}

void JPS3D::Neib::fneib(int dx, int dy, int dz, int norm1, int dev,
                        int &fx, int &fy, int &fz, int &nx, int &ny, int &nz)
{
    switch (norm1)
    {
    case 1:
        switch (dev)
        {
        case 0: fx =  0; fy =  1; fz = 0; break;
        case 1: fx =  0; fy = -1; fz = 0; break;
        case 2: fx =  1; fy =  0; fz = 0; break;
        case 3: fx =  1; fy =  1; fz = 0; break;
        case 4: fx =  1; fy = -1; fz = 0; break;
        case 5: fx = -1; fy =  0; fz = 0; break;
        case 6: fx = -1; fy =  1; fz = 0; break;
        case 7: fx = -1; fy = -1; fz = 0; break;
        }
        nx = fx; ny = fy; nz = dz;
        // The pattern above is for a z move; rotate it for x / y moves.
        if (dx != 0)
        {
            fz = fx; fx = 0;
            nz = fz; nx = dx;
        }
        if (dy != 0)
        {
            fz = fy; fy = 0;
            nz = fz; ny = dy;
        }
        return;
    case 2:
        if (dx == 0)
        {
            switch (dev)
            {
            case 0:  fx =  0; fy =   0; fz = -dz; nx =  0; ny =  dy; nz = -dz; return;
            case 1:  fx =  0; fy = -dy; fz =   0; nx =  0; ny = -dy; nz =  dz; return;
            case 2:  fx =  1; fy =   0; fz =   0; nx =  1; ny =  dy; nz =  dz; return;
            case 3:  fx = -1; fy =   0; fz =   0; nx = -1; ny =  dy; nz =  dz; return;
            case 4:  fx =  1; fy =   0; fz = -dz; nx =  1; ny =  dy; nz = -dz; return;
            case 5:  fx =  1; fy = -dy; fz =   0; nx =  1; ny = -dy; nz =  dz; return;
            case 6:  fx = -1; fy =   0; fz = -dz; nx = -1; ny =  dy; nz = -dz; return;
            case 7:  fx = -1; fy = -dy; fz =   0; nx = -1; ny = -dy; nz =  dz; return;
            // Extras
            case 8:  fx =  1; fy =   0; fz =   0; nx =  1; ny =  dy; nz =   0; return;
            case 9:  fx =  1; fy =   0; fz =   0; nx =  1; ny =   0; nz =  dz; return;
            case 10: fx = -1; fy =   0; fz =   0; nx = -1; ny =  dy; nz =   0; return;
            case 11: fx = -1; fy =   0; fz =   0; nx = -1; ny =   0; nz =  dz; return;
            }
        }
        else if (dy == 0)
        {
            switch (dev)
            {
            case 0:  fx =   0; fy =  0; fz = -dz; nx =  dx; ny =  0; nz = -dz; return;
            case 1:  fx = -dx; fy =  0; fz =   0; nx = -dx; ny =  0; nz =  dz; return;
            case 2:  fx =   0; fy =  1; fz =   0; nx =  dx; ny =  1; nz =  dz; return;
            case 3:  fx =   0; fy = -1; fz =   0; nx =  dx; ny = -1; nz =  dz; return;
            case 4:  fx =   0; fy =  1; fz = -dz; nx =  dx; ny =  1; nz = -dz; return;
            case 5:  fx = -dx; fy =  1; fz =   0; nx = -dx; ny =  1; nz =  dz; return;
            case 6:  fx =   0; fy = -1; fz = -dz; nx =  dx; ny = -1; nz = -dz; return;
            case 7:  fx = -dx; fy = -1; fz =   0; nx = -dx; ny = -1; nz =  dz; return;
            // Extras
            case 8:  fx =   0; fy =  1; fz =   0; nx =  dx; ny =  1; nz =   0; return;
            case 9:  fx =   0; fy =  1; fz =   0; nx =   0; ny =  1; nz =  dz; return;
            case 10: fx =   0; fy = -1; fz =   0; nx =  dx; ny = -1; nz =   0; return;
            case 11: fx =   0; fy = -1; fz =   0; nx =   0; ny = -1; nz =  dz; return;
            }
        }
        else // dz == 0
        {
            switch (dev)
            {
            case 0:  fx =   0; fy = -dy; fz =  0; nx =  dx; ny = -dy; nz =  0; return;
            case 1:  fx = -dx; fy =   0; fz =  0; nx = -dx; ny =  dy; nz =  0; return;
            case 2:  fx =   0; fy =   0; fz =  1; nx =  dx; ny =  dy; nz =  1; return;
            case 3:  fx =   0; fy =   0; fz = -1; nx =  dx; ny =  dy; nz = -1; return;
            case 4:  fx =   0; fy = -dy; fz =  1; nx =  dx; ny = -dy; nz =  1; return;
            case 5:  fx = -dx; fy =   0; fz =  1; nx = -dx; ny =  dy; nz =  1; return;
            case 6:  fx =   0; fy = -dy; fz = -1; nx =  dx; ny = -dy; nz = -1; return;
            case 7:  fx = -dx; fy =   0; fz = -1; nx = -dx; ny =  dy; nz = -1; return;
            // Extras
            case 8:  fx =   0; fy =   0; fz =  1; nx =  dx; ny =   0; nz =  1; return;
            case 9:  fx =   0; fy =   0; fz =  1; nx =   0; ny =  dy; nz =  1; return;
            case 10: fx =   0; fy =   0; fz = -1; nx =  dx; ny =   0; nz = -1; return;
            case 11: fx =   0; fy =   0; fz = -1; nx =   0; ny =  dy; nz = -1; return;
            }
        }
        return;
    case 3:
        switch (dev)
        {
        case 0:  fx = -dx; fy =   0; fz =   0; nx = -dx; ny =  dy; nz =  dz; return;
        case 1:  fx =   0; fy = -dy; fz =   0; nx =  dx; ny = -dy; nz =  dz; return;
        case 2:  fx =   0; fy =   0; fz = -dz; nx =  dx; ny =  dy; nz = -dz; return;
        // Need to check up to here for forced!
        case 3:  fx =   0; fy = -dy; fz = -dz; nx =  dx; ny = -dy; nz = -dz; return;
        case 4:  fx = -dx; fy =   0; fz = -dz; nx = -dx; ny =  dy; nz = -dz; return;
        case 5:  fx = -dx; fy = -dy; fz =   0; nx = -dx; ny = -dy; nz =  dz; return;
        // Extras
        case 6:  fx = -dx; fy =   0; fz =   0; nx = -dx; ny =   0; nz =  dz; return;
        case 7:  fx = -dx; fy =   0; fz =   0; nx = -dx; ny =  dy; nz =   0; return;
        case 8:  fx =   0; fy = -dy; fz =   0; nx =   0; ny = -dy; nz =  dz; return;
        case 9:  fx =   0; fy = -dy; fz =   0; nx =  dx; ny = -dy; nz =   0; return;
        case 10: fx =   0; fy =   0; fz = -dz; nx =   0; ny =  dy; nz = -dz; return;
        case 11: fx =   0; fy =   0; fz = -dz; nx =  dx; ny =   0; nz = -dz; return;
        }
        return;
    }
}
