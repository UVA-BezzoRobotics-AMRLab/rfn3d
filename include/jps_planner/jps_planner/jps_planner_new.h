/**
 * @file jps_planner.h
 * @brief JPSPlanner
 */
#ifndef JPS_PLANNER_BASE_H
#define JPS_PLANNER_BASE_H

#include <jps_basis/data_type.h>
#include <jps_planner/jps_planner/graph_search.h>
#include <jps_collision/map_util.h>

class GraphSearch;
/**
 * @brief Abstract base for planning
 */
// template <int Dim>
class JPSPlanner
{
  public:
    /**
     * @brief Simple constructor
     * @param verbose enable debug mode
     */
    JPSPlanner(bool verbose = false);

    ///Set map util for collistion checking
    void setMapUtil(const std::shared_ptr<JPS::MapUtil<3>>& map_util);
    /**
     * @brief Status of the planner
     *
     * 0 --- exit normally;
     * -1 --- no path found;
     * 1, 2 --- start or goal is not free.
     */
    int status();
    ///Get the modified path
    vec_Vecf<3> getPath();
    ///Get the raw path
    vec_Vecf<3> getRawPath();
    ///remove redundant points on the same line
    vec_Vecf<3> removeLinePts(const vec_Vecf<3> &path);
    ///Remove some corner waypoints
    vec_Vecf<3> removeCornerPts(const vec_Vecf<3> &path);
    ///Must be called before run the planning thread
    void updateMap();
    ///Planning function
    bool plan(const Vecf<3> &start, const Vecf<3> &goal, decimal_t eps = 1, bool use_jps = true);
    ///Get the nodes in open set
    vec_Vecf<3> getOpenSet() const;
    ///Get the nodes in close set
    vec_Vecf<3> getCloseSet() const;
    ///Get all the nodes
    vec_Vecf<3> getAllSet() const;
  protected:
    ///Assume using 3D voxel map for all 2d and 3d planning
    std::shared_ptr<JPS::MapUtil<3>> map_util_;
    ///The planner
    std::shared_ptr<JPS::GraphSearch> graph_search_;
    ///Raw path from planner
    vec_Vecf<3> raw_path_;
    ///Modified path for future usage
    vec_Vecf<3> path_;
    ///Flag indicating the success of planning
    int status_ = 0;
    ///Enabled for printing info
    bool planner_verbose_;
    ///1-D map array
    std::vector<char> cmap_;
};

// ///Planner for 2D OccMap
// typedef JPSPlanner<2> JPSPlanner2D;

// ///Planner for 3D VoxelMap
// typedef JPSPlanner<3> JPSPlanner3D;

#endif
