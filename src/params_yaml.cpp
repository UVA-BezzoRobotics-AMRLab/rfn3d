#include <rfn3d/params_yaml.h>

#include <iostream>
#include <set>

#include <yaml-cpp/yaml.h>

namespace
{

class Reader
{
public:
    explicit Reader(const YAML::Node &root) : _root(root) {}

    template <typename T>
    void get(const std::string &key, T &value)
    {
        _known.insert(key);
        if (const YAML::Node node = _root[key])
        {
            value = node.as<T>();
        }
    }

    void get(const std::string &key, Eigen::Vector3d &value)
    {
        _known.insert(key);
        if (const YAML::Node node = _root[key])
        {
            if (!node.IsSequence() || node.size() != 3)
            {
                throw YAML::Exception(node.Mark(), key + " must be a list of 3 numbers");
            }
            value = Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
        }
    }

    void warnUnknown(const std::string &path) const
    {
        for (const auto &kv : _root)
        {
            const std::string key = kv.first.as<std::string>();
            if (_known.count(key) == 0)
            {
                std::cerr << "[rfn3d] " << path << ": unknown parameter '" << key << "' ignored\n";
            }
        }
    }

private:
    const YAML::Node &_root;
    std::set<std::string> _known;
};

} // namespace

bool load_params_yaml(const std::string &path, planner_params_t &params, std::string &error)
{
    planner_params_t p = params;
    try
    {
        const YAML::Node root = YAML::LoadFile(path);
        if (!root.IsMap())
        {
            error = path + ": expected a map of parameter: value";
            return false;
        }

        Reader r(root);
        r.get("solver", p.solver);
        r.get("plan_rate", p.plan_rate);
        r.get("publish_debug", p.publish_debug);
        r.get("debug_frame", p.debug_frame);
        r.get("traj_dt", p.traj_dt);
        r.get("max_dist_horizon", p.max_dist_horizon);
        r.get("failsafe_count", p.failsafe_count);
        r.get("cloud_crop", p.cloud_crop);
        r.get("frontend", p.frontend);
        r.get("lock_z", p.lock_z);
        r.get("z_lock", p.z_lock);
        r.get("rrt_range", p.rrt_range);
        r.get("robot_radius", p.robot_radius);
        r.get("map_resolution", p.map_resolution);
        r.get("bounds_min", p.bounds_min);
        r.get("bounds_max", p.bounds_max);
        r.get("map_padding", p.map_padding);
        r.get("corridor_progress", p.corridor_progress);
        r.get("corridor_range", p.corridor_range);
        r.get("n_segments", p.n_segments);
        r.get("solver_dc", p.solver_dc);
        r.get("v_max", p.v_max);
        r.get("a_max", p.a_max);
        r.get("j_max", p.j_max);
        r.get("w_max", p.w_max);
        r.get("force_final_constraint", p.force_final_constraint);
        r.get("factor_init", p.factor_init);
        r.get("factor_final", p.factor_final);
        r.get("factor_increment", p.factor_increment);
        r.get("n_threads", p.n_threads);
        r.get("use_minvo", p.use_minvo);
        r.get("solver_verbose", p.solver_verbose);
        r.warnUnknown(path);
    }
    catch (const YAML::Exception &e)
    {
        error = path + ": " + e.what();
        return false;
    }

    if (!(p.plan_rate > 0.0))
    {
        error = path + ": plan_rate must be > 0";
        return false;
    }

    params = p;
    return true;
}
