#include <rfn3d/planner_debug.h>

#include <cstring>

#include <blackbox/blackbox.h>
#include <foxglove/PointCloud.pb.h>
#include <foxglove/SceneUpdate.pb.h>
#include <gcopter/geo_utils.hpp>
#include <gcopter/quickhull.hpp>
#include <rfn3d/PlanStats.pb.h>

namespace {

struct Rgba {
  double r, g, b, a;
};

constexpr Rgba kJumpColor{1.0, 0.55, 0.0, 1.0};      // orange
constexpr Rgba kPathColor{0.1, 0.9, 0.2, 1.0};       // green
constexpr Rgba kStartColor{0.2, 0.4, 1.0, 1.0};      // blue
constexpr Rgba kGoalColor{1.0, 0.1, 0.1, 1.0};       // red
constexpr Rgba kGoalUsedColor{1.0, 0.9, 0.0, 1.0};   // yellow
constexpr Rgba kNewTrajColor{1.0, 0.2, 0.9, 1.0};    // magenta
constexpr Rgba kCommittedColor{0.0, 0.9, 0.9, 1.0};  // cyan
constexpr Rgba kPolyColors[2] = {{0.2, 0.4, 1.0, 0.15}, {0.2, 0.9, 0.6, 0.15}};

void setColor(foxglove::Color* c, const Rgba& rgba) {
  c->set_r(rgba.r);
  c->set_g(rgba.g);
  c->set_b(rgba.b);
  c->set_a(rgba.a);
}

void setPoint(foxglove::Point3* p, const Eigen::Vector3d& v) {
  p->set_x(v.x());
  p->set_y(v.y());
  p->set_z(v.z());
}

void setVec(rfn3d::Vec3* p, const Eigen::Vector3d& v) {
  p->set_x(v.x());
  p->set_y(v.y());
  p->set_z(v.z());
}

void setIdentity(foxglove::Pose* pose) { pose->mutable_orientation()->set_w(1.0); }

foxglove::SceneEntity* addEntity(foxglove::SceneUpdate& update, const std::string& frame,
                                 const std::string& id, int64_t ns) {
  foxglove::SceneEntity* e = update.add_entities();
  blackbox::setStamp(e->mutable_timestamp(), ns);
  e->set_frame_id(frame);
  e->set_id(id);
  return e;  // zero lifetime: shown until replaced
}

// Each update replaces everything previously shown on its topic.
void clearAll(foxglove::SceneUpdate& update, int64_t ns) {
  foxglove::SceneEntityDeletion* d = update.add_deletions();
  blackbox::setStamp(d->mutable_timestamp(), ns);
  d->set_type(foxglove::SceneEntityDeletion::ALL);
}

void addLine(foxglove::SceneEntity* e, const std::vector<Eigen::Vector3d>& pts,
             double thickness, const Rgba& color) {
  if (pts.size() < 2) {
    return;
  }
  foxglove::LinePrimitive* line = e->add_lines();
  line->set_type(foxglove::LinePrimitive::LINE_STRIP);
  setIdentity(line->mutable_pose());
  line->set_thickness(thickness);
  setColor(line->mutable_color(), color);
  for (const auto& p : pts) {
    setPoint(line->add_points(), p);
  }
}

void addSphere(foxglove::SceneEntity* e, const Eigen::Vector3d& p, double d, const Rgba& color) {
  foxglove::SpherePrimitive* s = e->add_spheres();
  s->mutable_pose()->mutable_position()->set_x(p.x());
  s->mutable_pose()->mutable_position()->set_y(p.y());
  s->mutable_pose()->mutable_position()->set_z(p.z());
  s->mutable_pose()->mutable_orientation()->set_w(1.0);
  s->mutable_size()->set_x(d);
  s->mutable_size()->set_y(d);
  s->mutable_size()->set_z(d);
  setColor(s->mutable_color(), color);
}

void addField(foxglove::PointCloud& pc, const char* name, uint32_t offset,
              foxglove::PackedElementField::NumericType type) {
  foxglove::PackedElementField* f = pc.add_fields();
  f->set_name(name);
  f->set_offset(offset);
  f->set_type(type);
}

// x, y, z as float32 plus an optional uint8 "state" per point.
foxglove::PointCloud makeCloud(const std::string& frame, int64_t ns,
                               const std::vector<Eigen::Vector3d>& pts,
                               const std::vector<uint8_t>* state) {
  foxglove::PointCloud pc;
  blackbox::setStamp(pc.mutable_timestamp(), ns);
  pc.set_frame_id(frame);
  setIdentity(pc.mutable_pose());
  const uint32_t stride = state ? 13 : 12;
  pc.set_point_stride(stride);
  addField(pc, "x", 0, foxglove::PackedElementField::FLOAT32);
  addField(pc, "y", 4, foxglove::PackedElementField::FLOAT32);
  addField(pc, "z", 8, foxglove::PackedElementField::FLOAT32);
  if (state) {
    addField(pc, "state", 12, foxglove::PackedElementField::UINT8);
  }

  std::string data(pts.size() * stride, '\0');
  for (size_t i = 0; i < pts.size(); ++i) {
    const float xyz[3] = {static_cast<float>(pts[i].x()), static_cast<float>(pts[i].y()),
                          static_cast<float>(pts[i].z())};
    std::memcpy(&data[i * stride], xyz, sizeof(xyz));
    if (state) {
      data[i * stride + 12] = static_cast<char>((*state)[i]);
    }
  }
  pc.set_data(std::move(data));
  return pc;
}

std::vector<Eigen::Vector3d> positions(const std::vector<rfn_state_t>& traj) {
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(traj.size());
  for (const auto& s : traj) {
    pts.push_back(s.pos);
  }
  return pts;
}

}  // namespace

void PlannerDebug::publish(const PlannerCore& core, const planner_params_t& params,
                           const plan_context_t& ctx, const Eigen::Vector3d& goal_requested,
                           const std::vector<Eigen::Vector3d>& cloud,
                           const std::vector<rfn_state_t>& committed) {
  const int64_t ns = blackbox::nowNs();
  const Eigen::Vector3d start = ctx.initialPVAJ.col(0);

  // Obstacle cloud as handed to the planner.
  blackbox::record("/rfn3d/cloud", makeCloud(frame_id_, ns, cloud, nullptr), ns);

  // Voxel map: true obstacles and the inflated surface the corridor is cut from.
  size_t occupied_count = 0;
  {
    std::vector<Eigen::Vector3d> pts;
    std::vector<uint8_t> state;
    if (const voxel_map::VoxelMap* map = core.get_map()) {
      const Eigen::Vector3i size = map->getSize();
      const std::vector<uint8_t>& vox = map->getVoxels();
      for (int z = 0; z < size.z(); ++z) {
        for (int y = 0; y < size.y(); ++y) {
          for (int x = 0; x < size.x(); ++x) {
            if (vox[x + size.x() * (y + size.y() * z)] == voxel_map::Occupied) {
              pts.push_back(map->posI2D(Eigen::Vector3i(x, y, z)));
              state.push_back(1);
            }
          }
        }
      }
      occupied_count = pts.size();
      std::vector<Eigen::Vector3d> surf;
      map->getSurf(surf);
      pts.insert(pts.end(), surf.begin(), surf.end());
      state.resize(pts.size(), 2);
    }
    blackbox::record("/rfn3d/map", makeCloud(frame_id_, ns, pts, &state), ns);
  }

  // Front-end search.
  const std::vector<Eigen::Vector3d> front_path = core.get_front_path();
  {
    foxglove::SceneUpdate update;
    clearAll(update, ns);
    const double voxel = params.map_resolution;

    foxglove::SceneEntity* jumps = addEntity(update, frame_id_, "jump_points", ns);
    addLine(jumps, core.get_jump_points(), 0.03, kJumpColor);
    for (const auto& p : core.get_jump_points()) {
      addSphere(jumps, p, 1.5 * voxel, kJumpColor);
    }

    addLine(addEntity(update, frame_id_, "path", ns), front_path, 0.06, kPathColor);

    foxglove::SceneEntity* marks = addEntity(update, frame_id_, "endpoints", ns);
    addSphere(marks, start, 0.25, kStartColor);
    addSphere(marks, goal_requested, 0.3, kGoalColor);
    if ((core.get_goal_used() - goal_requested).norm() > 1e-6) {
      addSphere(marks, core.get_goal_used(), 0.3, kGoalUsedColor);
    }
    blackbox::record("/rfn3d/search", update, ns);
  }

  // Corridor polytopes as translucent meshes, one entity each, alternating colors.
  const std::vector<Eigen::MatrixX4d> hpolys = core.get_hpolys();
  {
    foxglove::SceneUpdate update;
    clearAll(update, ns);
    for (size_t i = 0; i < hpolys.size(); ++i) {
      Eigen::Matrix3Xd vpoly;
      if (!geo_utils::enumerateVs(hpolys[i], vpoly) || vpoly.cols() < 4) {
        continue;
      }
      quickhull::QuickHull<double> qh;
      const auto hull = qh.getConvexHull(vpoly.data(), vpoly.cols(), false, true);
      const auto& idx = hull.getIndexBuffer();

      foxglove::SceneEntity* e = addEntity(update, frame_id_, "poly_" + std::to_string(i), ns);
      foxglove::TriangleListPrimitive* tri = e->add_triangles();
      setIdentity(tri->mutable_pose());
      setColor(tri->mutable_color(), kPolyColors[i % 2]);
      for (size_t k = 0; k < idx.size(); ++k) {
        setPoint(tri->add_points(), vpoly.col(idx[k]));
      }
    }
    blackbox::record("/rfn3d/corridor", update, ns);
  }

  // Trajectories: what this plan produced, and what the vehicle is following.
  const std::vector<rfn_state_t> new_traj = core.get_trajectory();
  {
    foxglove::SceneUpdate update;
    clearAll(update, ns);
    addLine(addEntity(update, frame_id_, "new", ns), positions(new_traj), 0.05, kNewTrajColor);
    addLine(addEntity(update, frame_id_, "committed", ns), positions(committed), 0.03,
            kCommittedColor);
    blackbox::record("/rfn3d/trajectory", update, ns);
  }

  // Plan record.
  {
    const plan_timing_t& t = core.get_timing();
    rfn3d::PlanStats stats;
    blackbox::setStamp(stats.mutable_timestamp(), ns);
    stats.set_status(std::string(plan_metadata::to_string(ctx.status)));
    stats.set_success(ctx.status == plan_metadata::Status::SUCCESS);
    stats.set_failsafe(ctx.failsafe);
    stats.set_fail_count(static_cast<uint32_t>(ctx.fail_count));
    stats.set_frontend(params.lock_z ? "jps" : params.frontend);
    stats.set_lock_z(params.lock_z);
    stats.set_map_ms(t.map_ms);
    stats.set_search_ms(t.search_ms);
    stats.set_corridor_ms(t.corridor_ms);
    stats.set_solver_ms(t.solver_ms);
    stats.set_total_ms(ctx.total_ms);
    setVec(stats.mutable_start(), start);
    setVec(stats.mutable_goal(), core.get_goal_used());
    stats.set_horizon(ctx.horizon);
    stats.set_cloud_points(static_cast<uint32_t>(cloud.size()));
    stats.set_map_occupied(static_cast<uint32_t>(occupied_count));
    stats.set_jump_points(static_cast<uint32_t>(core.get_jump_points().size()));
    stats.set_path_points(static_cast<uint32_t>(front_path.size()));
    stats.set_polytopes(static_cast<uint32_t>(hpolys.size()));
    stats.set_traj_points(static_cast<uint32_t>(new_traj.size()));
    double length = 0.0;
    for (size_t i = 1; i < front_path.size(); ++i) {
      length += (front_path[i] - front_path[i - 1]).norm();
    }
    stats.set_path_length(length);
    blackbox::record("/rfn3d/plan", stats, ns);
  }
}
