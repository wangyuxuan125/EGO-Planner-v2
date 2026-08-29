#include "optimizer/poly_traj_optimizer.h"

#include <Eigen/Eigenvalues>

#include <array>
#include <chrono>
#include <queue>

#ifdef TF_SFC_WITH_DECOMP_ROS
#include <decomp_ros_msgs/PolyhedronArray.h>
#endif

using namespace std;

#define VERBOSE_OUTPUT false
#define PRINTF_COND(STR, ...) \
  if (VERBOSE_OUTPUT)         \
  printf(STR, __VA_ARGS__)

namespace
{
enum class SegmentState
{
  FREE,
  INVALID_INPUT,
  OUTSIDE_MAP,
  OCCUPIED
};

SegmentState segmentState(const GridMap::Ptr &grid_map,
                          const Eigen::Vector3d &start,
                          const Eigen::Vector3d &finish)
{
  if (!grid_map || !start.allFinite() || !finish.allFinite())
  {
    return SegmentState::INVALID_INPUT;
  }
  const auto point_state = [&](const Eigen::Vector3d &point) {
    if (!grid_map->isInInflatedMap(point))
    {
      return SegmentState::OUTSIDE_MAP;
    }
    if (grid_map->getInflateOccupancy(point) != 0)
    {
      return SegmentState::OCCUPIED;
    }
    return SegmentState::FREE;
  };
  SegmentState state = point_state(start);
  if (state != SegmentState::FREE)
  {
    return state;
  }
  state = point_state(finish);
  if (state != SegmentState::FREE)
  {
    return state;
  }

  // Traverse every intersected voxel rather than sampling along the segment.
  // Uniform samples can miss a voxel on diagonal 3-D rays even at 0.5-cell
  // spacing, which previously allowed a velocity-aligned seed to pass here
  // and fail the point-wise DecompUtil validation immediately afterwards.
  const double resolution = grid_map->getResolution();
  RayCaster raycaster;
  Eigen::Vector3d voxel;
  if (raycaster.setInput(start / resolution, finish / resolution))
  {
    while (raycaster.step(voxel))
    {
      const Eigen::Vector3d center =
          ((voxel.array() + 0.5) * resolution).matrix();
      state = point_state(center);
      if (state != SegmentState::FREE)
      {
        return state;
      }
    }
  }
  return SegmentState::FREE;
}

ego_planner::tf_sfc::FailureReason segmentFailureReason(const SegmentState state)
{
  switch (state)
  {
  case SegmentState::OUTSIDE_MAP:
    return ego_planner::tf_sfc::FailureReason::SEED_PATH_OUTSIDE_MAP;
  case SegmentState::OCCUPIED:
    return ego_planner::tf_sfc::FailureReason::SEED_PATH_OCCUPIED;
  case SegmentState::INVALID_INPUT:
    return ego_planner::tf_sfc::FailureReason::INVALID_INPUT;
  case SegmentState::FREE:
    break;
  }
  return ego_planner::tf_sfc::FailureReason::SEED_PATH_FAILURE;
}

// Exact squared distance between a segment and an axis-aligned voxel box.
// As in the TF-SFC inflator, box-face crossing parameters partition [0, 1]
// into intervals with a fixed active set, so the convex piecewise quadratic
// needs only its interval endpoints and stationary points.
double segmentAabbDistanceSquared(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &finish,
                                  const Eigen::Vector3d &box_center,
                                  const Eigen::Vector3d &half_extent)
{
  if (!start.allFinite() || !finish.allFinite() ||
      !box_center.allFinite() || !half_extent.allFinite())
  {
    return -std::numeric_limits<double>::infinity();
  }

  const Eigen::Vector3d delta = finish - start;
  std::array<double, 8> breakpoints;
  int breakpoint_count = 0;
  breakpoints[breakpoint_count++] = 0.0;
  breakpoints[breakpoint_count++] = 1.0;
  for (int axis = 0; axis < 3; ++axis)
  {
    if (std::abs(delta(axis)) <= 1.0e-12)
    {
      continue;
    }
    for (int side = 0; side < 2; ++side)
    {
      const double bound =
          box_center(axis) +
          (side == 0 ? -half_extent(axis) : half_extent(axis));
      const double ratio = (bound - start(axis)) / delta(axis);
      if (ratio > 0.0 && ratio < 1.0)
      {
        breakpoints[breakpoint_count++] = ratio;
      }
    }
  }
  std::sort(breakpoints.begin(),
            breakpoints.begin() + breakpoint_count);
  int unique_count = 0;
  for (int index = 0; index < breakpoint_count; ++index)
  {
    if (unique_count == 0 ||
        std::abs(breakpoints[index] - breakpoints[unique_count - 1]) >
            1.0e-12)
    {
      breakpoints[unique_count++] = breakpoints[index];
    }
  }

  double best_distance_squared =
      std::numeric_limits<double>::infinity();
  const auto evaluate = [&](const double ratio) {
    const Eigen::Vector3d segment_point = start + ratio * delta;
    Eigen::Vector3d box_point;
    for (int axis = 0; axis < 3; ++axis)
    {
      box_point(axis) =
          std::max(box_center(axis) - half_extent(axis),
                   std::min(box_center(axis) + half_extent(axis),
                            segment_point(axis)));
    }
    best_distance_squared =
        std::min(best_distance_squared,
                 (box_point - segment_point).squaredNorm());
  };

  for (int index = 0; index < unique_count; ++index)
  {
    evaluate(breakpoints[index]);
  }
  for (int interval = 0; interval + 1 < unique_count; ++interval)
  {
    const double lower_ratio = breakpoints[interval];
    const double upper_ratio = breakpoints[interval + 1];
    const double middle_ratio = 0.5 * (lower_ratio + upper_ratio);
    const Eigen::Vector3d middle = start + middle_ratio * delta;
    double quadratic = 0.0;
    double linear_constant = 0.0;
    for (int axis = 0; axis < 3; ++axis)
    {
      double linear = 0.0;
      double constant = 0.0;
      const double lower = box_center(axis) - half_extent(axis);
      const double upper = box_center(axis) + half_extent(axis);
      if (middle(axis) < lower)
      {
        linear = -delta(axis);
        constant = lower - start(axis);
      }
      else if (middle(axis) > upper)
      {
        linear = delta(axis);
        constant = start(axis) - upper;
      }
      quadratic += linear * linear;
      linear_constant += linear * constant;
    }
    if (quadratic > 1.0e-18)
    {
      const double stationary = -linear_constant / quadratic;
      if (stationary > lower_ratio && stationary < upper_ratio)
      {
        evaluate(stationary);
      }
    }
  }
  return best_distance_squared;
}

// A convex corridor that contains an overlap ball at both segment endpoints
// must contain the complete radius-r capsule between them.  Collision-free
// ray traversal alone only certifies the centreline.  This test checks the
// capsule against occupied inflated voxel AABBs without changing A* itself.
bool segmentHasInflatedClearance(const GridMap::Ptr &grid_map,
                                 const Eigen::Vector3d &start,
                                 const Eigen::Vector3d &finish,
                                 const double required_clearance)
{
  if (!grid_map || !start.allFinite() || !finish.allFinite() ||
      segmentState(grid_map, start, finish) != SegmentState::FREE)
  {
    return false;
  }
  const double radius = std::max(required_clearance, 0.0);
  if (radius <= 0.0)
  {
    return true;
  }

  Eigen::Vector3d map_low, map_high;
  grid_map->getInflatedMapBounds(map_low, map_high);
  const double boundary_clearance =
      std::min(std::min((start - map_low).minCoeff(),
                        (map_high - start).minCoeff()),
               std::min((finish - map_low).minCoeff(),
                        (map_high - finish).minCoeff()));
  constexpr double kClearanceTolerance = 1.0e-9;
  if (boundary_clearance + kClearanceTolerance < radius)
  {
    return false;
  }

  const double resolution = grid_map->getResolution();
  const Eigen::Vector3d half_extent =
      Eigen::Vector3d::Constant(0.5 * resolution);
  // A segment point can be half a voxel diagonal from the centre of the
  // traversed ray cell, and the tested obstacle AABB contributes another
  // half diagonal.  This neighbourhood therefore covers every voxel whose
  // box can be within radius of the continuous segment.
  const int neighbour_cells = std::max(
      1, static_cast<int>(std::ceil(
             (radius + std::sqrt(3.0) * resolution) / resolution)));
  const auto neighbourhood_is_clear =
      [&](const Eigen::Vector3i &base_index) {
        for (int dx = -neighbour_cells; dx <= neighbour_cells; ++dx)
        {
          for (int dy = -neighbour_cells; dy <= neighbour_cells; ++dy)
          {
            for (int dz = -neighbour_cells; dz <= neighbour_cells; ++dz)
            {
              const Eigen::Vector3i index =
                  base_index + Eigen::Vector3i(dx, dy, dz);
              const Eigen::Vector3d voxel_center =
                  (index.cast<double>().array() + 0.5).matrix() *
                  resolution;
              if (!grid_map->isInInflatedMap(voxel_center) ||
                  grid_map->getInflateOccupancy(voxel_center) == 0)
              {
                continue;
              }
              const double distance_squared =
                  segmentAabbDistanceSquared(start, finish, voxel_center,
                                             half_extent);
              if (!std::isfinite(distance_squared) ||
                  distance_squared + kClearanceTolerance <
                      radius * radius)
              {
                return false;
              }
            }
          }
        }
        return true;
      };

  const Eigen::Vector3i start_index =
      (start / resolution).array().floor().cast<int>();
  const Eigen::Vector3i finish_index =
      (finish / resolution).array().floor().cast<int>();
  if (!neighbourhood_is_clear(start_index) ||
      ((finish_index - start_index).squaredNorm() > 0 &&
       !neighbourhood_is_clear(finish_index)))
  {
    return false;
  }

  RayCaster raycaster;
  Eigen::Vector3d voxel;
  if (raycaster.setInput(start / resolution, finish / resolution))
  {
    while (raycaster.step(voxel))
    {
      const Eigen::Vector3i index =
          voxel.array().floor().cast<int>();
      if (!neighbourhood_is_clear(index))
      {
        return false;
      }
    }
  }
  return true;
}

// Find a short, bounded 6-neighbour detour around a clearance-invalid A*
// edge. The search is corridor-specific postprocessing: global A* output and
// metrics remain unchanged, and every accepted local edge is capsule-certified.
bool findClearanceLocalDetour(
    const GridMap::Ptr &grid_map,
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &finish,
    const double required_clearance,
    const double search_radius,
    std::vector<Eigen::Vector3d> &detour)
{
  detour.clear();
  if (!grid_map || search_radius <= 0.0 ||
      !start.allFinite() || !finish.allFinite())
  {
    return false;
  }

  const double resolution = grid_map->getResolution();
  const Eigen::Vector3i start_index =
      (start / resolution).array().floor().cast<int>();
  const Eigen::Vector3i finish_index =
      (finish / resolution).array().floor().cast<int>();
  if ((finish_index - start_index).squaredNorm() == 0)
  {
    return false;
  }

  const int search_cells = std::max(
      1, static_cast<int>(std::ceil(search_radius / resolution)));
  Eigen::Vector3i lower;
  Eigen::Vector3i upper;
  for (int axis = 0; axis < 3; ++axis)
  {
    lower(axis) =
        std::min(start_index(axis), finish_index(axis)) - search_cells;
    upper(axis) =
        std::max(start_index(axis), finish_index(axis)) + search_cells;
  }
  const Eigen::Vector3i dimensions = upper - lower +
                                     Eigen::Vector3i::Ones();
  const int total_nodes =
      dimensions.x() * dimensions.y() * dimensions.z();
  if (total_nodes <= 0)
  {
    return false;
  }

  const auto inside = [&](const Eigen::Vector3i &index) {
    return (index.array() >= lower.array()).all() &&
           (index.array() <= upper.array()).all();
  };
  const auto linearIndex = [&](const Eigen::Vector3i &index) {
    const Eigen::Vector3i local = index - lower;
    return (local.z() * dimensions.y() + local.y()) *
               dimensions.x() +
           local.x();
  };
  const auto gridIndex = [&](const int linear) -> Eigen::Vector3i {
    int remainder = linear;
    const int x = remainder % dimensions.x();
    remainder /= dimensions.x();
    const int y = remainder % dimensions.y();
    const int z = remainder / dimensions.y();
    return lower + Eigen::Vector3i(x, y, z);
  };
  const auto position =
      [&](const Eigen::Vector3i &index) -> Eigen::Vector3d {
    if ((index - start_index).squaredNorm() == 0)
    {
      return start;
    }
    if ((index - finish_index).squaredNorm() == 0)
    {
      return finish;
    }
    return ((index.cast<double>().array() + 0.5) * resolution)
        .matrix();
  };

  const int start_linear = linearIndex(start_index);
  const int finish_linear = linearIndex(finish_index);
  std::vector<int> parent(total_nodes, -2);
  std::queue<int> frontier;
  parent[start_linear] = -1;
  frontier.push(start_linear);
  const std::array<Eigen::Vector3i, 6> steps{{
      Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0),
      Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0),
      Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1)}};

  while (!frontier.empty())
  {
    const int current_linear = frontier.front();
    frontier.pop();
    if (current_linear == finish_linear)
    {
      std::vector<int> reverse_path;
      for (int node = finish_linear; node != start_linear;
           node = parent[node])
      {
        if (node < 0 || parent[node] == -2)
        {
          return false;
        }
        reverse_path.push_back(node);
      }
      std::reverse(reverse_path.begin(), reverse_path.end());
      detour.reserve(reverse_path.size());
      for (const int node : reverse_path)
      {
        detour.push_back(position(gridIndex(node)));
      }
      return !detour.empty();
    }

    const Eigen::Vector3i current_index =
        gridIndex(current_linear);
    const Eigen::Vector3d current_position =
        position(current_index);
    for (const Eigen::Vector3i &step : steps)
    {
      const Eigen::Vector3i neighbour_index =
          current_index + step;
      if (!inside(neighbour_index))
      {
        continue;
      }
      const int neighbour_linear =
          linearIndex(neighbour_index);
      if (parent[neighbour_linear] != -2)
      {
        continue;
      }
      const Eigen::Vector3d neighbour_position =
          position(neighbour_index);
      if (!grid_map->isInInflatedMap(neighbour_position) ||
          grid_map->getInflateOccupancy(neighbour_position) != 0 ||
          !segmentHasInflatedClearance(
              grid_map, current_position, neighbour_position,
              required_clearance))
      {
        continue;
      }
      parent[neighbour_linear] = current_linear;
      frontier.push(neighbour_linear);
    }
  }
  return false;
}

// Repair only clearance-invalid diagonal edges in the returned A* polyline.
// A* remains unchanged and its raw path metrics are logged before this step.
// For a 26-neighbour grid edge, deterministic axis-order permutations replace
// a corner-skimming diagonal with at most three clearance-certified axial
// edges. All corridor methods share this post-A* seed repair.
bool repairRawPathCornerCuts(
    const GridMap::Ptr &grid_map,
    std::vector<Eigen::Vector3d> &raw_path,
    const double required_clearance,
    const double local_search_radius,
    bool &repair_used,
    bool &local_search_used,
    int &failure_point_id)
{
  repair_used = false;
  local_search_used = false;
  failure_point_id = -1;
  if (!grid_map || raw_path.size() < 2)
  {
    return false;
  }

  std::vector<Eigen::Vector3d> repaired;
  repaired.reserve(3 * raw_path.size());
  repaired.push_back(raw_path.front());
  for (size_t point_id = 1; point_id < raw_path.size(); ++point_id)
  {
    const Eigen::Vector3d target = raw_path[point_id];
    const Eigen::Vector3d edge_start = repaired.back();
    if (segmentHasInflatedClearance(
            grid_map, edge_start, target, required_clearance))
    {
      repaired.push_back(target);
      continue;
    }

    bool edge_repaired = false;
    std::array<int, 3> axis_order{{0, 1, 2}};
    do
    {
      Eigen::Vector3d current = edge_start;
      std::vector<Eigen::Vector3d> candidate_points;
      bool candidate_valid = true;
      for (const int axis : axis_order)
      {
        if (std::abs(target(axis) - current(axis)) <= 1.0e-9)
        {
          continue;
        }
        Eigen::Vector3d next = current;
        next(axis) = target(axis);
        if (!grid_map->isInInflatedMap(next) ||
            grid_map->getInflateOccupancy(next) != 0 ||
            !segmentHasInflatedClearance(
                grid_map, current, next, required_clearance))
        {
          candidate_valid = false;
          break;
        }
        candidate_points.push_back(next);
        current = next;
      }
      if (candidate_valid &&
          (current - target).norm() <= 1.0e-8)
      {
        repaired.insert(repaired.end(),
                        candidate_points.begin(),
                        candidate_points.end());
        edge_repaired = true;
        repair_used = true;
        break;
      }
    } while (std::next_permutation(axis_order.begin(),
                                   axis_order.end()));

    if (!edge_repaired)
    {
      std::vector<Eigen::Vector3d> local_detour;
      if (findClearanceLocalDetour(
              grid_map, edge_start, target, required_clearance,
              local_search_radius, local_detour))
      {
        repaired.insert(repaired.end(),
                        local_detour.begin(), local_detour.end());
        edge_repaired = true;
        repair_used = true;
        local_search_used = true;
      }
    }
    if (!edge_repaired)
    {
      failure_point_id = static_cast<int>(point_id) - 1;
      return false;
    }
  }

  raw_path.swap(repaired);
  return true;
}

// Conservative distance from a point to the rolling-map boundary or to the
// nearest inflated occupied voxel AABB. This refines MINCO junction locations
// after A*; it does not alter the A* graph, costs, or returned raw path.
double pointInflatedClearance(const GridMap::Ptr &grid_map,
                              const Eigen::Vector3d &point,
                              const double requested_radius)
{
  if (!grid_map || !point.allFinite() ||
      !grid_map->isInInflatedMap(point) ||
      grid_map->getInflateOccupancy(point) != 0)
  {
    return -std::numeric_limits<double>::infinity();
  }
  const double radius = std::max(requested_radius, 0.0);
  if (radius <= 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }

  Eigen::Vector3d map_low, map_high;
  grid_map->getInflatedMapBounds(map_low, map_high);
  double clearance = std::min((point - map_low).minCoeff(),
                              (map_high - point).minCoeff());

  const double resolution = grid_map->getResolution();
  const double half_voxel = 0.5 * resolution;
  const int search_cells = std::max(
      1, static_cast<int>(std::ceil(
             (radius + std::sqrt(3.0) * half_voxel) / resolution)));
  const Eigen::Vector3i center_index =
      (point / resolution).array().floor().cast<int>();
  for (int dx = -search_cells; dx <= search_cells; ++dx)
  {
    for (int dy = -search_cells; dy <= search_cells; ++dy)
    {
      for (int dz = -search_cells; dz <= search_cells; ++dz)
      {
        const Eigen::Vector3i index =
            center_index + Eigen::Vector3i(dx, dy, dz);
        const Eigen::Vector3d voxel_center =
            (index.cast<double>().array() + 0.5).matrix() * resolution;
        if (!grid_map->isInInflatedMap(voxel_center))
        {
          continue;
        }
        if (grid_map->getInflateOccupancy(voxel_center) == 0)
        {
          continue;
        }
        Eigen::Vector3d outside =
            (point - voxel_center).cwiseAbs() -
            Eigen::Vector3d::Constant(half_voxel);
        outside = outside.cwiseMax(Eigen::Vector3d::Zero());
        clearance = std::min(clearance, outside.norm());
      }
    }
  }
  return clearance;
}

// Relocate only the internal bends of the simplified path. Candidate bends are
// searched by increasing displacement and must keep both adjacent radius-r
// seed capsules clear of occupied voxel AABBs. This is the seed-containment
// precondition used by Liu-style ellipsoid decomposition and FIRI-style
// restrictive inflation; it is deliberately a post-A* corridor operation.
bool refineSimplifiedJunctions(
    const GridMap::Ptr &grid_map,
    ego_planner::tf_sfc::PointVector &path,
    const double required_clearance,
    const double search_radius,
    const double search_step,
    bool &refinement_used)
{
  refinement_used = false;
  if (!grid_map || path.size() < 3 || required_clearance <= 0.0)
  {
    return true;
  }
  if (search_radius <= 0.0 || search_step <= 0.0)
  {
    return false;
  }

  ego_planner::tf_sfc::PointVector offsets;
  const int cells =
      std::max(1, static_cast<int>(std::ceil(search_radius / search_step)));
  for (int dx = -cells; dx <= cells; ++dx)
  {
    for (int dy = -cells; dy <= cells; ++dy)
    {
      for (int dz = -cells; dz <= cells; ++dz)
      {
        const Eigen::Vector3d offset =
            search_step * Eigen::Vector3d(dx, dy, dz);
        if (offset.norm() <= search_radius + 1.0e-9)
        {
          offsets.push_back(offset);
        }
      }
    }
  }
  std::stable_sort(
      offsets.begin(), offsets.end(),
      [](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
        return lhs.squaredNorm() < rhs.squaredNorm();
      });

  for (size_t point_id = 1; point_id + 1 < path.size(); ++point_id)
  {
    const Eigen::Vector3d original = path[point_id];
    bool found = false;
    Eigen::Vector3d accepted = original;
    for (const Eigen::Vector3d &offset : offsets)
    {
      const Eigen::Vector3d candidate = original + offset;
      if ((candidate - path[point_id - 1]).norm() <= 0.5 * search_step ||
          (candidate - path[point_id + 1]).norm() <= 0.5 * search_step)
      {
        continue;
      }
      if (pointInflatedClearance(grid_map, candidate,
                                 required_clearance) +
              1.0e-9 <
          required_clearance)
      {
        continue;
      }
      if (segmentState(grid_map, path[point_id - 1], candidate) !=
              SegmentState::FREE ||
          segmentState(grid_map, candidate, path[point_id + 1]) !=
              SegmentState::FREE)
      {
        continue;
      }
      if (!segmentHasInflatedClearance(
              grid_map, path[point_id - 1], candidate,
              required_clearance) ||
          !segmentHasInflatedClearance(
              grid_map, candidate, path[point_id + 1],
              required_clearance))
      {
        continue;
      }
      accepted = candidate;
      found = true;
      break;
    }
    if (!found)
    {
      return false;
    }
    refinement_used =
        refinement_used || (accepted - original).norm() > 1.0e-6;
    path[point_id] = accepted;
  }
  return true;
}

struct SeedPathBuildInfo
{
  std::string strategy = "astar";
  bool initial_velocity_seed_attempted = false;
  bool initial_velocity_seed_used = false;
  bool velocity_seed_fallback_used = false;
  std::string velocity_seed_fallback_reason = "none";
  bool clearance_retry_attempted = false;
  bool clearance_retry_success = false;
  std::string clearance_retry_initial_strategy = "none";
  int clearance_retry_first_failure_point_id = -1;
  bool clearance_astar_attempted = false;
  bool clearance_astar_success = false;
  int clearance_astar_call_count = 0;
  double clearance_astar_ms = 0.0;
  int seed_validation_failure_point_id = -1;
  bool astar_search_attempted = false;
  bool astar_search_success = false;
  int astar_search_call_count = 0;
  double astar_search_ms = 0.0;
  int raw_seed_path_point_count = 0;
  double raw_seed_path_length_m = 0.0;
  int seed_path_point_count = 0;
  double seed_path_length_m = 0.0;
  bool seed_path_edge_valid = false;
  double seed_path_coverage_ratio = 0.0;
  bool seed_start_in_map = false;
  bool seed_finish_in_map = false;
  bool partial_target_search_attempted = false;
  bool partial_target_found = false;
  double partial_boundary_ratio = 0.0;
  Eigen::Vector3d inflated_map_low = Eigen::Vector3d::Zero();
  Eigen::Vector3d inflated_map_high = Eigen::Vector3d::Zero();
  double seed_path_build_ms = 0.0;
};

struct SeedPathBuildTimer
{
  explicit SeedPathBuildTimer(SeedPathBuildInfo &build_info)
      : build_info_(build_info), started_(ros::WallTime::now())
  {
  }

  ~SeedPathBuildTimer()
  {
    build_info_.seed_path_build_ms =
        (ros::WallTime::now() - started_).toSec() * 1000.0;
  }

  SeedPathBuildInfo &build_info_;
  ros::WallTime started_;
};

bool buildFixedPieceSeedPath(const GridMap::Ptr &grid_map,
                             const AStar::Ptr &a_star,
                             const Eigen::Vector3d &start,
                             const Eigen::Vector3d &start_velocity,
                             const Eigen::Vector3d &finish,
                             const int piece_num,
                             const bool allow_partial_corridors,
                             const int min_valid_pieces,
                             const double min_junction_clearance,
                             const double junction_refine_radius,
                             const double junction_refine_step,
                             const double initial_velocity_segment_length,
                             const double initial_velocity_threshold,
                             const double degenerate_seed_length,
                             const bool use_clearance_astar,
                             const double clearance_astar_time_limit,
                             ego_planner::tf_sfc::PointVector &seed_path,
                             int &covered_piece_num,
                             ego_planner::tf_sfc::FailureReason &uncovered_failure_reason,
                             ego_planner::tf_sfc::FailureReason &failure_reason,
                             SeedPathBuildInfo &build_info)
{
  seed_path.clear();
  build_info = SeedPathBuildInfo();
  SeedPathBuildTimer build_timer(build_info);
  covered_piece_num = 0;
  uncovered_failure_reason = ego_planner::tf_sfc::FailureReason::NONE;
  failure_reason = ego_planner::tf_sfc::FailureReason::INVALID_INPUT;
  if (!grid_map || !a_star || piece_num <= 0 || !start.allFinite() ||
      !start_velocity.allFinite() || !finish.allFinite())
  {
    return false;
  }

  build_info.strategy = "start_map_validation";
  build_info.seed_start_in_map = grid_map->isInInflatedMap(start);
  if (!build_info.seed_start_in_map)
  {
    build_info.strategy = "start_outside_inflated_map";
    failure_reason = ego_planner::tf_sfc::FailureReason::SEED_PATH_OUTSIDE_MAP;
    return false;
  }
  if (grid_map->getInflateOccupancy(start) != 0)
  {
    failure_reason = ego_planner::tf_sfc::FailureReason::SEED_START_INVALID;
    return false;
  }

  Eigen::Vector3d seed_finish = finish;
  build_info.seed_finish_in_map = grid_map->isInInflatedMap(finish);
  bool full_coverage = build_info.seed_finish_in_map &&
                       grid_map->getInflateOccupancy(finish) == 0;
  double coverage_ratio = 1.0;
  if (!full_coverage)
  {
    uncovered_failure_reason = ego_planner::tf_sfc::FailureReason::OUTSIDE_LOCAL_MAP;
    build_info.partial_target_search_attempted = true;
    build_info.strategy = "partial_target_backoff";
    grid_map->getInflatedMapBounds(build_info.inflated_map_low,
                                   build_info.inflated_map_high);
    const Eigen::Vector3d &map_low = build_info.inflated_map_low;
    const Eigen::Vector3d &map_high = build_info.inflated_map_high;
    if (!allow_partial_corridors || piece_num <= 1)
    {
      build_info.strategy =
          allow_partial_corridors ? "partial_piece_budget_unavailable"
                                  : "partial_corridors_disabled";
      failure_reason = ego_planner::tf_sfc::FailureReason::OUTSIDE_LOCAL_MAP;
      return false;
    }
    const Eigen::Vector3d delta = finish - start;
    const double distance = delta.norm();
    if (distance <= 2.0 * grid_map->getResolution())
    {
      failure_reason = ego_planner::tf_sfc::FailureReason::OUTSIDE_LOCAL_MAP;
      return false;
    }

    double boundary_ratio = 1.0;
    for (int axis = 0; axis < 3; ++axis)
    {
      if (delta(axis) > 1.0e-9)
      {
        boundary_ratio = std::min(boundary_ratio,
                                  (map_high(axis) - start(axis)) / delta(axis));
      }
      else if (delta(axis) < -1.0e-9)
      {
        boundary_ratio = std::min(boundary_ratio,
                                  (map_low(axis) - start(axis)) / delta(axis));
      }
    }
    const double ratio_step = grid_map->getResolution() / distance;
    build_info.partial_boundary_ratio = boundary_ratio;
    coverage_ratio = std::min(1.0, boundary_ratio) - 1.5 * ratio_step;
    bool found_free_target = false;
    for (; coverage_ratio > ratio_step; coverage_ratio -= ratio_step)
    {
      seed_finish = start + coverage_ratio * delta;
      if (grid_map->isInInflatedMap(seed_finish) &&
          grid_map->getInflateOccupancy(seed_finish) == 0)
      {
        found_free_target = true;
        break;
      }
    }
    build_info.partial_target_found = found_free_target;
    if (!found_free_target)
    {
      build_info.strategy =
          boundary_ratio <= ratio_step
              ? "inflated_map_forward_extent_not_ready"
              : "partial_target_backoff_failed";
      failure_reason = ego_planner::tf_sfc::FailureReason::OUTSIDE_LOCAL_MAP;
      return false;
    }
  }
  build_info.seed_path_coverage_ratio = coverage_ratio;

  const double resolution = grid_map->getResolution();
  const double seed_distance = (seed_finish - start).norm();
  std::vector<Eigen::Vector3d> raw_path;

  // A* commonly collapses a same-voxel query to a zero-segment path. Preserve
  // a real segment for DecompUtil, following Elastic-Tracker's short-path
  // guard, but collision-check the direction instead of blindly adding +z.
  if (seed_distance <= 1.5 * resolution &&
      segmentState(grid_map, start, seed_finish) == SegmentState::FREE)
  {
    if (seed_distance > 1.0e-6)
    {
      raw_path.push_back(start);
      raw_path.push_back(seed_finish);
      build_info.strategy = "direct_short_seed";
    }
    else
    {
      std::vector<Eigen::Vector3d> directions;
      if (start_velocity.norm() > initial_velocity_threshold)
      {
        directions.push_back(start_velocity.normalized());
      }
      directions.push_back(Eigen::Vector3d::UnitX());
      directions.push_back(-Eigen::Vector3d::UnitX());
      directions.push_back(Eigen::Vector3d::UnitY());
      directions.push_back(-Eigen::Vector3d::UnitY());
      directions.push_back(Eigen::Vector3d::UnitZ());
      directions.push_back(-Eigen::Vector3d::UnitZ());
      const double probe_length = std::max(degenerate_seed_length, resolution);
      bool found_probe = false;
      for (const Eigen::Vector3d &direction : directions)
      {
        for (double scale = 1.0; scale >= 0.25; scale *= 0.5)
        {
          const Eigen::Vector3d candidate =
              start + scale * probe_length * direction;
          if (segmentState(grid_map, start, candidate) == SegmentState::FREE)
          {
            raw_path.push_back(start);
            raw_path.push_back(candidate);
            found_probe = true;
            break;
          }
        }
        if (found_probe)
        {
          break;
        }
      }
      if (!found_probe)
      {
        failure_reason =
            ego_planner::tf_sfc::FailureReason::INSUFFICIENT_PIECES;
        return false;
      }
      build_info.strategy = "collision_checked_degenerate_probe";
    }
  }
  else
  {
    Eigen::Vector3d search_start = start;
    if (start_velocity.norm() > initial_velocity_threshold &&
        initial_velocity_segment_length > 0.0 && piece_num > 1)
    {
      build_info.initial_velocity_seed_attempted = true;
      const Eigen::Vector3d candidate =
          start + initial_velocity_segment_length * start_velocity.normalized();
      const bool prefix_has_room =
          (candidate - seed_finish).norm() > 0.5 * resolution;
      const SegmentState prefix_state =
          prefix_has_room ? segmentState(grid_map, start, candidate)
                          : SegmentState::INVALID_INPUT;
      if (prefix_has_room && prefix_state == SegmentState::FREE)
      {
        search_start = candidate;
        build_info.initial_velocity_seed_used = true;
        build_info.strategy = "velocity_aligned_astar";
      }
      else
      {
        build_info.velocity_seed_fallback_used = true;
        build_info.velocity_seed_fallback_reason =
            prefix_has_room
                ? ego_planner::tf_sfc::failureReasonName(
                      segmentFailureReason(prefix_state))
                : "velocity_prefix_too_close";
        build_info.strategy = "astar_velocity_prefix_fallback";
      }
    }

    if (build_info.initial_velocity_seed_used)
    {
      build_info.strategy = "velocity_aligned_edge_validated_astar";
    }
    else if (build_info.velocity_seed_fallback_used)
    {
      build_info.strategy = "edge_validated_astar_velocity_prefix_fallback";
    }
    else
    {
      build_info.strategy = "edge_validated_astar";
    }
    if (use_clearance_astar)
    {
      build_info.strategy = "clearance_" + build_info.strategy;
    }

    AStar::EdgeValidator clearance_edge_validator;
    if (use_clearance_astar)
    {
      clearance_edge_validator =
          [grid_map, min_junction_clearance](
              const Eigen::Vector3d &edge_start,
              const Eigen::Vector3d &edge_finish) {
            return segmentHasInflatedClearance(
                grid_map, edge_start, edge_finish,
                std::max(min_junction_clearance, 0.0));
          };
    }

    const auto run_astar = [&](const Eigen::Vector3d &query_start,
                               const Eigen::Vector3d &query_finish) {
      build_info.astar_search_attempted = true;
      ++build_info.astar_search_call_count;
      if (use_clearance_astar)
      {
        build_info.clearance_astar_attempted = true;
        ++build_info.clearance_astar_call_count;
      }
      const ros::WallTime search_started = ros::WallTime::now();
      const ASTAR_RET result =
          a_star->AstarSearch(
              resolution, query_start, query_finish, true, true,
              clearance_edge_validator,
              use_clearance_astar ? clearance_astar_time_limit : 0.20);
      const double elapsed_ms =
          (ros::WallTime::now() - search_started).toSec() * 1000.0;
      build_info.astar_search_ms += elapsed_ms;
      if (use_clearance_astar)
      {
        build_info.clearance_astar_ms += elapsed_ms;
        build_info.clearance_astar_success =
            build_info.clearance_astar_success ||
            result == ASTAR_RET::SUCCESS;
      }
      return result;
    };
    ASTAR_RET search_result = run_astar(search_start, seed_finish);
    if (search_result != ASTAR_RET::SUCCESS &&
        build_info.initial_velocity_seed_used)
    {
      // A velocity-aligned prefix is a robustness aid, not a reason to lose an
      // otherwise valid A* route.
      build_info.initial_velocity_seed_used = false;
      build_info.velocity_seed_fallback_used = true;
      build_info.velocity_seed_fallback_reason = "velocity_astar_failure";
      build_info.strategy =
          "edge_validated_astar_velocity_search_fallback";
      search_start = start;
      search_result = run_astar(search_start, seed_finish);
    }
    if (search_result != ASTAR_RET::SUCCESS)
    {
      failure_reason = ego_planner::tf_sfc::FailureReason::SEED_ASTAR_FAILURE;
      return false;
    }

    raw_path = a_star->getPath();
    if (raw_path.empty())
    {
      failure_reason = ego_planner::tf_sfc::FailureReason::SEED_ASTAR_FAILURE;
      return false;
    }
    build_info.astar_search_success = true;

    // Preserve the A* grid endpoints. Replacing the first/last grid point with
    // an exact state can turn a valid grid edge into a longer diagonal that
    // crosses an occupied voxel.
    const Eigen::Vector3d astar_grid_start = raw_path.front();
    if ((astar_grid_start - search_start).norm() > 1.0e-6)
    {
      const SegmentState start_bridge_state =
          segmentState(grid_map, search_start, astar_grid_start);
      if (start_bridge_state != SegmentState::FREE)
      {
        build_info.seed_validation_failure_point_id = 0;
        failure_reason = segmentFailureReason(start_bridge_state);
        return false;
      }
      raw_path.insert(raw_path.begin(), search_start);
    }
    else
    {
      raw_path.front() = search_start;
    }
    if (build_info.initial_velocity_seed_used)
    {
      // The velocity prefix was already certified before the A* request.
      raw_path.insert(raw_path.begin(), start);
    }

    const Eigen::Vector3d astar_grid_finish = raw_path.back();
    if ((astar_grid_finish - seed_finish).norm() > 1.0e-6)
    {
      const SegmentState finish_bridge_state =
          segmentState(grid_map, astar_grid_finish, seed_finish);
      if (finish_bridge_state != SegmentState::FREE)
      {
        build_info.seed_validation_failure_point_id =
            static_cast<int>(raw_path.size()) - 1;
        failure_reason = segmentFailureReason(finish_bridge_state);
        return false;
      }
      raw_path.push_back(seed_finish);
    }
    else
    {
      raw_path.back() = seed_finish;
    }
  }

  build_info.raw_seed_path_point_count = static_cast<int>(raw_path.size());
  for (std::size_t point_id = 1; point_id < raw_path.size(); ++point_id)
  {
    build_info.raw_seed_path_length_m +=
        (raw_path[point_id] - raw_path[point_id - 1]).norm();
  }

  bool corner_repair_used = false;
  bool local_search_repair_used = false;
  int corner_repair_failure_point_id = -1;
  if (!repairRawPathCornerCuts(
          grid_map, raw_path,
          std::max(min_junction_clearance, 0.0),
          std::max(junction_refine_radius, resolution),
          corner_repair_used, local_search_repair_used,
          corner_repair_failure_point_id))
  {
    build_info.seed_validation_failure_point_id =
        corner_repair_failure_point_id;
    build_info.strategy += "_clearance_local_grid_repair_failed";
    failure_reason =
        ego_planner::tf_sfc::FailureReason::SEED_CLEARANCE_FAILURE;
    return false;
  }
  if (local_search_repair_used)
  {
    build_info.strategy += "_clearance_local_grid_repaired";
  }
  else if (corner_repair_used)
  {
    build_info.strategy += "_clearance_corner_repaired";
  }

  ego_planner::tf_sfc::PointVector simplified;
  simplified.push_back(raw_path.front());
  int current = 0;
  if (build_info.initial_velocity_seed_used && raw_path.size() > 1)
  {
    simplified.push_back(raw_path[1]);
    current = 1;
  }
  while (current + 1 < static_cast<int>(raw_path.size()))
  {
    int next = static_cast<int>(raw_path.size()) - 1;
    while (next > current + 1)
    {
      const SegmentState candidate_state =
          segmentState(grid_map, raw_path[current], raw_path[next]);
      const bool candidate_capsule_clear =
          candidate_state == SegmentState::FREE &&
          segmentHasInflatedClearance(
              grid_map, raw_path[current], raw_path[next],
              std::max(min_junction_clearance, 0.0));
      if (candidate_capsule_clear)
      {
        break;
      }
      --next;
    }
    const SegmentState state =
        segmentState(grid_map, raw_path[current], raw_path[next]);
    if (state != SegmentState::FREE)
    {
      build_info.seed_validation_failure_point_id = current;
      failure_reason = segmentFailureReason(state);
      return false;
    }
    if (!segmentHasInflatedClearance(
            grid_map, raw_path[current], raw_path[next],
            std::max(min_junction_clearance, 0.0)))
    {
      build_info.seed_validation_failure_point_id = current;
      failure_reason =
          ego_planner::tf_sfc::FailureReason::OVERLAP_TOO_SMALL;
      return false;
    }
    simplified.push_back(raw_path[next]);
    current = next;
  }

  int segment_count = static_cast<int>(simplified.size()) - 1;
  const bool complete_corridor_budget = full_coverage;
  int max_covered_piece_num = complete_corridor_budget ? piece_num : piece_num - 1;
  if (segment_count <= 0 ||
      max_covered_piece_num < std::max(min_valid_pieces, 1))
  {
    failure_reason = ego_planner::tf_sfc::FailureReason::INSUFFICIENT_PIECES;
    return false;
  }
  if (segment_count > max_covered_piece_num && allow_partial_corridors &&
      piece_num > 1)
  {
    // Even when the goal is inside the rolling map, a short MINCO plan may
    // have fewer pieces than the collision-free A* polyline has bends. Keep a
    // certified prefix and leave one explicitly labelled tail piece to EGO's
    // original obstacle penalty and final collision check.
    if (complete_corridor_budget)
    {
      max_covered_piece_num = piece_num - 1;
      uncovered_failure_reason =
          ego_planner::tf_sfc::FailureReason::PIECE_BUDGET_TAIL;
    }
    if (max_covered_piece_num < std::max(min_valid_pieces, 1))
    {
      failure_reason = ego_planner::tf_sfc::FailureReason::INSUFFICIENT_PIECES;
      return false;
    }
    // A certified local prefix does not need to reach the rolling-map boundary.
    // Keep the farthest line-of-sight A* bends that fit the available prefix
    // pieces and leave the remaining route to EGO's original obstacle terms.
    simplified.resize(max_covered_piece_num + 1);
    segment_count = max_covered_piece_num;
  }
  else if (segment_count > max_covered_piece_num)
  {
    failure_reason = ego_planner::tf_sfc::FailureReason::INSUFFICIENT_PIECES;
    return false;
  }
  const int proportional_piece_num = complete_corridor_budget
                                         ? piece_num
                                         : std::max(1, static_cast<int>(std::floor(
                                                           piece_num * coverage_ratio)));
  covered_piece_num = std::min(
      max_covered_piece_num,
      std::max(segment_count,
               std::max(min_valid_pieces, proportional_piece_num)));
  std::vector<int> divisions(segment_count, 1);
  for (int remaining = covered_piece_num - segment_count; remaining > 0; --remaining)
  {
    int best = 0;
    double best_length = -1.0;
    for (int segment_id = 0; segment_id < segment_count; ++segment_id)
    {
      const double divided_length =
          (simplified[segment_id + 1] - simplified[segment_id]).norm() /
          static_cast<double>(divisions[segment_id]);
      if (divided_length > best_length)
      {
        best_length = divided_length;
        best = segment_id;
      }
    }
    ++divisions[best];
  }

  seed_path.push_back(simplified.front());
  for (int segment_id = 0; segment_id < segment_count; ++segment_id)
  {
    for (int division = 1; division <= divisions[segment_id]; ++division)
    {
      seed_path.push_back(
          simplified[segment_id] +
          (simplified[segment_id + 1] - simplified[segment_id]) *
              static_cast<double>(division) /
              static_cast<double>(divisions[segment_id]));
    }
  }
  bool junction_refinement_used = false;
  if (!refineSimplifiedJunctions(
          grid_map, seed_path, std::max(min_junction_clearance, 0.0),
          junction_refine_radius, junction_refine_step,
          junction_refinement_used))
  {
    build_info.strategy += "_overlap_local_refine_failed";
    failure_reason =
        ego_planner::tf_sfc::FailureReason::SEED_CLEARANCE_FAILURE;
    return false;
  }
  if (junction_refinement_used)
  {
    build_info.strategy += "_overlap_local_refined";
  }

  const bool point_count_valid =
      static_cast<int>(seed_path.size()) == covered_piece_num + 1;
  bool seed_edges_valid = point_count_valid;
  ego_planner::tf_sfc::FailureReason edge_failure_reason =
      ego_planner::tf_sfc::FailureReason::NONE;
  build_info.seed_path_point_count = static_cast<int>(seed_path.size());
  for (std::size_t point_id = 1; point_id < seed_path.size(); ++point_id)
  {
    build_info.seed_path_length_m +=
        (seed_path[point_id] - seed_path[point_id - 1]).norm();
    const SegmentState edge_state =
        segmentState(grid_map, seed_path[point_id - 1], seed_path[point_id]);
    if (edge_state != SegmentState::FREE)
    {
      seed_edges_valid = false;
      if (edge_failure_reason ==
          ego_planner::tf_sfc::FailureReason::NONE)
      {
        edge_failure_reason = segmentFailureReason(edge_state);
      }
    }
    else if (!segmentHasInflatedClearance(
                 grid_map, seed_path[point_id - 1], seed_path[point_id],
                 std::max(min_junction_clearance, 0.0)))
    {
      seed_edges_valid = false;
      if (edge_failure_reason ==
          ego_planner::tf_sfc::FailureReason::NONE)
      {
        edge_failure_reason =
            ego_planner::tf_sfc::FailureReason::SEED_CLEARANCE_FAILURE;
      }
    }
  }
  build_info.seed_path_edge_valid = seed_edges_valid;
  const bool valid = point_count_valid && seed_edges_valid;
  failure_reason =
      valid
          ? ego_planner::tf_sfc::FailureReason::NONE
          : (edge_failure_reason != ego_planner::tf_sfc::FailureReason::NONE
                 ? edge_failure_reason
                 : ego_planner::tf_sfc::FailureReason::SEED_PATH_FAILURE);
  return valid;
}

bool buildFixedPieceSeedPathWithClearanceRetry(
    const GridMap::Ptr &grid_map,
    const AStar::Ptr &a_star,
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &start_velocity,
    const Eigen::Vector3d &finish,
    const int piece_num,
    const bool allow_partial_corridors,
    const int min_valid_pieces,
    const double min_junction_clearance,
    const double junction_refine_radius,
    const double junction_refine_step,
    const double initial_velocity_segment_length,
    const double initial_velocity_threshold,
    const double degenerate_seed_length,
    const bool retry_without_velocity_on_clearance_failure,
    const bool clearance_astar_enabled,
    const double clearance_astar_time_limit,
    ego_planner::tf_sfc::PointVector &seed_path,
    int &covered_piece_num,
    ego_planner::tf_sfc::FailureReason &uncovered_failure_reason,
    ego_planner::tf_sfc::FailureReason &failure_reason,
    SeedPathBuildInfo &build_info)
{
  const bool primary_ok = buildFixedPieceSeedPath(
      grid_map, a_star, start, start_velocity, finish, piece_num,
      allow_partial_corridors, min_valid_pieces, min_junction_clearance,
      junction_refine_radius, junction_refine_step,
      initial_velocity_segment_length, initial_velocity_threshold,
      degenerate_seed_length, false, clearance_astar_time_limit,
      seed_path, covered_piece_num,
      uncovered_failure_reason, failure_reason, build_info);
  if (primary_ok ||
      !retry_without_velocity_on_clearance_failure ||
      failure_reason !=
          ego_planner::tf_sfc::FailureReason::SEED_CLEARANCE_FAILURE ||
      (!clearance_astar_enabled &&
       !build_info.initial_velocity_seed_used))
  {
    return primary_ok;
  }

  const SeedPathBuildInfo primary_info = build_info;
  ROS_WARN(
      "TF-SFC seed failed clearance certification at point %d; retrying "
      "once with %s A*.",
      primary_info.seed_validation_failure_point_id,
      clearance_astar_enabled ? "clearance-aware" : "ordinary");

  ego_planner::tf_sfc::PointVector retry_seed_path;
  int retry_covered_piece_num = 0;
  ego_planner::tf_sfc::FailureReason retry_uncovered_failure_reason =
      ego_planner::tf_sfc::FailureReason::NONE;
  ego_planner::tf_sfc::FailureReason retry_failure_reason =
      ego_planner::tf_sfc::FailureReason::NONE;
  SeedPathBuildInfo retry_info;
  const bool retry_ok = buildFixedPieceSeedPath(
      grid_map, a_star, start, Eigen::Vector3d::Zero(), finish, piece_num,
      allow_partial_corridors, min_valid_pieces, min_junction_clearance,
      junction_refine_radius, junction_refine_step, 0.0,
      initial_velocity_threshold, degenerate_seed_length,
      clearance_astar_enabled, clearance_astar_time_limit, retry_seed_path,
      retry_covered_piece_num, retry_uncovered_failure_reason,
      retry_failure_reason, retry_info);

  retry_info.clearance_retry_attempted = true;
  retry_info.clearance_retry_success = retry_ok;
  retry_info.clearance_retry_initial_strategy = primary_info.strategy;
  retry_info.clearance_retry_first_failure_point_id =
      primary_info.seed_validation_failure_point_id;
  retry_info.initial_velocity_seed_attempted =
      primary_info.initial_velocity_seed_attempted;
  retry_info.initial_velocity_seed_used = false;
  retry_info.velocity_seed_fallback_used =
      primary_info.initial_velocity_seed_used ||
      primary_info.velocity_seed_fallback_used;
  retry_info.velocity_seed_fallback_reason =
      primary_info.initial_velocity_seed_used
          ? "seed_clearance_failure"
          : primary_info.velocity_seed_fallback_reason;
  retry_info.astar_search_attempted =
      retry_info.astar_search_attempted ||
      primary_info.astar_search_attempted;
  retry_info.astar_search_success =
      retry_info.astar_search_success ||
      primary_info.astar_search_success;
  retry_info.astar_search_call_count +=
      primary_info.astar_search_call_count;
  retry_info.astar_search_ms += primary_info.astar_search_ms;
  retry_info.seed_path_build_ms += primary_info.seed_path_build_ms;
  retry_info.strategy +=
      clearance_astar_enabled
          ? (retry_ok ? "_clearance_astar_retry_succeeded"
                      : "_clearance_astar_retry_failed")
          : (retry_ok ? "_clearance_velocity_retry_succeeded"
                      : "_clearance_velocity_retry_failed");

  seed_path.swap(retry_seed_path);
  covered_piece_num = retry_covered_piece_num;
  uncovered_failure_reason = retry_uncovered_failure_reason;
  failure_reason = retry_failure_reason;
  build_info = retry_info;
  return retry_ok;
}
}

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::optimizeTrajectory(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      double &final_cost)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    // Preparision 1: Some mise params
    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_unsafe, flag_success, flag_swarm_too_close;
    bool tf_sfc_generated = false;
    bool fallback_to_ego = false;
    bool projection_applied = false;
    bool hard_parameterization_active = false;
    hard_parameterization_active_ = false;
    bool final_collision_free = false;
    bool final_obstacle_collision = false;
    bool final_swarm_clearance_failure = false;
    std::string terminal_failure_reason = "none";
    bool corridor_retry_requested = false;
    bool strict_corridor_rejected = false;
    bool progress_guard_evaluated = false;
    bool progress_guard_passed = true;
    double min_initial_seed_progress_m = 0.0;
    double max_target_axis_progress_drop_m = 0.0;
    double max_target_overshoot_m = 0.0;
    std::string progress_guard_reason = "not_evaluated";
    bool corridor_generation_attempted = false;
    const std::string requested_corridor_method = tf_sfc_parameters_.enabled
                                                      ? tf_sfc_parameters_.corridor_method
                                                      : "ego";
    const bool trajectory_repair_enabled =
        tf_sfc_parameters_.enabled &&
        requested_corridor_method == "tf_sfc" &&
        tf_sfc_parameters_.trajectory_repair_enabled;
    int trajectory_repair_attempt_count = 0;
    int trajectory_repair_accept_count = 0;
    double trajectory_repair_ms = 0.0;
    tf_sfc::TrajectoryRepairResult trajectory_repair_result;
    const bool post_optimization_repair_enabled =
        trajectory_repair_enabled &&
        tf_sfc_parameters_.post_optimization_repair_enabled;
    int post_optimization_repair_attempt_count = 0;
    int post_optimization_repair_accept_count = 0;
    double post_optimization_repair_ms = 0.0;
    tf_sfc::TrajectoryRepairResult post_optimization_repair_result;
    double optimizer_time_ms = 0.0;
    double tf_sfc_generation_ms = 0.0;
    double tf_sfc_inflation_ms = 0.0;
    bool objective_compliance_attempted = false;
    bool objective_compliance_success = false;
    int objective_compliance_evaluation_count = 0;
    int objective_compliance_regularized_eigenvalue_count = 0;
    double objective_compliance_ms = 0.0;
    double objective_compliance_raw_min_eigenvalue = 0.0;
    double objective_compliance_raw_max_eigenvalue = 0.0;
    double objective_compliance_regularized_condition_number = 0.0;
    std::string objective_compliance_reason = "not_requested";
    int total_lbfgs_iterations = 0;
    int last_lbfgs_result = 0;
    int corridor_enforcement_passes = 0;
    double corridor_penalty_scale = 1.0;
    SeedPathBuildInfo seed_path_build_info;
    int corridor_candidate_count = 0;
    int corridor_candidate_accept_count = 0;
    bool corridor_rollback_applied = false;
    std::string corridor_rollback_reason = "none";
    double best_corridor_violation_m =
        std::numeric_limits<double>::infinity();
    double max_junction_violation_initial_m = 0.0;
    double max_junction_violation_final_m = 0.0;
    tf_sfc::CorridorVector logged_corridors;
    tf_sfc::CorridorEvaluation initial_corridor_evaluation;
    multitopology_data_.initial_obstacles_avoided = false;
    wei_swarm_mod_ = wei_swarm_;

    const auto populate_seed_record =
        [&](tf_sfc::ExperimentRunRecord &record) {
          record.planning_event_id =
              "d" + std::to_string(drone_id_) + "-g" +
              std::to_string(experiment_goal_id_) + "-r" +
              std::to_string(experiment_replan_id_);
          record.retry_index = experiment_retry_index_;
          record.commanded_goal_valid =
              experiment_commanded_goal_valid_;
          record.commanded_goal_x_m = experiment_commanded_goal_.x();
          record.commanded_goal_y_m = experiment_commanded_goal_.y();
          record.commanded_goal_z_m = experiment_commanded_goal_.z();
          record.planning_start_x_m = iniState(0, 0);
          record.planning_start_y_m = iniState(1, 0);
          record.planning_start_z_m = iniState(2, 0);
          record.planning_target_x_m = finState(0, 0);
          record.planning_target_y_m = finState(1, 0);
          record.planning_target_z_m = finState(2, 0);
          record.astar_search_attempted =
              seed_path_build_info.astar_search_attempted;
          record.astar_search_success =
              seed_path_build_info.astar_search_success;
          record.astar_search_call_count =
              seed_path_build_info.astar_search_call_count;
          record.astar_search_ms = seed_path_build_info.astar_search_ms;
          record.raw_seed_path_point_count =
              seed_path_build_info.raw_seed_path_point_count;
          record.raw_seed_path_length_m =
              seed_path_build_info.raw_seed_path_length_m;
          record.seed_path_point_count =
              seed_path_build_info.seed_path_point_count;
          record.seed_piece_count =
              std::max(seed_path_build_info.seed_path_point_count - 1, 0);
          record.corridor_slot_count =
              static_cast<int>(logged_corridors.size());
          record.seed_minco_alignment_valid =
              record.seed_piece_count > 0 &&
              record.seed_piece_count <= piece_num_ &&
              record.corridor_slot_count == piece_num_;
          record.seed_path_length_m =
              seed_path_build_info.seed_path_length_m;
          record.seed_path_edge_valid =
              seed_path_build_info.seed_path_edge_valid;
          record.seed_path_coverage_ratio =
              seed_path_build_info.seed_path_coverage_ratio;
          record.allow_partial_corridors =
              tf_sfc_parameters_.allow_partial_corridors;
          record.seed_start_in_map =
              seed_path_build_info.seed_start_in_map;
          record.seed_finish_in_map =
              seed_path_build_info.seed_finish_in_map;
          record.partial_target_search_attempted =
              seed_path_build_info.partial_target_search_attempted;
          record.partial_target_found =
              seed_path_build_info.partial_target_found;
          record.partial_boundary_ratio =
              seed_path_build_info.partial_boundary_ratio;
          record.inflated_map_low_x_m =
              seed_path_build_info.inflated_map_low.x();
          record.inflated_map_low_y_m =
              seed_path_build_info.inflated_map_low.y();
          record.inflated_map_low_z_m =
              seed_path_build_info.inflated_map_low.z();
          record.inflated_map_high_x_m =
              seed_path_build_info.inflated_map_high.x();
          record.inflated_map_high_y_m =
              seed_path_build_info.inflated_map_high.y();
          record.inflated_map_high_z_m =
              seed_path_build_info.inflated_map_high.z();
          record.seed_path_build_ms =
              seed_path_build_info.seed_path_build_ms;
          record.corridor_inflation_ms = tf_sfc_inflation_ms;
          record.seed_frontend_evaluated =
              tf_sfc_parameters_.enabled && static_cast<bool>(tf_sfc_manager_);
          record.seed_frontend_success =
              record.seed_frontend_evaluated &&
              seed_path_build_info.seed_path_point_count >= 2 &&
              seed_path_build_info.seed_path_edge_valid;
          record.corridor_generation_attempted =
              corridor_generation_attempted;
          record.direction_fallback_allowed =
              tf_sfc_parameters_.allow_direction_fallback;
          record.objective_compliance_attempted =
              objective_compliance_attempted;
          record.objective_compliance_success =
              objective_compliance_success;
          record.objective_compliance_spatial_variable_count =
              3 * std::max(piece_num_ - 1, 0);
          record.objective_compliance_evaluation_count =
              objective_compliance_evaluation_count;
          record.objective_compliance_regularized_eigenvalue_count =
              objective_compliance_regularized_eigenvalue_count;
          record.objective_compliance_ms = objective_compliance_ms;
          record.objective_compliance_raw_min_eigenvalue =
              objective_compliance_raw_min_eigenvalue;
          record.objective_compliance_raw_max_eigenvalue =
              objective_compliance_raw_max_eigenvalue;
          record.objective_compliance_regularized_condition_number =
              objective_compliance_regularized_condition_number;
          record.objective_compliance_reason =
              objective_compliance_reason;
          record.direct_spatial_variable_count =
              3 * std::max(piece_num_ - 1, 0);
          record.used_direction_mode = -1;
          for (const tf_sfc::Corridor &corridor : logged_corridors)
          {
            const tf_sfc::CorridorMetrics &metrics = corridor.metrics;
            if (metrics.valid && metrics.used_direction_mode >= 0)
            {
              if (record.used_direction_mode < 0)
              {
                record.used_direction_mode = metrics.used_direction_mode;
              }
              else if (record.used_direction_mode !=
                       metrics.used_direction_mode)
              {
                // -2 denotes a mixed-mode corridor set.
                record.used_direction_mode = -2;
              }
            }
            if (metrics.valid)
            {
              record.face_sample_pairs_per_evaluation +=
                  metrics.face_count *
                  (std::max(tf_sfc_parameters_.samples_per_piece, 2) + 1);
            }
          }
          record.trajectory_repair_enabled = trajectory_repair_enabled;
          record.trajectory_repair_attempt_count =
              trajectory_repair_attempt_count;
          record.trajectory_repair_accept_count =
              trajectory_repair_accept_count;
          record.trajectory_repair_piece_id =
              trajectory_repair_result.piece_id;
          record.trajectory_repair_candidate_face_count =
              trajectory_repair_result.candidate_face_count;
          record.trajectory_repair_ms = trajectory_repair_ms;
          record.trajectory_repair_global_violation_before_m =
              trajectory_repair_result.global_violation_before_m;
          record.trajectory_repair_global_violation_after_m =
              trajectory_repair_result.global_violation_after_m;
          record.trajectory_repair_piece_violation_before_m =
              trajectory_repair_result.piece_violation_before_m;
          record.trajectory_repair_piece_violation_after_m =
              trajectory_repair_result.piece_violation_after_m;
          record.trajectory_repair_overlap_previous_m =
              trajectory_repair_result.overlap_previous_m;
          record.trajectory_repair_overlap_next_m =
              trajectory_repair_result.overlap_next_m;
          record.trajectory_repair_seed_fallback_used =
              trajectory_repair_result.seed_fallback_used;
          record.trajectory_repair_geometric_acceptance_used =
              trajectory_repair_result.geometric_acceptance_used;
          record.trajectory_repair_candidate_weighted_width_m =
              trajectory_repair_result.candidate_weighted_width_m;
          record.trajectory_repair_sample_source =
              trajectory_repair_result.sample_source;
          record.trajectory_repair_overlap_anchor_source =
              trajectory_repair_result.overlap_anchor_source;
          record.trajectory_repair_primary_failure_reason =
              trajectory_repair_result.primary_failure_reason;
          record.trajectory_repair_reason = trajectory_repair_result.reason;
          record.post_optimization_repair_enabled =
              post_optimization_repair_enabled;
          record.post_optimization_repair_attempt_count =
              post_optimization_repair_attempt_count;
          record.post_optimization_repair_accept_count =
              post_optimization_repair_accept_count;
          record.post_optimization_repair_piece_id =
              post_optimization_repair_result.piece_id;
          record.post_optimization_repair_candidate_face_count =
              post_optimization_repair_result.candidate_face_count;
          record.post_optimization_repair_ms =
              post_optimization_repair_ms;
          record.post_optimization_repair_violation_before_m =
              post_optimization_repair_result.global_violation_before_m;
          record.post_optimization_repair_violation_after_m =
              post_optimization_repair_result.global_violation_after_m;
          record.post_optimization_repair_geometric_acceptance_used =
              post_optimization_repair_result.geometric_acceptance_used;
          record.post_optimization_repair_candidate_weighted_width_m =
              post_optimization_repair_result.candidate_weighted_width_m;
          record.post_optimization_repair_junction_reencode_shift_m =
              post_optimization_repair_result.junction_reencode_shift_m;
          record.post_optimization_repair_sample_source =
              post_optimization_repair_result.sample_source;
          record.post_optimization_repair_overlap_anchor_source =
              post_optimization_repair_result.overlap_anchor_source;
          record.post_optimization_repair_reason =
              post_optimization_repair_result.reason;
        };

    // Preparision 2: Trajectory related params
    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();
    jerkOpt_.reset(iniState, finState, piece_num_);

    Eigen::MatrixXd guidedInnerPts = initInnerPts;
    tf_sfc::PointVector active_seed_path;
    tf_corridors_.clear();
    hard_corridor_parameterization_.clear();
    if (tf_sfc_manager_)
    {
      tf_sfc_manager_->setCorridorPenaltyScale(1.0);
    }
    const auto prepareObjectiveCompliance =
        [&](const Eigen::MatrixXd &inner_points,
            const Eigen::VectorXd &durations) {
          if (tf_sfc_parameters_.direction_mode !=
              tf_sfc::DirectionMode::FULL_OBJECTIVE_COMPLIANCE)
          {
            return true;
          }
          objective_compliance_attempted = true;
          std::vector<Eigen::Matrix3d,
                      Eigen::aligned_allocator<Eigen::Matrix3d>>
              compliances;
          int evaluation_count = 0;
          int regularized_eigenvalue_count = 0;
          double raw_min_eigenvalue = 0.0;
          double raw_max_eigenvalue = 0.0;
          double regularized_condition_number = 0.0;
          std::string reason;
          const ros::WallTime metric_started = ros::WallTime::now();
          objective_compliance_success =
              computeFullObjectivePieceCompliances(
                  inner_points, durations, compliances,
                  evaluation_count, regularized_eigenvalue_count,
                  raw_min_eigenvalue, raw_max_eigenvalue,
                  regularized_condition_number, reason);
          objective_compliance_evaluation_count += evaluation_count;
          objective_compliance_regularized_eigenvalue_count +=
              regularized_eigenvalue_count;
          objective_compliance_raw_min_eigenvalue = raw_min_eigenvalue;
          objective_compliance_raw_max_eigenvalue = raw_max_eigenvalue;
          objective_compliance_regularized_condition_number =
              regularized_condition_number;
          objective_compliance_reason = reason;
          objective_compliance_ms +=
              (ros::WallTime::now() - metric_started).toSec() * 1000.0;
          tf_sfc_manager_->setPieceObjectiveCompliances(
              objective_compliance_success
                  ? compliances
                  : std::vector<
                        Eigen::Matrix3d,
                        Eigen::aligned_allocator<Eigen::Matrix3d>>());
          if (!objective_compliance_success)
          {
            ROS_ERROR_THROTTLE(
                1.0,
                "TF-SFC full-objective compliance failed (%s).",
                objective_compliance_reason.c_str());
          }
          return objective_compliance_success;
        };
    if (tf_sfc_parameters_.enabled && tf_sfc_manager_)
    {
      // TF-SFC is frozen during one optimization request. The explicit fallback
      // switch controls both generation failures and later EGO rebound retries.
      const ros::WallTime corridor_started = ros::WallTime::now();
      bool corridor_ok = false;
      if (requested_corridor_method == "ellipsoid_decomp")
      {
        tf_sfc::PointVector seed_path;
        int covered_piece_num = 0;
        tf_sfc::FailureReason uncovered_failure_reason =
            tf_sfc::FailureReason::NONE;
        tf_sfc::FailureReason seed_failure_reason = tf_sfc::FailureReason::NONE;
        const auto generate_ellipsoid_corridors =
            [&](const tf_sfc::PointVector &candidate_seed,
                const tf_sfc::FailureReason candidate_uncovered_reason) {
              const ros::WallTime inflation_started = ros::WallTime::now();
              corridor_generation_attempted = true;
              const bool generated = tf_sfc_manager_->generateEllipsoidDecomp(
                  candidate_seed, piece_num_, candidate_uncovered_reason,
                  tf_corridors_);
              tf_sfc_inflation_ms +=
                  (ros::WallTime::now() - inflation_started).toSec() * 1000.0;
              return generated;
            };
        if (!tf_sfc_manager_->ellipsoidDecompAvailable())
        {
          tf_sfc::Corridor failed;
          failed.metrics.piece_id = 0;
          failed.metrics.failure_reason = tf_sfc::FailureReason::DECOMP_UTIL_UNAVAILABLE;
          tf_corridors_.push_back(failed);
        }
        else if (!buildFixedPieceSeedPathWithClearanceRetry(
                     grid_map_, a_star_, iniState.col(0), iniState.col(1),
                     finState.col(0), piece_num_,
                     tf_sfc_parameters_.allow_partial_corridors,
                     tf_sfc_parameters_.min_valid_pieces,
                     tf_sfc_parameters_.min_overlap_radius,
                     tf_sfc_parameters_.junction_refine_radius,
                     tf_sfc_parameters_.junction_refine_step,
                     tf_sfc_parameters_.decomp_initial_velocity_segment,
                     tf_sfc_parameters_.decomp_initial_velocity_threshold,
                     tf_sfc_parameters_.decomp_degenerate_seed_length,
                     tf_sfc_parameters_
                         .seed_retry_without_velocity_on_clearance_failure,
                     tf_sfc_parameters_.seed_clearance_astar_enabled,
                     tf_sfc_parameters_.seed_clearance_astar_time_limit,
                     seed_path, covered_piece_num, uncovered_failure_reason,
                     seed_failure_reason, seed_path_build_info))
        {
          tf_sfc::Corridor failed;
          failed.metrics.piece_id = 0;
          failed.metrics.failure_reason = seed_failure_reason;
          tf_corridors_.push_back(failed);
        }
        else
        {
          for (int point_id = 1; point_id <= covered_piece_num &&
                                 point_id < piece_num_;
               ++point_id)
          {
            guidedInnerPts.col(point_id - 1) = seed_path[point_id];
          }
          corridor_ok = generate_ellipsoid_corridors(
              seed_path, uncovered_failure_reason);
          tf_sfc::FailureReason first_generation_failure =
              tf_sfc::FailureReason::NONE;
          int first_generation_failure_point_id = -1;
          for (const tf_sfc::Corridor &corridor : tf_corridors_)
          {
            if (!corridor.metrics.valid)
            {
              first_generation_failure = corridor.metrics.failure_reason;
              first_generation_failure_point_id = corridor.metrics.piece_id;
              break;
            }
          }
          if (!corridor_ok &&
              tf_sfc_parameters_.decomp_retry_seed_validation_without_velocity &&
              seed_path_build_info.initial_velocity_seed_used &&
              first_generation_failure ==
                  tf_sfc::FailureReason::SEED_PATH_OCCUPIED)
          {
            // The velocity prefix can be valid when it is proposed but stale
            // or conservatively occupied when DecompUtil consumes the final
            // subdivided seed. Rebuild the complete seed from the current
            // state with ordinary A* once instead of rejecting immediately.
            ROS_WARN(
                "TF-SFC velocity-aligned seed failed final occupancy "
                "validation at point %d; retrying once with ordinary A*.",
                first_generation_failure_point_id);
            tf_sfc::PointVector fallback_seed_path;
            int fallback_covered_piece_num = 0;
            tf_sfc::FailureReason fallback_uncovered_failure_reason =
                tf_sfc::FailureReason::NONE;
            tf_sfc::FailureReason fallback_seed_failure_reason =
                tf_sfc::FailureReason::NONE;
            SeedPathBuildInfo fallback_build_info;
            const bool fallback_seed_ok = buildFixedPieceSeedPath(
                grid_map_, a_star_, iniState.col(0), iniState.col(1),
                finState.col(0), piece_num_,
                tf_sfc_parameters_.allow_partial_corridors,
                tf_sfc_parameters_.min_valid_pieces,
                tf_sfc_parameters_.min_overlap_radius,
                tf_sfc_parameters_.junction_refine_radius,
                tf_sfc_parameters_.junction_refine_step,
                0.0,
                tf_sfc_parameters_.decomp_initial_velocity_threshold,
                tf_sfc_parameters_.decomp_degenerate_seed_length,
                false, tf_sfc_parameters_.seed_clearance_astar_time_limit,
                fallback_seed_path, fallback_covered_piece_num,
                fallback_uncovered_failure_reason,
                fallback_seed_failure_reason, fallback_build_info);
            fallback_build_info.initial_velocity_seed_attempted = true;
            fallback_build_info.initial_velocity_seed_used = false;
            fallback_build_info.velocity_seed_fallback_used = true;
            fallback_build_info.velocity_seed_fallback_reason =
                tf_sfc::failureReasonName(first_generation_failure);
            fallback_build_info.seed_validation_failure_point_id =
                first_generation_failure_point_id;
            fallback_build_info.astar_search_attempted =
                fallback_build_info.astar_search_attempted ||
                seed_path_build_info.astar_search_attempted;
            fallback_build_info.astar_search_success =
                fallback_build_info.astar_search_success ||
                seed_path_build_info.astar_search_success;
            fallback_build_info.astar_search_call_count +=
                seed_path_build_info.astar_search_call_count;
            fallback_build_info.astar_search_ms +=
                seed_path_build_info.astar_search_ms;
            fallback_build_info.seed_path_build_ms +=
                seed_path_build_info.seed_path_build_ms;
            if (fallback_seed_ok)
            {
              fallback_build_info.strategy =
                  "velocity_validation_fallback_" +
                  fallback_build_info.strategy;
              seed_path_build_info = fallback_build_info;
              seed_path = fallback_seed_path;
              covered_piece_num = fallback_covered_piece_num;
              uncovered_failure_reason =
                  fallback_uncovered_failure_reason;
              guidedInnerPts = initInnerPts;
              for (int point_id = 1;
                   point_id <= covered_piece_num && point_id < piece_num_;
                   ++point_id)
              {
                guidedInnerPts.col(point_id - 1) = seed_path[point_id];
              }
              corridor_ok = generate_ellipsoid_corridors(
                  seed_path, uncovered_failure_reason);
              ROS_INFO("TF-SFC ordinary-A* seed retry %s.",
                       corridor_ok ? "succeeded" : "failed");
            }
            else
            {
              fallback_build_info.strategy =
                  "velocity_validation_fallback_failed";
              seed_path_build_info = fallback_build_info;
              tf_corridors_.clear();
              tf_sfc::Corridor failed;
              failed.metrics.piece_id = 0;
              failed.metrics.failure_reason = fallback_seed_failure_reason;
              tf_corridors_.push_back(failed);
              ROS_WARN("TF-SFC ordinary-A* seed retry could not build a "
                       "valid seed (%s).",
                       tf_sfc::failureReasonName(
                           fallback_seed_failure_reason));
            }
          }
        }
      }
      else if (requested_corridor_method == "obb" ||
               requested_corridor_method == "tf_sfc")
      {
        // A convex collision-free corridor cannot contain a seed segment that
        // already crosses inflated occupancy.  GCOPTER naturally receives an
        // RRT* route; EGO's ordinary initial MINCO trajectory is not guaranteed
        // to be collision-free because rebound optimization normally repairs it.
        // Reuse the same existing A* seed builder as the Liu baseline so OBB and
        // proposed TF-SFC differ only in corridor construction, not in search.
        tf_sfc::PointVector seed_path;
        int covered_piece_num = 0;
        tf_sfc::FailureReason uncovered_failure_reason =
            tf_sfc::FailureReason::NONE;
        tf_sfc::FailureReason seed_failure_reason =
            tf_sfc::FailureReason::NONE;
        if (!buildFixedPieceSeedPathWithClearanceRetry(
                grid_map_, a_star_, iniState.col(0), iniState.col(1),
                finState.col(0), piece_num_,
                tf_sfc_parameters_.allow_partial_corridors,
                tf_sfc_parameters_.min_valid_pieces,
                tf_sfc_parameters_.min_overlap_radius,
                tf_sfc_parameters_.junction_refine_radius,
                tf_sfc_parameters_.junction_refine_step,
                tf_sfc_parameters_.decomp_initial_velocity_segment,
                tf_sfc_parameters_.decomp_initial_velocity_threshold,
                tf_sfc_parameters_.decomp_degenerate_seed_length,
                tf_sfc_parameters_
                    .seed_retry_without_velocity_on_clearance_failure,
                tf_sfc_parameters_.seed_clearance_astar_enabled,
                tf_sfc_parameters_.seed_clearance_astar_time_limit,
                seed_path, covered_piece_num, uncovered_failure_reason,
                seed_failure_reason, seed_path_build_info))
        {
          tf_sfc::Corridor failed;
          failed.metrics.piece_id = 0;
          failed.metrics.failure_reason = seed_failure_reason;
          tf_corridors_.push_back(failed);
        }
        else
        {
          for (int point_id = 1;
               point_id <= covered_piece_num && point_id < piece_num_;
               ++point_id)
          {
            guidedInnerPts.col(point_id - 1) = seed_path[point_id];
          }
          jerkOpt_.generate(guidedInnerPts, initT);
          prepareObjectiveCompliance(guidedInnerPts, initT);
          const ros::WallTime inflation_started = ros::WallTime::now();
          corridor_generation_attempted = true;
          corridor_ok = tf_sfc_manager_->generateFromSeedPath(
              jerkOpt_.getTraj(), seed_path, uncovered_failure_reason,
              tf_corridors_);
          active_seed_path = seed_path;
          tf_sfc_inflation_ms +=
              (ros::WallTime::now() - inflation_started).toSec() * 1000.0;

          tf_sfc::FailureReason first_generation_failure =
              tf_sfc::FailureReason::NONE;
          int first_generation_failure_point_id = -1;
          for (const tf_sfc::Corridor &corridor : tf_corridors_)
          {
            if (!corridor.metrics.valid)
            {
              first_generation_failure = corridor.metrics.failure_reason;
              first_generation_failure_point_id =
                  corridor.metrics.piece_id;
              break;
            }
          }
          const bool retryable_geometry_failure =
              first_generation_failure ==
                  tf_sfc::FailureReason::INITIAL_OBB_OCCUPIED ||
              first_generation_failure ==
                  tf_sfc::FailureReason::OBSTACLE_SEPARATION_FAILURE ||
              first_generation_failure ==
                  tf_sfc::FailureReason::FACE_BUDGET_EXHAUSTED;
          if (!corridor_ok &&
              tf_sfc_parameters_
                  .decomp_retry_seed_validation_without_velocity &&
              seed_path_build_info.initial_velocity_seed_used &&
              retryable_geometry_failure)
          {
            ROS_WARN(
                "TF-SFC velocity-aligned seed failed corridor construction "
                "at piece %d (%s); retrying once with ordinary A*.",
                first_generation_failure_point_id,
                tf_sfc::failureReasonName(first_generation_failure));
            tf_sfc::PointVector fallback_seed_path;
            int fallback_covered_piece_num = 0;
            tf_sfc::FailureReason fallback_uncovered_failure_reason =
                tf_sfc::FailureReason::NONE;
            tf_sfc::FailureReason fallback_seed_failure_reason =
                tf_sfc::FailureReason::NONE;
            SeedPathBuildInfo fallback_build_info;
            const bool fallback_seed_ok = buildFixedPieceSeedPath(
                grid_map_, a_star_, iniState.col(0), iniState.col(1),
                finState.col(0), piece_num_,
                tf_sfc_parameters_.allow_partial_corridors,
                tf_sfc_parameters_.min_valid_pieces,
                tf_sfc_parameters_.min_overlap_radius,
                tf_sfc_parameters_.junction_refine_radius,
                tf_sfc_parameters_.junction_refine_step, 0.0,
                tf_sfc_parameters_.decomp_initial_velocity_threshold,
                tf_sfc_parameters_.decomp_degenerate_seed_length,
                false, tf_sfc_parameters_.seed_clearance_astar_time_limit,
                fallback_seed_path, fallback_covered_piece_num,
                fallback_uncovered_failure_reason,
                fallback_seed_failure_reason, fallback_build_info);
            fallback_build_info.initial_velocity_seed_attempted = true;
            fallback_build_info.initial_velocity_seed_used = false;
            fallback_build_info.velocity_seed_fallback_used = true;
            fallback_build_info.velocity_seed_fallback_reason =
                tf_sfc::failureReasonName(first_generation_failure);
            fallback_build_info.seed_validation_failure_point_id =
                first_generation_failure_point_id;
            fallback_build_info.astar_search_attempted =
                fallback_build_info.astar_search_attempted ||
                seed_path_build_info.astar_search_attempted;
            fallback_build_info.astar_search_success =
                fallback_build_info.astar_search_success ||
                seed_path_build_info.astar_search_success;
            fallback_build_info.astar_search_call_count +=
                seed_path_build_info.astar_search_call_count;
            fallback_build_info.astar_search_ms +=
                seed_path_build_info.astar_search_ms;
            fallback_build_info.seed_path_build_ms +=
                seed_path_build_info.seed_path_build_ms;
            if (fallback_seed_ok)
            {
              fallback_build_info.strategy =
                  "corridor_geometry_fallback_" +
                  fallback_build_info.strategy;
              seed_path_build_info = fallback_build_info;
              seed_path = fallback_seed_path;
              covered_piece_num = fallback_covered_piece_num;
              uncovered_failure_reason =
                  fallback_uncovered_failure_reason;
              guidedInnerPts = initInnerPts;
              for (int point_id = 1;
                   point_id <= covered_piece_num && point_id < piece_num_;
                   ++point_id)
              {
                guidedInnerPts.col(point_id - 1) =
                    seed_path[point_id];
              }
              jerkOpt_.generate(guidedInnerPts, initT);
              prepareObjectiveCompliance(guidedInnerPts, initT);
              tf_corridors_.clear();
              const ros::WallTime retry_inflation_started =
                  ros::WallTime::now();
              corridor_generation_attempted = true;
              corridor_ok = tf_sfc_manager_->generateFromSeedPath(
                  jerkOpt_.getTraj(), fallback_seed_path,
                  fallback_uncovered_failure_reason, tf_corridors_);
              active_seed_path = fallback_seed_path;
              tf_sfc_inflation_ms +=
                  (ros::WallTime::now() - retry_inflation_started)
                      .toSec() *
                  1000.0;
              ROS_INFO("TF-SFC ordinary-A* geometry retry %s.",
                       corridor_ok ? "succeeded" : "failed");
            }
            else
            {
              fallback_build_info.strategy =
                  "corridor_geometry_fallback_failed";
              seed_path_build_info = fallback_build_info;
              tf_corridors_.clear();
              tf_sfc::Corridor failed;
              failed.metrics.piece_id = 0;
              failed.metrics.failure_reason =
                  fallback_seed_failure_reason;
              tf_corridors_.push_back(failed);
              ROS_WARN(
                  "TF-SFC ordinary-A* geometry retry could not build a "
                  "valid seed (%s).",
                  tf_sfc::failureReasonName(
                      fallback_seed_failure_reason));
            }
          }
        }
      }
      else
      {
        tf_sfc::Corridor failed;
        failed.metrics.piece_id = 0;
        failed.metrics.failure_reason = tf_sfc::FailureReason::INVALID_INPUT;
        tf_corridors_.push_back(failed);
      }
      // Keep a valid trajectory object for diagnostics even when corridor
      // generation is rejected before the optimizer starts.
      jerkOpt_.generate(guidedInnerPts, initT);
      if (corridor_ok && trajectory_repair_enabled)
      {
        const int max_repair_passes =
            std::max(tf_sfc_parameters_.trajectory_repair_max_passes, 0);
        for (int pass = 0; pass < max_repair_passes; ++pass)
        {
          tf_sfc::TrajectoryRepairResult pass_result;
          const bool accepted =
              tf_sfc_manager_->repairWorstCorridorForTrajectory(
                  jerkOpt_.getTraj(), active_seed_path, tf_corridors_,
                  pass_result);
          trajectory_repair_result = pass_result;
          trajectory_repair_ms += pass_result.generation_time_ms;
          tf_sfc_inflation_ms += pass_result.generation_time_ms;
          if (pass_result.attempted)
          {
            ++trajectory_repair_attempt_count;
          }
          if (accepted)
          {
            ++trajectory_repair_accept_count;
            ROS_INFO(
                "TF-SFC trajectory repair accepted for piece %d: "
                "global %.6f -> %.6f m, piece %.6f -> %.6f m, "
                "source=%s, faces=%d.",
                pass_result.piece_id,
                pass_result.global_violation_before_m,
                pass_result.global_violation_after_m,
                pass_result.piece_violation_before_m,
                pass_result.piece_violation_after_m,
                pass_result.sample_source.c_str(),
                pass_result.candidate_face_count);
          }
          else if (pass_result.attempted)
          {
            ROS_WARN(
                "TF-SFC trajectory repair rejected for piece %d (%s): "
                "global %.6f -> %.6f m, piece %.6f -> %.6f m, source=%s.",
                pass_result.piece_id, pass_result.reason.c_str(),
                pass_result.global_violation_before_m,
                pass_result.global_violation_after_m,
                pass_result.piece_violation_before_m,
                pass_result.piece_violation_after_m,
                pass_result.sample_source.c_str());
          }
          if (!accepted ||
              pass_result.global_violation_after_m <=
                  tf_sfc_parameters_.trajectory_repair_trigger)
          {
            break;
          }
        }
      }
      if (corridor_ok)
      {
        if (tf_sfc_parameters_.use_projection)
        {
          Eigen::MatrixXd projectedInnerPts = guidedInnerPts;
          if (tf_sfc_manager_->projectJunctions(projectedInnerPts, tf_corridors_))
          {
            guidedInnerPts = projectedInnerPts;
            projection_applied = true;
          }
        }
        if (tf_sfc_parameters_.hard_corridor_parameterization)
        {
          hard_parameterization_active =
              hard_corridor_parameterization_.configure(
                  tf_corridors_, piece_num_, tf_sfc_parameters_.hard_max_vertices,
                  tf_sfc_parameters_.hard_vertex_tolerance);
          hard_parameterization_active_ = hard_parameterization_active;
          if (!hard_parameterization_active)
          {
            tf_sfc::Corridor failed;
            failed.metrics.piece_id = 0;
            failed.metrics.failure_reason =
                tf_sfc::FailureReason::HARD_PARAMETERIZATION_FAILURE;
            tf_corridors_.push_back(failed);
            corridor_ok = false;
          }
        }
        if (corridor_ok)
        {
          tf_sfc_generated = true;
          max_junction_violation_initial_m =
              hard_parameterization_active
                  ? hard_corridor_parameterization_.maxJunctionViolation(
                        guidedInnerPts)
                  : 0.0;
        }
      }
      tf_sfc_generation_ms =
          (ros::WallTime::now() - corridor_started).toSec() * 1000.0;
      logged_corridors = tf_corridors_;
      publishTfSfcCorridors(logged_corridors);
      if (!corridor_ok)
      {
        if (!tf_sfc_parameters_.allow_ego_fallback)
        {
          tf_sfc::ExperimentRunRecord record;
          record.run_id = tf_sfc_experiment_logger_
                              ? tf_sfc_experiment_logger_->makeRunId(drone_id_)
                              : "";
          record.experiment_tag = tf_sfc_experiment_logger_
                                      ? tf_sfc_experiment_logger_->experimentTag()
                                      : tf_sfc_parameters_.experiment_tag;
          record.status = "tf_sfc_generation_failure";
          record.requested_method = requested_corridor_method;
          record.method = requested_corridor_method;
          record.timestamp_s = t0.toSec();
          record.drone_id = drone_id_;
          record.goal_id = experiment_goal_id_;
          record.replan_id = experiment_replan_id_;
          record.attempt_id = experiment_attempt_id_;
          record.touch_goal = touch_goal_;
          record.seed_path_strategy = seed_path_build_info.strategy;
          record.initial_velocity_seed_attempted =
              seed_path_build_info.initial_velocity_seed_attempted;
          record.initial_velocity_seed_used =
              seed_path_build_info.initial_velocity_seed_used;
          record.velocity_seed_fallback_used =
              seed_path_build_info.velocity_seed_fallback_used;
          record.velocity_seed_fallback_reason =
              seed_path_build_info.velocity_seed_fallback_reason;
          record.clearance_retry_attempted =
              seed_path_build_info.clearance_retry_attempted;
          record.clearance_retry_success =
              seed_path_build_info.clearance_retry_success;
          record.clearance_retry_initial_strategy =
              seed_path_build_info.clearance_retry_initial_strategy;
          record.clearance_retry_first_failure_point_id =
              seed_path_build_info.clearance_retry_first_failure_point_id;
          record.clearance_astar_attempted =
              seed_path_build_info.clearance_astar_attempted;
          record.clearance_astar_success =
              seed_path_build_info.clearance_astar_success;
          record.clearance_astar_call_count =
              seed_path_build_info.clearance_astar_call_count;
          record.clearance_astar_ms =
              seed_path_build_info.clearance_astar_ms;
          record.seed_validation_failure_point_id =
              seed_path_build_info.seed_validation_failure_point_id;
          populate_seed_record(record);
          record.tf_sfc_enabled = true;
          record.hard_parameterization_enabled =
              tf_sfc_parameters_.hard_corridor_parameterization;
          record.hard_parameterization_active = false;
          record.hard_total_junction_count = piece_num_ - 1;
          record.direction_mode = (requested_corridor_method == "obb" || requested_corridor_method == "tf_sfc")
                                      ? static_cast<int>(tf_sfc_parameters_.direction_mode)
                                      : -1;
          record.total_planning_ms = (ros::Time::now() - t0).toSec() * 1000.0;
          record.corridor_generation_ms = tf_sfc_generation_ms;
          record.piece_count = piece_num_;
          record.final_cost = std::numeric_limits<double>::quiet_NaN();
          record.trajectory_duration_s = jerkOpt_.getTraj().getTotalDuration();
          for (const tf_sfc::Corridor &corridor : logged_corridors)
          {
            const tf_sfc::CorridorMetrics &metrics = corridor.metrics;
            if (metrics.seed_containment_evaluated)
            {
              ++record.seed_containment_evaluated_count;
              record.seed_contained_corridor_count +=
                  metrics.seed_contained ? 1 : 0;
              record.max_seed_containment_violation_m = std::max(
                  record.max_seed_containment_violation_m,
                  metrics.seed_containment_max_violation_m);
            }
            if (metrics.valid)
            {
              ++record.corridor_count;
            }
            else
            {
              ++record.failed_piece_count;
              if (record.first_failure_reason == "none")
              {
                record.first_failure_reason =
                    tf_sfc::failureReasonName(corridor.metrics.failure_reason);
              }
            }
          }
          record.terminal_failure_reason = record.first_failure_reason;
          if (tf_sfc_experiment_logger_ && tf_sfc_experiment_logger_->enabled())
          {
            tf_sfc_experiment_logger_->log(record, logged_corridors);
          }
          const char *first_failure = "unknown";
          for (const tf_sfc::Corridor &corridor : logged_corridors)
          {
            if (!corridor.metrics.valid)
            {
              first_failure = tf_sfc::failureReasonName(
                  corridor.metrics.failure_reason);
              break;
            }
          }
          ROS_ERROR_THROTTLE(
              1.0,
              "TF-SFC generation failed (method=%s, first_failure=%s) and fallback is disabled; rejecting this plan.",
              requested_corridor_method.c_str(), first_failure);
          return false;
        }
        fallback_to_ego = true;
        tf_corridors_.clear();
        ROS_WARN_THROTTLE(1.0, "TF-SFC generation failed; using the original EGO optimization.");
      }
    }

    spatial_variable_num_ = hard_parameterization_active
                                ? hard_corridor_parameterization_.spatialVariableCount()
                                : 3 * (piece_num_ - 1);
    variable_num_ = spatial_variable_num_ + piece_num_;
    std::vector<double> x_init(variable_num_, 0.0);
    if (hard_parameterization_active)
    {
      Eigen::VectorXd spatial_initial;
      if (!hard_corridor_parameterization_.encode(guidedInnerPts,
                                                   spatial_initial))
      {
        ROS_ERROR("TF-SFC hard corridor parameterization could not encode the initial junctions.");
        return false;
      }
      Eigen::Map<Eigen::VectorXd>(x_init.data(), spatial_variable_num_) =
          spatial_initial;
      Eigen::MatrixXd decoded_initial;
      if (!hard_corridor_parameterization_.decode(spatial_initial,
                                                   decoded_initial))
      {
        ROS_ERROR("TF-SFC hard corridor parameterization could not decode its initial junctions.");
        return false;
      }
      guidedInnerPts = decoded_initial;
      max_junction_violation_initial_m =
          hard_corridor_parameterization_.maxJunctionViolation(
              decoded_initial);
    }
    else
    {
      memcpy(x_init.data(), guidedInnerPts.data(),
             guidedInnerPts.size() * sizeof(x_init[0]));
    }
    Eigen::Map<Eigen::VectorXd> Vt(x_init.data() + spatial_variable_num_,
                                   initT.size());
    RealT2VirtualT(initT, Vt);
    if (tf_sfc_generated && !fallback_to_ego && tf_sfc_manager_)
    {
      // Snapshot the exact hard-parameterized trajectory used to initialize
      // L-BFGS, not the preimage seed supplied to the inverse mapping.
      jerkOpt_.generate(guidedInnerPts, initT);
      initial_corridor_evaluation =
          tf_sfc_manager_->evaluateTrajectory(jerkOpt_.getTraj());
    }
    min_ellip_dist2_.resize(swarm_trajs_->size());

    std::vector<double> best_corridor_x(variable_num_);
    bool best_corridor_candidate_valid = false;
    double best_corridor_cost = std::numeric_limits<double>::infinity();
    auto regenerateTrajectoryFromDecision = [&](const double *decision) {
      Eigen::MatrixXd inner_points;
      if (hard_parameterization_active)
      {
        const Eigen::Map<const Eigen::VectorXd> spatial(decision,
                                                        spatial_variable_num_);
        if (!hard_corridor_parameterization_.decode(spatial, inner_points))
        {
          inner_points = Eigen::MatrixXd::Constant(
              3, piece_num_ - 1, std::numeric_limits<double>::quiet_NaN());
        }
      }
      else
      {
        inner_points = Eigen::Map<const Eigen::MatrixXd>(
            decision, 3, piece_num_ - 1);
      }
      Eigen::Map<const Eigen::VectorXd> virtual_times(
          decision + spatial_variable_num_, piece_num_);
      Eigen::VectorXd real_times(piece_num_);
      VirtualT2RealT(virtual_times, real_times);
      jerkOpt_.generate(inner_points, real_times);
    };
    auto trajectoryIsFinite = [&]() {
      const poly_traj::Trajectory &trajectory = jerkOpt_.getTraj();
      if (trajectory.getPieceNum() <= 0 ||
          !std::isfinite(trajectory.getTotalDuration()) ||
          trajectory.getTotalDuration() <= 0.0)
      {
        return false;
      }
      const int sample_count = std::max(
          2, tf_sfc_parameters_.samples_per_piece * trajectory.getPieceNum());
      for (int sample_id = 0; sample_id <= sample_count; ++sample_id)
      {
        const double time = trajectory.getTotalDuration() *
                            static_cast<double>(sample_id) /
                            static_cast<double>(sample_count);
        if (!trajectory.getPos(time).allFinite() ||
            !trajectory.getVel(time).allFinite() ||
            !trajectory.getAcc(time).allFinite())
        {
          return false;
        }
      }
      return true;
    };
    auto evaluateTrajectoryProgress = [&]() {
      progress_guard_evaluated = false;
      progress_guard_passed = true;
      progress_guard_reason = "disabled";
      min_initial_seed_progress_m = 0.0;
      max_target_axis_progress_drop_m = 0.0;
      max_target_overshoot_m = 0.0;
      if (!tf_sfc_parameters_.progress_guard_enabled ||
          !tf_sfc_generated || fallback_to_ego)
      {
        return true;
      }

      const poly_traj::Trajectory &trajectory = jerkOpt_.getTraj();
      const Eigen::Vector3d start = iniState.col(0);
      const Eigen::Vector3d target = finState.col(0);
      Eigen::Vector3d target_axis = target - start;
      const double target_distance = target_axis.norm();
      if (trajectory.getPieceNum() <= 0 || target_distance <= 1.0e-6)
      {
        progress_guard_reason = "degenerate_target";
        return true;
      }
      target_axis /= target_distance;
      Eigen::Vector3d seed_axis = target_axis;
      if (active_seed_path.size() >= 2 &&
          (active_seed_path[1] - active_seed_path[0]).norm() > 1.0e-6)
      {
        seed_axis =
            (active_seed_path[1] - active_seed_path[0]).normalized();
      }

      progress_guard_evaluated = true;
      progress_guard_reason = "accepted";
      const int sample_count = std::max(
          8, tf_sfc_parameters_.samples_per_piece *
                 trajectory.getPieceNum());
      const double duration = trajectory.getTotalDuration();
      const double initial_horizon = std::min(
          duration, tf_sfc_parameters_.progress_guard_horizon_s);
      double peak_target_progress = 0.0;
      min_initial_seed_progress_m =
          std::numeric_limits<double>::infinity();
      for (int sample_id = 0; sample_id <= sample_count; ++sample_id)
      {
        const double time = duration *
                            static_cast<double>(sample_id) /
                            static_cast<double>(sample_count);
        const Eigen::Vector3d displacement =
            trajectory.getPos(time) - start;
        const double target_progress = displacement.dot(target_axis);
        peak_target_progress = std::max(peak_target_progress,
                                        target_progress);
        max_target_axis_progress_drop_m = std::max(
            max_target_axis_progress_drop_m,
            peak_target_progress - target_progress);
        max_target_overshoot_m = std::max(
            max_target_overshoot_m,
            target_progress - target_distance);
        if (time <= initial_horizon + 1.0e-9)
        {
          min_initial_seed_progress_m = std::min(
              min_initial_seed_progress_m,
              displacement.dot(seed_axis));
        }
      }
      if (!std::isfinite(min_initial_seed_progress_m))
      {
        min_initial_seed_progress_m = 0.0;
      }
      if (min_initial_seed_progress_m <
          -tf_sfc_parameters_.max_initial_progress_regression)
      {
        progress_guard_passed = false;
        progress_guard_reason = "initial_seed_regression";
      }
      else if (max_target_axis_progress_drop_m >
               tf_sfc_parameters_.max_target_axis_progress_drop)
      {
        progress_guard_passed = false;
        progress_guard_reason = "target_axis_progress_drop";
      }
      else if (max_target_overshoot_m >
               tf_sfc_parameters_.max_target_overshoot)
      {
        progress_guard_passed = false;
        progress_guard_reason = "target_overshoot";
      }
      return progress_guard_passed;
    };
    auto rememberCorridorCandidate = [&](const double violation) {
      std::copy(x_init.begin(), x_init.end(), best_corridor_x.begin());
      best_corridor_candidate_valid = true;
      best_corridor_cost = final_cost;
      best_corridor_violation_m = violation;
      ++corridor_candidate_accept_count;
    };
    auto rollbackCorridorCandidate = [&](const std::string &reason) {
      if (!best_corridor_candidate_valid)
      {
        return false;
      }
      std::copy(best_corridor_x.begin(), best_corridor_x.end(), x_init.begin());
      regenerateTrajectoryFromDecision(x_init.data());
      final_cost = best_corridor_cost;
      corridor_rollback_applied = true;
      corridor_rollback_reason = reason;
      return true;
    };
    bool post_optimization_repair_active = false;
    auto tryPostOptimizationRepair =
        [&](const tf_sfc::CorridorEvaluation &evaluation) {
          if (!post_optimization_repair_enabled ||
              !hard_parameterization_active || !tf_sfc_manager_ ||
              active_seed_path.size() < 2 ||
              evaluation.max_violation_m <=
                  tf_sfc_parameters_.max_final_violation ||
              post_optimization_repair_attempt_count >=
                  std::max(
                      tf_sfc_parameters_
                          .post_optimization_repair_max_passes,
                      0))
          {
            return false;
          }

          const tf_sfc::CorridorVector previous_corridors = tf_corridors_;
          tf_sfc::TrajectoryRepairResult repair_result;
          const poly_traj::Trajectory repair_trajectory = jerkOpt_.getTraj();
          const bool corridor_repaired =
              tf_sfc_manager_->repairWorstCorridorForTrajectory(
                  repair_trajectory, active_seed_path, tf_corridors_,
                  repair_result);
          post_optimization_repair_result = repair_result;
          post_optimization_repair_ms += repair_result.generation_time_ms;
          tf_sfc_inflation_ms += repair_result.generation_time_ms;
          if (repair_result.attempted)
          {
            ++post_optimization_repair_attempt_count;
          }
          if (!corridor_repaired)
          {
            return false;
          }

          tf_sfc::HardCorridorParameterization repaired_parameterization;
          if (!repaired_parameterization.configure(
                  tf_corridors_, piece_num_,
                  tf_sfc_parameters_.hard_max_vertices,
                  tf_sfc_parameters_.hard_vertex_tolerance))
          {
            tf_corridors_ = previous_corridors;
            tf_sfc_manager_->setCorridors(previous_corridors);
            post_optimization_repair_result.accepted = false;
            post_optimization_repair_result.reason =
                "hard_parameterization_failure";
            return false;
          }

          const Eigen::MatrixXd positions = repair_trajectory.getPositions();
          if (positions.cols() != piece_num_ + 1)
          {
            tf_corridors_ = previous_corridors;
            tf_sfc_manager_->setCorridors(previous_corridors);
            post_optimization_repair_result.accepted = false;
            post_optimization_repair_result.reason =
                "trajectory_dimension_mismatch";
            return false;
          }
          const Eigen::MatrixXd current_inner_points =
              positions.middleCols(1, std::max(piece_num_ - 1, 0));
          Eigen::VectorXd repaired_spatial;
          Eigen::MatrixXd decoded_inner_points;
          if (!repaired_parameterization.encode(current_inner_points,
                                                 repaired_spatial) ||
              !repaired_parameterization.decode(repaired_spatial,
                                                 decoded_inner_points) ||
              decoded_inner_points.rows() != current_inner_points.rows() ||
              decoded_inner_points.cols() != current_inner_points.cols() ||
              (current_inner_points.size() > 0 &&
               (decoded_inner_points - current_inner_points)
                       .cwiseAbs()
                       .maxCoeff() > 1.0e-5))
          {
            tf_corridors_ = previous_corridors;
            tf_sfc_manager_->setCorridors(previous_corridors);
            post_optimization_repair_result.accepted = false;
            post_optimization_repair_result.reason =
                "hard_reencode_shift";
            return false;
          }

          const int repaired_spatial_variable_num =
              repaired_parameterization.spatialVariableCount();
          const int repaired_variable_num =
              repaired_spatial_variable_num + piece_num_;
          std::vector<double> repaired_decision(repaired_variable_num, 0.0);
          Eigen::Map<Eigen::VectorXd>(repaired_decision.data(),
                                      repaired_spatial_variable_num) =
              repaired_spatial;
          Eigen::Map<Eigen::VectorXd> repaired_virtual_times(
              repaired_decision.data() + repaired_spatial_variable_num,
              piece_num_);
          RealT2VirtualT(repair_trajectory.getDurations(),
                         repaired_virtual_times);

          hard_corridor_parameterization_ = repaired_parameterization;
          spatial_variable_num_ = repaired_spatial_variable_num;
          variable_num_ = repaired_variable_num;
          x_init.swap(repaired_decision);
          best_corridor_x.assign(variable_num_, 0.0);
          best_corridor_candidate_valid = false;
          best_corridor_cost = std::numeric_limits<double>::infinity();
          best_corridor_violation_m =
              std::numeric_limits<double>::infinity();
          corridor_penalty_scale = 1.0;
          tf_sfc_manager_->setCorridorPenaltyScale(corridor_penalty_scale);
          final_cost = std::numeric_limits<double>::infinity();
          post_optimization_repair_active = true;
          ++post_optimization_repair_accept_count;
          logged_corridors = tf_corridors_;
          publishTfSfcCorridors(logged_corridors);
          ROS_WARN(
              "TF-SFC bounded post-optimization repair accepted for piece "
              "%d: %.6f -> %.6f m, source=%s, faces=%d; re-optimizing "
              "once.",
              repair_result.piece_id,
              repair_result.global_violation_before_m,
              repair_result.global_violation_after_m,
              repair_result.sample_source.c_str(),
              repair_result.candidate_face_count);
          return true;
        };

    // Preparision 3: LBFGS related params
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    // lbfgs_params.abs_curv_cond = 0;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;
    do
    {
      /* ---------- prepare ---------- */
      iter_num_ = 0;
      flag_force_return = false;
      force_stop_type_ = DONT_STOP;
      flag_still_unsafe = false;
      flag_success = false;
      flag_swarm_too_close = false;
      final_collision_free = false;
      final_obstacle_collision = false;
      final_swarm_clearance_failure = false;
      terminal_failure_reason = "none";
      corridor_retry_requested = false;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init.data(),
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);

      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;
      optimizer_time_ms += time_ms;
      total_lbfgs_iterations += iter_num_;
      last_lbfgs_result = result;

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        flag_force_return = false;
        bool candidate_invalid = false;

        const Eigen::Map<const Eigen::VectorXd> decision_vector(
            x_init.data(), variable_num_);
        if (!decision_vector.allFinite())
        {
          if (rollbackCorridorCandidate("nonfinite_decision"))
          {
            final_collision_free = true;
          }
          strict_corridor_rejected = tf_sfc_generated && !fallback_to_ego;
          candidate_invalid = true;
          terminal_failure_reason = "nonfinite_decision";
          ROS_ERROR_THROTTLE(
              1.0,
              "Trajectory optimization produced a non-finite decision vector; "
              "rejecting the candidate%s.",
              corridor_rollback_applied ? " and restoring the best safe iterate" : "");
        }
        else
        {
          // The final callback evaluation is not guaranteed to coincide with
          // the decision vector returned by L-BFGS. Regenerate explicitly so
          // every safety and corridor check evaluates the exact candidate.
          regenerateTrajectoryFromDecision(x_init.data());
        }

        /* double check: fine collision check */
        std::vector<std::pair<int, int>> segments_nouse;
        for (size_t i = 0; i < swarm_trajs_->size(); ++i)
        {
          flag_swarm_too_close |= min_ellip_dist2_[i] < pow((swarm_clearance_ + swarm_trajs_->at(i).des_clearance) * 1.25, 2);
        }
        if (!candidate_invalid && !trajectoryIsFinite())
        {
          if (rollbackCorridorCandidate("nonfinite_trajectory"))
          {
            final_collision_free = true;
          }
          strict_corridor_rejected = tf_sfc_generated && !fallback_to_ego;
          candidate_invalid = true;
          terminal_failure_reason = "nonfinite_trajectory";
          ROS_ERROR_THROTTLE(
              1.0,
              "Trajectory optimization produced a non-finite trajectory; "
              "rejecting the candidate%s.",
              corridor_rollback_applied ? " and restoring the best safe iterate" : "");
        }
        else if (!candidate_invalid && !flag_swarm_too_close)
        {
          if (finelyCheckAndSetConstraintPoints(segments_nouse, jerkOpt_, false) == CHK_RET::OBS_FREE)
          {
            final_collision_free = true;
            if (!evaluateTrajectoryProgress())
            {
              terminal_failure_reason = progress_guard_reason;
              ROS_ERROR_THROTTLE(
                  1.0,
                  "TF-SFC progress guard rejected a collision-free "
                  "trajectory (%s): initial %.3f m, max drop %.3f m, "
                  "overshoot %.3f m.",
                  progress_guard_reason.c_str(),
                  min_initial_seed_progress_m,
                  max_target_axis_progress_drop_m,
                  max_target_overshoot_m);
            }
            const bool strict_check_enabled =
                progress_guard_passed && tf_sfc_generated &&
                !fallback_to_ego && tf_sfc_manager_ &&
                tf_sfc_parameters_.enforce_final_corridor;
            tf_sfc::CorridorEvaluation enforcement_evaluation;
            if (strict_check_enabled)
            {
              enforcement_evaluation =
                  tf_sfc_manager_->evaluateTrajectory(jerkOpt_.getTraj());
              ++corridor_candidate_count;

              const bool first_safe_candidate =
                  !best_corridor_candidate_valid;
              const bool within_tolerance =
                  enforcement_evaluation.max_violation_m <=
                  tf_sfc_parameters_.max_final_violation;
              const bool monotonically_improved =
                  enforcement_evaluation.max_violation_m +
                      tf_sfc_parameters_.enforcement_min_improvement <
                  best_corridor_violation_m;
              if (first_safe_candidate || within_tolerance ||
                  monotonically_improved)
              {
                rememberCorridorCandidate(
                    enforcement_evaluation.max_violation_m);
              }
              else
              {
                if (tryPostOptimizationRepair(enforcement_evaluation))
                {
                  corridor_retry_requested = true;
                  terminal_failure_reason = "none";
                }
                else
                {
                  rollbackCorridorCandidate("violation_not_improved");
                  strict_corridor_rejected = true;
                  final_collision_free = true;
                  terminal_failure_reason =
                      "corridor_violation_not_improved";
                  ROS_ERROR_THROTTLE(
                      1.0,
                      "TF-SFC continuation did not improve sampled violation "
                      "(candidate %.6f m, best %.6f m); restoring the best "
                      "safe iterate and rejecting this plan.",
                      enforcement_evaluation.max_violation_m,
                      best_corridor_violation_m);
                }
              }
            }

            if (!strict_corridor_rejected && !corridor_retry_requested &&
                strict_check_enabled &&
                enforcement_evaluation.max_violation_m >
                    tf_sfc_parameters_.max_final_violation)
            {
              const bool can_continue =
                  !post_optimization_repair_active &&
                  tf_sfc_parameters_.use_soft_penalty &&
                  tf_sfc_parameters_.weight > 0.0 &&
                  tf_sfc_parameters_.enforcement_weight_multiplier > 1.0 &&
                  corridor_enforcement_passes <
                      tf_sfc_parameters_.max_enforcement_passes;
              if (can_continue)
              {
                ++corridor_enforcement_passes;
                corridor_penalty_scale *=
                    tf_sfc_parameters_.enforcement_weight_multiplier;
                tf_sfc_manager_->setCorridorPenaltyScale(
                    corridor_penalty_scale);
                corridor_retry_requested = true;
                ROS_WARN(
                    "TF-SFC sampled final violation %.6f m exceeds %.6f m; "
                    "continuation pass %d/%d with corridor weight %.3f.",
                    enforcement_evaluation.max_violation_m,
                    tf_sfc_parameters_.max_final_violation,
                    corridor_enforcement_passes,
                    tf_sfc_parameters_.max_enforcement_passes,
                    tf_sfc_parameters_.weight * corridor_penalty_scale);
              }
              else
              {
                if (tryPostOptimizationRepair(enforcement_evaluation))
                {
                  corridor_retry_requested = true;
                  terminal_failure_reason = "none";
                }
                else
                {
                  rollbackCorridorCandidate("max_enforcement_passes");
                  strict_corridor_rejected = true;
                  terminal_failure_reason =
                      "corridor_violation_limit";
                  ROS_ERROR_THROTTLE(
                      1.0,
                      "TF-SFC sampled final violation %.6f m exceeds %.6f m "
                      "after %d continuation pass(es); rejecting this plan.",
                      enforcement_evaluation.max_violation_m,
                      tf_sfc_parameters_.max_final_violation,
                      corridor_enforcement_passes);
                }
              }
            }
            else if (progress_guard_passed &&
                     !strict_corridor_rejected &&
                     !corridor_retry_requested)
            {
              flag_success = true;
              PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
            }
          }
          else
          {
            if (post_optimization_repair_active)
            {
              strict_corridor_rejected = true;
              final_obstacle_collision = true;
              terminal_failure_reason =
                  "post_repair_obstacle_collision";
              ROS_ERROR_THROTTLE(
                  1.0,
                  "TF-SFC bounded post-repair optimization collided; "
                  "rejecting without another restart.");
              continue;
            }
            if (corridor_enforcement_passes > 0 &&
                rollbackCorridorCandidate("collision_candidate"))
            {
              final_collision_free = true;
              strict_corridor_rejected = true;
              terminal_failure_reason =
                  "continuation_obstacle_collision";
              ROS_ERROR_THROTTLE(
                  1.0,
                  "TF-SFC continuation produced a colliding candidate; "
                  "restoring the best safe iterate and rejecting this plan.");
              continue;
            }
            // A not-blank return value means collision to obstales
            if (tf_sfc_manager_ &&
                (!tf_sfc_generated ||
                 (tf_sfc_parameters_.allow_ego_fallback &&
                  !hard_parameterization_active)))
            {
              tf_sfc_manager_->clearCorridors();
              tf_corridors_.clear();
              fallback_to_ego = tf_sfc_generated;
            }
            flag_still_unsafe = true;
            final_obstacle_collision = true;
            terminal_failure_reason = "obstacle_collision";
            restart_nums++;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f, fine check collided, keep optimizing\n\033[0m", iter_num_, time_ms);
          }
        }
        else if (!candidate_invalid)
        {
          if (post_optimization_repair_active)
          {
            strict_corridor_rejected = true;
            final_swarm_clearance_failure = true;
            terminal_failure_reason =
                "post_repair_swarm_clearance";
            ROS_ERROR_THROTTLE(
                1.0,
                "TF-SFC bounded post-repair optimization violated swarm "
                "clearance; rejecting without another restart.");
            continue;
          }
          if (corridor_enforcement_passes > 0 &&
              rollbackCorridorCandidate("swarm_clearance_candidate"))
          {
            final_collision_free = true;
            strict_corridor_rejected = true;
            terminal_failure_reason =
                "continuation_swarm_clearance";
            ROS_ERROR_THROTTLE(
                1.0,
                "TF-SFC continuation violated swarm clearance; restoring the "
                "best safe iterate and rejecting this plan.");
            continue;
          }
          PRINTF_COND("Swarm clearance not satisfied, keep optimizing. iter=%d,time(ms)=%5.3f, wei_swarm_mod_=%f\n", iter_num_, time_ms, wei_swarm_mod_);
          flag_still_unsafe = true;
          final_swarm_clearance_failure = true;
          terminal_failure_reason = "swarm_clearance";
          restart_nums++;
          wei_swarm_mod_ *= 2;
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        if (post_optimization_repair_active)
        {
          strict_corridor_rejected = true;
          terminal_failure_reason = "post_repair_solver_cancelled";
          ROS_ERROR_THROTTLE(
              1.0,
              "TF-SFC bounded post-repair optimization was cancelled; "
              "rejecting without another rebound.");
          continue;
        }
        if (corridor_enforcement_passes > 0 &&
            rollbackCorridorCandidate("solver_cancelled"))
        {
          final_collision_free = true;
          strict_corridor_rejected = true;
          terminal_failure_reason = "continuation_solver_cancelled";
          ROS_ERROR_THROTTLE(
              1.0,
              "TF-SFC continuation was cancelled; restoring the best safe "
              "iterate and rejecting this plan.");
          continue;
        }
        if (tf_sfc_manager_ &&
            (!tf_sfc_generated ||
             (tf_sfc_parameters_.allow_ego_fallback &&
              !hard_parameterization_active)))
        {
          tf_sfc_manager_->clearCorridors();
          tf_corridors_.clear();
          fallback_to_ego = tf_sfc_generated;
        }
        flag_force_return = true;
        terminal_failure_reason = "solver_cancelled";
        rebound_times++;
        PRINTF_COND("iter=%d, time(ms)=%f, rebound\n", iter_num_, time_ms);
      }
      else
      {
        if (post_optimization_repair_active)
        {
          strict_corridor_rejected = true;
          terminal_failure_reason = "post_repair_solver_error";
          ROS_ERROR_THROTTLE(
              1.0,
              "TF-SFC bounded post-repair optimization failed; rejecting "
              "without another solver pass.");
          continue;
        }
        if (corridor_enforcement_passes > 0 &&
            rollbackCorridorCandidate("solver_error"))
        {
          final_collision_free = true;
          strict_corridor_rejected = true;
          terminal_failure_reason = "continuation_solver_error";
          ROS_ERROR_THROTTLE(
              1.0,
              "TF-SFC continuation failed in L-BFGS; restoring the best safe "
              "iterate and rejecting this plan.");
          continue;
        }
        PRINTF_COND("iter=%d, time(ms)=%f, error\n", iter_num_, time_ms);
        terminal_failure_reason = "solver_error";
        ROS_WARN_COND(VERBOSE_OUTPUT, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }

    } while ((flag_still_unsafe && restart_nums < 3) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20) ||
             corridor_retry_requested);

    if (hard_parameterization_active)
    {
      Eigen::MatrixXd final_junctions;
      const Eigen::Map<const Eigen::VectorXd> final_spatial(
          x_init.data(), spatial_variable_num_);
      if (hard_corridor_parameterization_.decode(final_spatial,
                                                  final_junctions))
      {
        max_junction_violation_final_m =
            hard_corridor_parameterization_.maxJunctionViolation(
                final_junctions);
      }
      else
      {
        max_junction_violation_final_m =
            std::numeric_limits<double>::infinity();
      }
    }

    if (tf_sfc_experiment_logger_ && tf_sfc_experiment_logger_->enabled())
    {
      tf_sfc::ExperimentRunRecord record;
      record.run_id = tf_sfc_experiment_logger_->makeRunId(drone_id_);
      record.experiment_tag = tf_sfc_experiment_logger_->experimentTag();
      record.status = flag_success ? "success" :
                      ((progress_guard_evaluated &&
                        !progress_guard_passed)
                           ? "progress_guard_failure" :
                       (strict_corridor_rejected ? "corridor_violation_failure" :
                       (last_lbfgs_result == lbfgs::LBFGSERR_CANCELED ? "rebound_limit" :
                        (final_obstacle_collision ? "obstacle_collision_failure" :
                         (final_swarm_clearance_failure ? "swarm_clearance_failure" :
                          "solver_failure")))));
      record.timestamp_s = t0.toSec();
      record.requested_method = requested_corridor_method;
      record.method = fallback_to_ego ? "ego" : requested_corridor_method;
      record.drone_id = drone_id_;
      record.goal_id = experiment_goal_id_;
      record.replan_id = experiment_replan_id_;
      record.attempt_id = experiment_attempt_id_;
      record.touch_goal = touch_goal_;
      record.seed_path_strategy = seed_path_build_info.strategy;
      record.initial_velocity_seed_attempted =
          seed_path_build_info.initial_velocity_seed_attempted;
      record.initial_velocity_seed_used =
          seed_path_build_info.initial_velocity_seed_used;
      record.velocity_seed_fallback_used =
          seed_path_build_info.velocity_seed_fallback_used;
      record.velocity_seed_fallback_reason =
          seed_path_build_info.velocity_seed_fallback_reason;
      record.clearance_retry_attempted =
          seed_path_build_info.clearance_retry_attempted;
      record.clearance_retry_success =
          seed_path_build_info.clearance_retry_success;
      record.clearance_retry_initial_strategy =
          seed_path_build_info.clearance_retry_initial_strategy;
      record.clearance_retry_first_failure_point_id =
          seed_path_build_info.clearance_retry_first_failure_point_id;
      record.clearance_astar_attempted =
          seed_path_build_info.clearance_astar_attempted;
      record.clearance_astar_success =
          seed_path_build_info.clearance_astar_success;
      record.clearance_astar_call_count =
          seed_path_build_info.clearance_astar_call_count;
      record.clearance_astar_ms =
          seed_path_build_info.clearance_astar_ms;
      record.seed_validation_failure_point_id =
          seed_path_build_info.seed_validation_failure_point_id;
      populate_seed_record(record);
      record.tf_sfc_enabled = tf_sfc_parameters_.enabled;
      record.direction_mode = (requested_corridor_method == "obb" || requested_corridor_method == "tf_sfc")
                                  ? static_cast<int>(tf_sfc_parameters_.direction_mode)
                                  : -1;
      record.success = flag_success;
      record.collision_free = final_collision_free;
      record.final_obstacle_collision = final_obstacle_collision;
      record.final_swarm_clearance_failure =
          final_swarm_clearance_failure;
      record.terminal_failure_reason = flag_success
                                           ? "none"
                                           : terminal_failure_reason;
      record.tf_sfc_generated = tf_sfc_generated;
      record.fallback_to_ego = fallback_to_ego;
      record.projection_applied = projection_applied;
      record.hard_parameterization_enabled =
          tf_sfc_parameters_.hard_corridor_parameterization;
      record.hard_parameterization_active = hard_parameterization_active;
      record.hard_constrained_junction_count =
          hard_parameterization_active
              ? hard_corridor_parameterization_.constrainedJunctionCount()
              : 0;
      record.hard_total_junction_count = piece_num_ - 1;
      record.hard_spatial_variable_count = spatial_variable_num_;
      if (record.hard_parameterization_active &&
          record.direct_spatial_variable_count > 0)
      {
        record.hard_spatial_variable_overhead_ratio =
            static_cast<double>(record.hard_spatial_variable_count) /
            static_cast<double>(record.direct_spatial_variable_count);
      }
      record.max_junction_violation_initial_m =
          max_junction_violation_initial_m;
      record.max_junction_violation_final_m =
          max_junction_violation_final_m;
      record.lbfgs_result = last_lbfgs_result;
      record.total_planning_ms = (ros::Time::now() - t0).toSec() * 1000.0;
      record.optimizer_ms = optimizer_time_ms;
      record.corridor_generation_ms = tf_sfc_generation_ms;
      record.lbfgs_iterations = total_lbfgs_iterations;
      record.restart_count = restart_nums;
      record.rebound_count = rebound_times;
      record.piece_count = piece_num_;
      record.final_cost = final_cost;
      const poly_traj::Trajectory &result_trajectory = jerkOpt_.getTraj();
      tf_sfc::CorridorEvaluation final_corridor_evaluation;
      if (tf_sfc_generated && !fallback_to_ego && tf_sfc_manager_)
      {
        final_corridor_evaluation =
            tf_sfc_manager_->evaluateTrajectory(result_trajectory);
      }
      record.corridor_constrained_piece_count =
          initial_corridor_evaluation.constrained_piece_count;
      record.corridor_penalty_cost_initial =
          initial_corridor_evaluation.penalty_cost;
      record.corridor_penalty_cost_final =
          final_corridor_evaluation.penalty_cost;
      record.max_corridor_violation_initial_m =
          initial_corridor_evaluation.max_violation_m;
      record.max_corridor_violation_final_m =
          final_corridor_evaluation.max_violation_m;
      record.corridor_penalty_samples_per_piece =
          tf_sfc_generated && !fallback_to_ego
              ? std::max(tf_sfc_parameters_.samples_per_piece, 2)
              : 0;
      record.initial_corridor_violating_face_sample_count =
          initial_corridor_evaluation.violating_face_sample_count;
      record.final_corridor_violating_face_sample_count =
          final_corridor_evaluation.violating_face_sample_count;
      record.max_corridor_violation_initial_piece_id =
          initial_corridor_evaluation.max_violation_piece_id;
      record.max_corridor_violation_initial_face_id =
          initial_corridor_evaluation.max_violation_face_id;
      record.max_corridor_violation_initial_sample_id =
          initial_corridor_evaluation.max_violation_sample_id;
      record.max_corridor_violation_initial_time_ratio =
          initial_corridor_evaluation.max_violation_time_ratio;
      record.max_corridor_violation_final_piece_id =
          final_corridor_evaluation.max_violation_piece_id;
      record.max_corridor_violation_final_face_id =
          final_corridor_evaluation.max_violation_face_id;
      record.max_corridor_violation_final_sample_id =
          final_corridor_evaluation.max_violation_sample_id;
      record.max_corridor_violation_final_time_ratio =
          final_corridor_evaluation.max_violation_time_ratio;
      record.final_corridor_enforcement_enabled =
          tf_sfc_generated && !fallback_to_ego &&
          tf_sfc_parameters_.enforce_final_corridor;
      record.max_final_violation_allowed_m =
          record.final_corridor_enforcement_enabled
              ? tf_sfc_parameters_.max_final_violation
              : 0.0;
      record.corridor_enforcement_passes = corridor_enforcement_passes;
      record.corridor_penalty_weight_initial =
          tf_sfc_generated && !fallback_to_ego
              ? tf_sfc_parameters_.weight
              : 0.0;
      record.corridor_penalty_weight_final =
          tf_sfc_generated && !fallback_to_ego
              ? tf_sfc_parameters_.weight * corridor_penalty_scale
              : 0.0;
      record.strict_corridor_rejected = strict_corridor_rejected;
      record.progress_guard_enabled =
          tf_sfc_parameters_.progress_guard_enabled;
      record.progress_guard_evaluated = progress_guard_evaluated;
      record.progress_guard_passed = progress_guard_passed;
      record.min_initial_seed_progress_m =
          min_initial_seed_progress_m;
      record.max_target_axis_progress_drop_m =
          max_target_axis_progress_drop_m;
      record.max_target_overshoot_m = max_target_overshoot_m;
      record.progress_guard_reason = progress_guard_reason;
      record.corridor_candidate_count = corridor_candidate_count;
      record.corridor_candidate_accept_count =
          corridor_candidate_accept_count;
      record.corridor_rollback_applied = corridor_rollback_applied;
      record.corridor_rollback_reason = corridor_rollback_reason;
      record.best_corridor_violation_m =
          std::isfinite(best_corridor_violation_m)
              ? best_corridor_violation_m
              : 0.0;
      record.trajectory_duration_s = result_trajectory.getPieceNum() > 0
                                         ? result_trajectory.getTotalDuration()
                                         : std::numeric_limits<double>::quiet_NaN();
      if (result_trajectory.getPieceNum() > 0)
      {
        const int length_samples = std::max(20, 20 * result_trajectory.getPieceNum());
        Eigen::Vector3d previous = result_trajectory.getPos(0.0);
        for (int i = 1; i <= length_samples; ++i)
        {
          const Eigen::Vector3d current = result_trajectory.getPos(
              record.trajectory_duration_s * static_cast<double>(i) /
              static_cast<double>(length_samples));
          record.trajectory_length_m_sampled += (current - previous).norm();
          previous = current;
        }
      }
      else
      {
        record.trajectory_length_m_sampled = std::numeric_limits<double>::quiet_NaN();
      }

      record.min_sample_slack = std::numeric_limits<double>::quiet_NaN();
      record.min_overlap_radius = std::numeric_limits<double>::quiet_NaN();
      record.mean_faces = std::numeric_limits<double>::quiet_NaN();
      record.mean_weighted_width = std::numeric_limits<double>::quiet_NaN();
      if (!logged_corridors.empty())
      {
        double total_weighted_width = 0.0;
        double min_sample_slack = std::numeric_limits<double>::infinity();
        double min_overlap_radius = std::numeric_limits<double>::infinity();
        int overlap_count = 0;
        for (const tf_sfc::Corridor &corridor : logged_corridors)
        {
          const tf_sfc::CorridorMetrics &metrics = corridor.metrics;
          if (metrics.seed_containment_evaluated)
          {
            ++record.seed_containment_evaluated_count;
            record.seed_contained_corridor_count +=
                metrics.seed_contained ? 1 : 0;
            record.max_seed_containment_violation_m = std::max(
                record.max_seed_containment_violation_m,
                metrics.seed_containment_max_violation_m);
          }
          if (!metrics.valid)
          {
            ++record.failed_piece_count;
            if (record.first_failure_reason == "none")
            {
              record.first_failure_reason = tf_sfc::failureReasonName(metrics.failure_reason);
            }
            continue;
          }
          ++record.corridor_count;
          record.total_faces += metrics.face_count;
          total_weighted_width += metrics.weighted_width;
          min_sample_slack = std::min(min_sample_slack, metrics.min_sample_slack);
          record.direction_fallback_count += metrics.direction_fallback ? 1 : 0;
          if (metrics.overlap_radius_to_next >= 0.0)
          {
            min_overlap_radius = std::min(min_overlap_radius,
                                          metrics.overlap_radius_to_next);
            ++overlap_count;
          }
        }
        if (record.corridor_count > 0)
        {
          record.mean_faces = static_cast<double>(record.total_faces) /
                              static_cast<double>(record.corridor_count);
          record.mean_weighted_width = total_weighted_width /
                                       static_cast<double>(record.corridor_count);
          record.min_sample_slack = min_sample_slack;
        }
        if (overlap_count > 0)
        {
          record.min_overlap_radius = min_overlap_radius;
        }
      }
      tf_sfc_experiment_logger_->log(record, logged_corridors);
    }

    return flag_success;
  }

  bool PolyTrajOptimizer::computePointsToCheck(
      poly_traj::Trajectory &traj,
      int id_cps_end, PtsChk_t &pts_check)
  {
    pts_check.clear();
    pts_check.resize(id_cps_end);
    const double RES = grid_map_->getResolution(), RES_2 = RES / 2;
    Eigen::VectorXd durations = traj.getDurations();
    Eigen::VectorXd t_seg_start(durations.size() + 1);
    t_seg_start(0) = 0;
    for (int i = 0; i < durations.size(); ++i)
      t_seg_start(i + 1) = t_seg_start(i) + durations(i);
    const double DURATION = durations.sum();
    double t = 0.0, t_step = min(RES / max_vel_, durations.minCoeff() / max(cps_num_prePiece_, 1) / 1.5);
    Eigen::Vector3d pt_last = traj.getPos(0.0);
    // pts_check[0].push_back(pt_last);
    int id_cps_curr = 0, id_piece_curr = 0;

    while (true)
    {
      if (t > DURATION)
      {
        if (touch_goal_ && pts_check.size() > 0)
        {
          while (pts_check.back().size() == 0)
          {
            pts_check.pop_back();
          }

          if (pts_check.size() <= 0)
          {
            ROS_ERROR("Failed to get points list to check (0x02). pts_check.size()=%d", (int)pts_check.size());
            return false;
          }
          else
          {
            return true;
          }
        }
        else
        {
          ROS_ERROR("Failed to get points list to check (0x01). touch_goal_=%d, pts_check.size()=%d", touch_goal_, (int)pts_check.size());
          pts_check.clear();
          return false;
        }
      }

      const double next_t_stp = t_seg_start(id_piece_curr) + durations(id_piece_curr) / cps_num_prePiece_ * ((id_cps_curr + 1) - cps_num_prePiece_ * id_piece_curr);
      if (t >= next_t_stp)
      {
        if (id_cps_curr + 1 >= cps_num_prePiece_ * (id_piece_curr + 1))
        {
          ++id_piece_curr;
        }
        if (++id_cps_curr >= id_cps_end)
        {
          break;
        }
      }

      Eigen::Vector3d pt = traj.getPos(t);
      if (t < 1e-5 || pts_check[id_cps_curr].size() == 0 || (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
      {
        pts_check[id_cps_curr].emplace_back(std::pair<double, Eigen::Vector3d>(t, pt));
        pt_last = pt;
      }

      t += t_step;
    }

    return true;
  }

  /* check collision and set {p,v} pairs to constrain points */
  PolyTrajOptimizer::CHK_RET PolyTrajOptimizer::finelyCheckAndSetConstraintPoints(
      std::vector<std::pair<int, int>> &segments,
      const poly_traj::MinJerkOpt &pt_data,
      const bool flag_first_init /*= true*/)
  {

    Eigen::MatrixXd init_points = pt_data.getInitConstraintPoints(cps_num_prePiece_);
    poly_traj::Trajectory traj = pt_data.getTraj();

    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    vector<std::pair<int, int>> segment_ids;
    constexpr int ENOUGH_INTERVAL = 2;
    int in_id = -1, out_id = -1;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_); // only check closed 2/3 points.

    PtsChk_t pts_check;
    if (!computePointsToCheck(traj, i_end, pts_check))
    {
      return CHK_RET::ERR;
    }

    for (int i = 0; i < i_end; ++i)
    {
      for (size_t j = 0; j < pts_check[i].size(); ++j)
      {
        occ = grid_map_->getInflateOccupancy(pts_check[i][j].second);

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == 0)
          {
            in_id = i;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i + 1;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id < 0 || out_id < 0)
          {
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
            return CHK_RET::ERR;
          }
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /* Collision free and return in advance */
    if (segment_ids.size() == 0)
    {
      return CHK_RET::OBS_FREE;
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // Search from back to head
      Eigen::Vector3d in(init_points.col(segment_ids[i].second)), out(init_points.col(segment_ids[i].first));
      ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
      if (ret == ASTAR_RET::SUCCESS)
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
      {
        segment_ids[i].second = segment_ids[i + 1].second;
        segment_ids.erase(segment_ids.begin() + i + 1);
        --i;
        ROS_WARN("A corner case 2, I have never exeam it.");
      }
      else
      {
        ROS_WARN_COND(VERBOSE_OUTPUT, "A-star error, force return!");
        return CHK_RET::ERR;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    segments = final_segment_ids;
    return CHK_RET::FINISH;
  }

  bool PolyTrajOptimizer::roughlyCheckConstraintPoints(void)
  {

    // int end_idx = cps_.cp_size - 1;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_); // only check closed 2/3 points.
    for (int i = 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.cp_size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.cp_size) // fail to get the obs free point
        {
          ROS_WARN("Local target in collision, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)), out(cps_.points.col(segment_ids[i].first));
        ASTAR_RET ret = a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          ROS_WARN("A corner case 2, I have never exeam it.");
        }
        else
        {
          ROS_ERROR_COND(VERBOSE_OUTPUT, "A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {

            last_Astar_id = Astar_id;

            if (val >= 0)
            {
              ++Astar_id; // Previous Astar search from back to head
              if (Astar_id >= (int)a_star_pathes[i].size())
              {
                break;
              }
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
              {
                break;
              }
            }

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN_COND(VERBOSE_OUTPUT, "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  bool PolyTrajOptimizer::allowRebound(void) //zxzxzx
  {
    // criterion 1
    if (iter_num_ < 3)
      return false;

    // criterion 2
    double min_product = 1;
    for (int i = 3; i <= cps_.points.cols() - 4; ++i) // ignore head and tail
    {
      double product = ((cps_.points.col(i) - cps_.points.col(i - 1)).normalized()).dot((cps_.points.col(i + 1) - cps_.points.col(i)).normalized());
      if (product < min_product)
      {
        min_product = product;
      }
    }
    if (min_product < 0.87) // 30 degree
      return false;

    // criterion 3
    if (multitopology_data_.use_multitopology_trajs)
    {
      if (!multitopology_data_.initial_obstacles_avoided)
      {
        bool avoided = true;
        for (int i = 1; i < cps_.points.cols() - 1; ++i)
        {
          if (cps_.base_point[i].size() > 0)
          {
            // Only adopts "0" since finelyCheckAndSetConstraintPoints() after one optimization can add more base_points.
            if ((cps_.points.col(i) - cps_.base_point[i][0]).dot(cps_.direction[i][0]) < 0)
            {
              avoided = false;
              break;
            }
          }
        }

        multitopology_data_.initial_obstacles_avoided = avoided;
      }

      if (!multitopology_data_.initial_obstacles_avoided)
      {
        return false;
      }
    }

    // all the criterion passed
    return true;
  }

  /* multi-topo support */
  std::vector<ConstraintPoints> PolyTrajOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;
    constexpr int VARIS = 2;
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
    std::vector<ConstraintPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.cp_size - 1)).norm() / (cps_.cp_size - 1);

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ConstraintPoints, ConstraintPoints>> RichInfoSegs;
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ConstraintPoints, ConstraintPoints> RichInfoOneSeg;
      ConstraintPoints RichInfoOneSeg_temp;
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.cp_size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.cp_size - 1; j++)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = RichInfoSegs[i].first.cp_size - 1; j >= 1; j--)
        {
          ;
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << obs_clearance_ << " cps.size=" << cps_.cp_size << endl;
            for (int temp_i = 0; temp_i < cps_.cp_size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                ROS_ERROR("Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ConstraintPoints> blank;
            return blank;
          }

          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
          {
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            if (l > l_upbound)
            {
              ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          else
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!");
            if (VERBOSE_OUTPUT)
              cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.cp_size)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.cp_size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else
      {
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!, 2");
          if (VERBOSE_OUTPUT)
            cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
    }

    // Step 2. Assemble each segment to make up the new control point sequence.
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ConstraintPoints cpsOneSample;
      cpsOneSample.resize_cp(cps_.cp_size);
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.cp_size)
      {

        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.cp_size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  }

  /* callbacks by the L-BFGS optimizer */
  bool PolyTrajOptimizer::evaluatePreCorridorSpatialObjective(
      const Eigen::MatrixXd &inner_points,
      const Eigen::VectorXd &durations,
      Eigen::MatrixXd &gradient,
      double &cost)
  {
    if (inner_points.rows() != 3 ||
        inner_points.cols() != piece_num_ - 1 ||
        durations.size() != piece_num_ || !inner_points.allFinite() ||
        !durations.allFinite() || (durations.array() <= 0.0).any() ||
        cps_num_prePiece_ < 2)
    {
      return false;
    }

    std::fill(min_ellip_dist2_.begin(), min_ellip_dist2_.end(),
              std::numeric_limits<double>::max());
    jerkOpt_.generate(inner_points, durations);
    Eigen::VectorXd gradient_time = Eigen::VectorXd::Zero(piece_num_);
    Eigen::VectorXd component_costs = Eigen::VectorXd::Zero(5);
    double smoothness_cost = 0.0;
    initAndGetSmoothnessGradCost2PT(gradient_time, smoothness_cost);
    addPVAJGradCost2CT(gradient_time, component_costs,
                       cps_num_prePiece_);
    gradient.resize(3, piece_num_ - 1);
    jerkOpt_.getGrad2TP(gradient_time, gradient);
    cost = smoothness_cost + component_costs.sum() +
           wei_time_ * durations.sum();
    return gradient.allFinite() && std::isfinite(cost);
  }

  bool PolyTrajOptimizer::computeFullObjectivePieceCompliances(
      const Eigen::MatrixXd &inner_points,
      const Eigen::VectorXd &durations,
      std::vector<Eigen::Matrix3d,
                  Eigen::aligned_allocator<Eigen::Matrix3d>> &compliances,
      int &evaluation_count,
      int &regularized_eigenvalue_count,
      double &raw_min_eigenvalue,
      double &raw_max_eigenvalue,
      double &regularized_condition_number,
      std::string &reason)
  {
    compliances.clear();
    evaluation_count = 0;
    regularized_eigenvalue_count = 0;
    raw_min_eigenvalue = 0.0;
    raw_max_eigenvalue = 0.0;
    regularized_condition_number = 0.0;
    reason = "invalid_input";
    const int spatial_dimension = static_cast<int>(inner_points.size());
    if (inner_points.rows() != 3 ||
        inner_points.cols() != piece_num_ - 1 ||
        spatial_dimension <= 0 || durations.size() != piece_num_)
    {
      return false;
    }

    const double finite_difference_step =
        tf_sfc_parameters_.objective_compliance_fd_step;
    if (!std::isfinite(finite_difference_step) ||
        finite_difference_step <= 0.0)
    {
      reason = "invalid_finite_difference_step";
      return false;
    }

    Eigen::MatrixXd hessian(spatial_dimension, spatial_dimension);
    std::vector<Eigen::MatrixXd> piece_position_jacobians(piece_num_);
    for (Eigen::MatrixXd &jacobian : piece_position_jacobians)
    {
      jacobian = Eigen::MatrixXd::Zero(3, spatial_dimension);
    }
    const auto representativePiecePositions = [&]() {
      Eigen::MatrixXd positions(3, piece_num_);
      const poly_traj::Trajectory &trajectory = jerkOpt_.getTraj();
      const int sample_intervals =
          std::max(tf_sfc_parameters_.samples_per_piece, 2);
      for (int piece_id = 0; piece_id < piece_num_; ++piece_id)
      {
        const poly_traj::Piece &piece = trajectory[piece_id];
        Eigen::Vector3d weighted_position = Eigen::Vector3d::Zero();
        double total_weight = 0.0;
        for (int sample_id = 0; sample_id <= sample_intervals;
             ++sample_id)
        {
          const double weight =
              (sample_id == 0 || sample_id == sample_intervals)
                  ? 0.5
                  : 1.0;
          weighted_position +=
              weight * piece.getPos(
                           piece.getDuration() *
                           static_cast<double>(sample_id) /
                           static_cast<double>(sample_intervals));
          total_weight += weight;
        }
        positions.col(piece_id) = weighted_position / total_weight;
      }
      return positions;
    };
    Eigen::MatrixXd positive_points = inner_points;
    Eigen::MatrixXd negative_points = inner_points;
    for (int variable_id = 0; variable_id < spatial_dimension;
         ++variable_id)
    {
      positive_points = inner_points;
      negative_points = inner_points;
      positive_points.data()[variable_id] += finite_difference_step;
      negative_points.data()[variable_id] -= finite_difference_step;
      Eigen::MatrixXd positive_gradient;
      Eigen::MatrixXd negative_gradient;
      double positive_cost = 0.0;
      double negative_cost = 0.0;
      const bool positive_ok = evaluatePreCorridorSpatialObjective(
          positive_points, durations, positive_gradient, positive_cost);
      ++evaluation_count;
      Eigen::MatrixXd positive_piece_positions;
      if (positive_ok)
      {
        positive_piece_positions = representativePiecePositions();
      }
      const bool negative_ok = evaluatePreCorridorSpatialObjective(
          negative_points, durations, negative_gradient, negative_cost);
      ++evaluation_count;
      Eigen::MatrixXd negative_piece_positions;
      if (negative_ok)
      {
        negative_piece_positions = representativePiecePositions();
      }
      if (!positive_ok || !negative_ok ||
          positive_gradient.size() != spatial_dimension ||
          negative_gradient.size() != spatial_dimension)
      {
        reason = "nonfinite_perturbed_objective";
        Eigen::MatrixXd restored_gradient;
        double restored_cost = 0.0;
        evaluatePreCorridorSpatialObjective(
            inner_points, durations, restored_gradient, restored_cost);
        ++evaluation_count;
        return false;
      }
      hessian.col(variable_id) =
          (Eigen::Map<const Eigen::VectorXd>(positive_gradient.data(),
                                             spatial_dimension) -
           Eigen::Map<const Eigen::VectorXd>(negative_gradient.data(),
                                             spatial_dimension)) /
          (2.0 * finite_difference_step);
      for (int piece_id = 0; piece_id < piece_num_; ++piece_id)
      {
        piece_position_jacobians[piece_id].col(variable_id) =
            (positive_piece_positions.col(piece_id) -
             negative_piece_positions.col(piece_id)) /
            (2.0 * finite_difference_step);
      }
    }

    Eigen::MatrixXd restored_gradient;
    double restored_cost = 0.0;
    const bool restored = evaluatePreCorridorSpatialObjective(
        inner_points, durations, restored_gradient, restored_cost);
    ++evaluation_count;
    if (!restored || !hessian.allFinite())
    {
      reason = "nonfinite_hessian";
      return false;
    }

    hessian = 0.5 * (hessian + hessian.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(hessian);
    if (solver.info() != Eigen::Success ||
        !solver.eigenvalues().allFinite())
    {
      reason = "hessian_eigendecomposition_failure";
      return false;
    }

    const Eigen::VectorXd raw_eigenvalues = solver.eigenvalues();
    raw_min_eigenvalue = raw_eigenvalues.minCoeff();
    raw_max_eigenvalue = raw_eigenvalues.maxCoeff();
    const double spectral_scale = std::max(
        std::max(std::abs(raw_min_eigenvalue),
                 std::abs(raw_max_eigenvalue)),
        tf_sfc_parameters_.objective_compliance_absolute_floor);
    const double eigenvalue_floor = std::max(
        tf_sfc_parameters_.objective_compliance_absolute_floor,
        tf_sfc_parameters_.objective_compliance_eigenvalue_floor_ratio *
            spectral_scale);
    // The pre-corridor objective is non-convex. A large negative eigenvalue
    // is high absolute curvature, not a nearly free deformation direction.
    // Using |lambda| before flooring therefore yields a positive local
    // absolute-curvature metric without promoting negative-curvature modes.
    Eigen::VectorXd regularized_eigenvalues =
        raw_eigenvalues.cwiseAbs();
    for (int eigenvalue_id = 0;
         eigenvalue_id < regularized_eigenvalues.size(); ++eigenvalue_id)
    {
      if (regularized_eigenvalues(eigenvalue_id) < eigenvalue_floor)
      {
        regularized_eigenvalues(eigenvalue_id) = eigenvalue_floor;
        ++regularized_eigenvalue_count;
      }
    }
    regularized_condition_number =
        regularized_eigenvalues.maxCoeff() /
        regularized_eigenvalues.minCoeff();
    const Eigen::MatrixXd inverse_hessian =
        solver.eigenvectors() *
        regularized_eigenvalues.cwiseInverse().asDiagonal() *
        solver.eigenvectors().transpose();
    if (!inverse_hessian.allFinite())
    {
      reason = "nonfinite_inverse_hessian";
      return false;
    }

    compliances.resize(piece_num_);
    for (int piece_id = 0; piece_id < piece_num_; ++piece_id)
    {
      if (!piece_position_jacobians[piece_id].allFinite() ||
          piece_position_jacobians[piece_id].norm() <= 1.0e-12)
      {
        reason = "piece_has_no_spatial_deformation_jacobian";
        compliances.clear();
        return false;
      }
      compliances[piece_id] =
          piece_position_jacobians[piece_id] * inverse_hessian *
          piece_position_jacobians[piece_id].transpose();
      compliances[piece_id] =
          0.5 * (compliances[piece_id] +
                 compliances[piece_id].transpose());
      if (!compliances[piece_id].allFinite())
      {
        reason = "nonfinite_piece_compliance";
        compliances.clear();
        return false;
      }
    }
    reason = "accepted";
    return true;
  }

  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    fill(opt->min_ellip_dist2_.begin(), opt->min_ellip_dist2_.end(), std::numeric_limits<double>::max());

    Eigen::MatrixXd P;
    const Eigen::Map<const Eigen::VectorXd> spatial_variables(
        x, opt->spatial_variable_num_);
    if (opt->hard_parameterization_active_)
    {
      if (!opt->hard_corridor_parameterization_.decode(spatial_variables, P))
      {
        Eigen::Map<Eigen::VectorXd>(grad, n).setZero();
        return 1.0e100;
      }
    }
    else
    {
      P = Eigen::Map<const Eigen::MatrixXd>(x, 3, opt->piece_num_ - 1);
    }
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + opt->spatial_variable_num_,
                                       opt->piece_num_);
    Eigen::MatrixXd gradP(3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> grad_spatial(grad,
                                             opt->spatial_variable_num_);
    Eigen::Map<Eigen::VectorXd> gradt(grad + opt->spatial_variable_num_,
                                      opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(5);

    opt->VirtualT2RealT(t, T); // Unbounded virtual time to real time

    opt->jerkOpt_.generate(P, T); // Generate trajectory from {P,T}

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAJGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Original EGO time-integral costs
    if (opt->tf_sfc_manager_ && !opt->tf_corridors_.empty())
    {
      // The optimizer and the strict final gate must see the same frozen
      // corridor sample grid. Keep this separate from EGO's original control
      // point count so the no-corridor baseline remains bit-for-bit unchanged.
      opt->addCorridorGradCost2CT(
          gradT, obs_swarm_feas_qvar_costs,
          std::max(opt->tf_sfc_parameters_.samples_per_piece, 2));
    }

    if (opt->allowRebound())
    {
      opt->roughlyCheckConstraintPoints(); // Trajectory rebound
    }

    opt->jerkOpt_.getGrad2TP(gradT, gradP); // Gradient prepagation

    double hard_parameterization_cost = 0.0;
    if (opt->hard_parameterization_active_)
    {
      if (!opt->hard_corridor_parameterization_.backwardGradient(
              spatial_variables, gradP, grad_spatial))
      {
        Eigen::Map<Eigen::VectorXd>(grad, n).setZero();
        return 1.0e100;
      }
      hard_parameterization_cost =
          opt->hard_corridor_parameterization_.addNormRestriction(
              spatial_variables, grad_spatial);
    }
    else
    {
      grad_spatial = Eigen::Map<const Eigen::VectorXd>(
          gradP.data(), gradP.size());
    }

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost); // Real time back to virtual time

    opt->iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost +
           hard_parameterization_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addCorridorGradCost2CT(
      EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    if (!tf_sfc_manager_ || tf_corridors_.empty() || K < 2)
    {
      return;
    }

    const int piece_count = gdT.size();
    Eigen::Vector3d pos, vel, gradp;
    Eigen::Matrix<double, 6, 1> beta0, beta1;
    for (int piece_id = 0; piece_id < piece_count; ++piece_id)
    {
      const Eigen::Matrix<double, 6, 3> &coeff =
          jerkOpt_.get_b().block<6, 3>(piece_id * 6, 0);
      const double step = jerkOpt_.get_T1()(piece_id) /
                          static_cast<double>(K);
      for (int sample_id = 0; sample_id <= K; ++sample_id)
      {
        const double time = step * static_cast<double>(sample_id);
        const double time2 = time * time;
        const double time3 = time2 * time;
        const double time4 = time2 * time2;
        const double time5 = time4 * time;
        beta0 << 1.0, time, time2, time3, time4, time5;
        beta1 << 0.0, 1.0, 2.0 * time, 3.0 * time2,
                 4.0 * time3, 5.0 * time4;
        pos = coeff.transpose() * beta0;
        vel = coeff.transpose() * beta1;

        double sample_cost = 0.0;
        if (!tf_sfc_manager_->corridorGradCost(
                piece_id, pos, gradp, sample_cost))
        {
          continue;
        }

        const double alpha =
            static_cast<double>(sample_id) / static_cast<double>(K);
        const double quadrature_weight =
            (sample_id == 0 || sample_id == K) ? 0.5 : 1.0;
        jerkOpt_.get_gdC().block<6, 3>(piece_id * 6, 0) +=
            quadrature_weight * step * beta0 * gradp.transpose();
        gdT(piece_id) += quadrature_weight *
                         (sample_cost / static_cast<double>(K) +
                          step * alpha * gradp.dot(vel));
        costs(4) += quadrature_weight * step * sample_cost;
      }
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    //
    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d gradp, gradv, grada, gradj;
    double costp, costv, costa, costj;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    // Eigen::MatrixXd constraint_pts(3, N * K + 1);

    // printf("A\n");

    // int innerLoop;
    double t = 0;
    for (int i = 0; i < N; ++i)
    {

      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }

        // swarm
        double gradt, grad_prev_t;
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t;
          }
          costs(1) += omg * step * costp;
        }

        // feasibility
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(2) += omg * step * costv;
        }

        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(2) += omg * step * costa;
        }

        if (feasibilityGradCostJ(jer, gradj, costj))
        {
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * sna;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaJc;
          gdT(i) += omg * (costj / K + step * gradViolaJt);
          costs(2) += omg * step * costj;
        }

        // printf("L\n");

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }

      t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    // lengthVarianceWithGradCost2p(cps_.points, K, gdp, var);
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(3) += var;
  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // Obatacle cost
    for (size_t j = 0; j < cps_.direction[i_dp].size(); ++j)
    {
      Eigen::Vector3d ray = (p - cps_.base_point[i_dp][j]);
      double dist = ray.dot(cps_.direction[i_dp][j]);
      double dist_err = obs_clearance_ - dist;
      double dist_err_soft = obs_clearance_soft_ - dist;
      Eigen::Vector3d dist_grad = cps_.direction[i_dp][j];

      if (dist_err > 0)
      {
        ret = true;
        costp += wei_obs_ * pow(dist_err, 3);
        gradp += -wei_obs_ * 3.0 * dist_err * dist_err * dist_grad;
      }

      if (dist_err_soft > 0)
      {
        ret = true;
        double r = 0.05;
        double rsqr = r * r;
        double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
        costp += wei_obs_soft_ * rsqr * (term - 1.0);
        gradp += -wei_obs_soft_ * dist_err_soft / term * dist_grad;
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    if (i_dp <= 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    for (size_t id = 0; id < swarm_trajs_->size(); id++)
    {
      if ((swarm_trajs_->at(id).drone_id < 0) || swarm_trajs_->at(id).drone_id == drone_id_)
      {
        continue;
      }

      double traj_i_satrt_time = swarm_trajs_->at(id).start_time;
      double pt_time = (t_now_ - traj_i_satrt_time) + t; // never assign a high-precision golbal time to a double directly!
      const double CLEARANCE = (swarm_clearance_ + swarm_trajs_->at(id).des_clearance) * 1.5; // 1.5 is to compensate slight constraint violation
      const double CLEARANCE2 = CLEARANCE * CLEARANCE;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < swarm_trajs_->at(id).duration)
      {
        swarm_p = swarm_trajs_->at(id).traj.getPos(pt_time);
        swarm_v = swarm_trajs_->at(id).traj.getVel(pt_time);
      }
      else
      {
        double exceed_time = pt_time - swarm_trajs_->at(id).duration;
        swarm_v = swarm_trajs_->at(id).traj.getVel(swarm_trajs_->at(id).duration);
        swarm_p = swarm_trajs_->at(id).traj.getPos(swarm_trajs_->at(id).duration) +
                  exceed_time * swarm_v;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;

        costp += wei_swarm_mod_ * dist2_err3;

        Eigen::Vector3d dJ_dP = wei_swarm_mod_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }

      if (min_ellip_dist2_[id] > ellip_dist2)
      {
        min_ellip_dist2_[id] = ellip_dist2;
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostJ(const Eigen::Vector3d &j,
                                               Eigen::Vector3d &gradj,
                                               double &costj)
  {
    double jpen = j.squaredNorm() - max_jer_ * max_jer_;
    if (jpen > 0)
    {
      gradj = wei_feas_ * 6 * jpen * jpen * j;
      costj = wei_feas_ * jpen * jpen * jpen;
      return true;
    }
    return false;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    // double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    // double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1)) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i)) / N * dps.col(i));
      }
    }
    return;
  }

  void PolyTrajOptimizer::lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                       const int n,
                                                       Eigen::MatrixXd &gdp,
                                                       double &var)
  {
    int N = ps.cols() - 1;
    int M = N / n;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd ds = dps.colwise().norm().transpose();
    Eigen::VectorXd ls(M), lsqrs(M);
    for (int i = 0; i < M; i++)
    {
      ls(i) = ds.segment(i * n, n).sum();
      lsqrs(i) = ls(i) * ls(i);
    }
    double lm = ls.mean();
    double lsqrm = lsqrs.mean();
    var = wei_sqrvar_ * (lsqrm - lm * lm) + 250.0 * M * lm;
    Eigen::VectorXd gdls = wei_sqrvar_ * 2.0 / M * (ls.array() - lm) + 250.0;
    Eigen::MatrixXd gdds = dps.colwise().normalized();
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i < M; i++)
    {
      gdp.block(0, i * n, 3, n) -= gdls(i) * gdds.block(0, i * n, 3, n);
      gdp.block(0, i * n + 1, 3, n) += gdls(i) * gdds.block(0, i * n, 3, n);
    }
    return;
  }

  /* helper functions */
  void PolyTrajOptimizer::publishTfSfcCorridors(
      const tf_sfc::CorridorVector &corridors)
  {
#ifdef TF_SFC_WITH_DECOMP_ROS
    if (!tf_sfc_parameters_.visualization_enabled ||
        tf_sfc_polyhedron_pub_.getTopic().empty())
    {
      return;
    }

    decomp_ros_msgs::PolyhedronArray message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = tf_sfc_parameters_.visualization_frame;
    for (const tf_sfc::Corridor &corridor : corridors)
    {
      if (!corridor.metrics.valid || corridor.hpoly.rows() == 0)
      {
        continue;
      }

      decomp_ros_msgs::Polyhedron polyhedron;
      for (int face_id = 0; face_id < corridor.hpoly.rows(); ++face_id)
      {
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        if (!tf_sfc::hpolyFaceToPointNormal(
                corridor.hpoly, face_id, point, normal))
        {
          continue;
        }
        geometry_msgs::Point point_message;
        point_message.x = point.x();
        point_message.y = point.y();
        point_message.z = point.z();
        geometry_msgs::Point normal_message;
        normal_message.x = normal.x();
        normal_message.y = normal.y();
        normal_message.z = normal.z();
        polyhedron.points.push_back(point_message);
        polyhedron.normals.push_back(normal_message);
      }
      if (!polyhedron.points.empty())
      {
        message.polyhedrons.push_back(polyhedron);
      }
    }
    tf_sfc_polyhedron_pub_.publish(message);
#else
    (void)corridors;
#endif
  }

  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constraint_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);
    nh.param("optimization/weight_swarm", wei_swarm_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    nh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);
    nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_jer", max_jer_, -1.0);

    nh.param("tf_sfc/enabled", tf_sfc_parameters_.enabled, false);
    nh.param("tf_sfc/use_projection", tf_sfc_parameters_.use_projection, true);
    nh.param("tf_sfc/use_soft_penalty", tf_sfc_parameters_.use_soft_penalty, false);
    nh.param("tf_sfc/allow_partial_corridors", tf_sfc_parameters_.allow_partial_corridors, true);
    nh.param("tf_sfc/allow_ego_fallback", tf_sfc_parameters_.allow_ego_fallback, true);
    nh.param("tf_sfc/allow_direction_fallback",
             tf_sfc_parameters_.allow_direction_fallback, true);
    nh.param("tf_sfc/enforce_final_corridor",
             tf_sfc_parameters_.enforce_final_corridor, true);
    nh.param("tf_sfc/hard_corridor_parameterization",
             tf_sfc_parameters_.hard_corridor_parameterization, true);
    nh.param("tf_sfc/visualization_enabled", tf_sfc_parameters_.visualization_enabled, true);
    nh.param<std::string>("tf_sfc/visualization_frame",
                          tf_sfc_parameters_.visualization_frame, "world");
    nh.param("tf_sfc/log_enabled", tf_sfc_parameters_.log_enabled, true);
    nh.param<std::string>("tf_sfc/log_directory", tf_sfc_parameters_.log_directory,
                          "/tmp/tf_sfc_results/ego");
    nh.param<std::string>("tf_sfc/experiment_tag", tf_sfc_parameters_.experiment_tag,
                          "default");
    nh.param<std::string>("tf_sfc/corridor_method", tf_sfc_parameters_.corridor_method,
                          "obb");
    nh.param("tf_sfc/max_faces", tf_sfc_parameters_.max_faces, 12);
    nh.param("tf_sfc/max_obs_faces", tf_sfc_parameters_.max_obs_faces, 6);
    nh.param("tf_sfc/samples_per_piece", tf_sfc_parameters_.samples_per_piece, 8);
    nh.param("tf_sfc/projection_passes", tf_sfc_parameters_.projection_passes, 4);
    nh.param("tf_sfc/min_valid_pieces", tf_sfc_parameters_.min_valid_pieces, 1);
    nh.param("tf_sfc/max_enforcement_passes",
             tf_sfc_parameters_.max_enforcement_passes, 2);
    nh.param("tf_sfc/hard_max_vertices",
             tf_sfc_parameters_.hard_max_vertices, 64);
    nh.param("tf_sfc/safety_margin", tf_sfc_parameters_.safety_margin, 0.25);
    nh.param("tf_sfc/interior_sample_margin",
             tf_sfc_parameters_.interior_sample_margin, 0.0);
    nh.param("tf_sfc/min_overlap_radius", tf_sfc_parameters_.min_overlap_radius, 0.02);
    nh.param("tf_sfc/junction_refine_radius",
             tf_sfc_parameters_.junction_refine_radius, 0.50);
    nh.param("tf_sfc/junction_refine_step",
             tf_sfc_parameters_.junction_refine_step, 0.05);
    nh.param("tf_sfc/max_inflation_distance", tf_sfc_parameters_.max_inflation_distance, 1.0);
    nh.param("tf_sfc/inflation_step", tf_sfc_parameters_.inflation_step, 0.10);
    nh.param("tf_sfc/weight", tf_sfc_parameters_.weight, 1000.0);
    nh.param("tf_sfc/enforcement_weight_multiplier",
             tf_sfc_parameters_.enforcement_weight_multiplier, 3.0);
    nh.param("tf_sfc/enforcement_min_improvement",
             tf_sfc_parameters_.enforcement_min_improvement, 1.0e-5);
    nh.param("tf_sfc/max_final_violation",
             tf_sfc_parameters_.max_final_violation, 1.0e-3);
    nh.param("tf_sfc/hard_vertex_tolerance",
             tf_sfc_parameters_.hard_vertex_tolerance, 1.0e-7);
    nh.param("tf_sfc/penalty_epsilon", tf_sfc_parameters_.penalty_epsilon, 0.02);
    nh.param("tf_sfc/decomp_local_bbox_forward",
             tf_sfc_parameters_.decomp_local_bbox_forward, 0.5);
    nh.param("tf_sfc/decomp_local_bbox_lateral",
             tf_sfc_parameters_.decomp_local_bbox_lateral, 1.0);
    nh.param("tf_sfc/decomp_local_bbox_vertical",
             tf_sfc_parameters_.decomp_local_bbox_vertical, 1.0);
    nh.param("tf_sfc/decomp_overlap_extension",
             tf_sfc_parameters_.decomp_overlap_extension, 0.20);
    nh.param("tf_sfc/decomp_initial_velocity_segment",
             tf_sfc_parameters_.decomp_initial_velocity_segment, 0.40);
    nh.param("tf_sfc/decomp_initial_velocity_threshold",
             tf_sfc_parameters_.decomp_initial_velocity_threshold, 0.20);
    nh.param("tf_sfc/decomp_degenerate_seed_length",
             tf_sfc_parameters_.decomp_degenerate_seed_length, 0.10);
    nh.param("tf_sfc/decomp_retry_seed_validation_without_velocity",
             tf_sfc_parameters_.decomp_retry_seed_validation_without_velocity,
             true);
    nh.param("tf_sfc/seed_retry_without_velocity_on_clearance_failure",
             tf_sfc_parameters_
                 .seed_retry_without_velocity_on_clearance_failure,
             true);
    nh.param("tf_sfc/seed_clearance_astar_enabled",
             tf_sfc_parameters_.seed_clearance_astar_enabled, true);
    nh.param("tf_sfc/seed_clearance_astar_time_limit",
             tf_sfc_parameters_.seed_clearance_astar_time_limit, 0.20);
    nh.param("tf_sfc/trajectory_repair_enabled",
             tf_sfc_parameters_.trajectory_repair_enabled, true);
    nh.param("tf_sfc/trajectory_repair_seed_fallback_enabled",
             tf_sfc_parameters_.trajectory_repair_seed_fallback_enabled,
             true);
    nh.param(
        "tf_sfc/trajectory_repair_seed_geometric_acceptance_enabled",
        tf_sfc_parameters_
            .trajectory_repair_seed_geometric_acceptance_enabled,
        false);
    nh.param("tf_sfc/trajectory_repair_max_passes",
             tf_sfc_parameters_.trajectory_repair_max_passes, 1);
    nh.param("tf_sfc/post_optimization_repair_enabled",
             tf_sfc_parameters_.post_optimization_repair_enabled, false);
    nh.param("tf_sfc/progress_guard_enabled",
             tf_sfc_parameters_.progress_guard_enabled, true);
    nh.param("tf_sfc/post_optimization_repair_max_passes",
             tf_sfc_parameters_.post_optimization_repair_max_passes, 1);
    nh.param(
        "tf_sfc/post_optimization_repair_max_seed_junction_shift",
        tf_sfc_parameters_
            .post_optimization_repair_max_seed_junction_shift,
        1.0e-5);
    nh.param("tf_sfc/trajectory_repair_trigger",
             tf_sfc_parameters_.trajectory_repair_trigger, 1.0e-3);
    nh.param("tf_sfc/trajectory_repair_min_improvement",
             tf_sfc_parameters_.trajectory_repair_min_improvement, 1.0e-4);
    nh.param("tf_sfc/face_quality_weight",
             tf_sfc_parameters_.face_quality_weight, 0.20);
    nh.param("tf_sfc/exact_face_subset_pruning_enabled",
             tf_sfc_parameters_.exact_face_subset_pruning_enabled, true);
    nh.param("tf_sfc/progress_guard_horizon_s",
             tf_sfc_parameters_.progress_guard_horizon_s, 1.0);
    nh.param("tf_sfc/max_initial_progress_regression",
             tf_sfc_parameters_.max_initial_progress_regression, 0.05);
    nh.param("tf_sfc/max_target_axis_progress_drop",
             tf_sfc_parameters_.max_target_axis_progress_drop, 0.25);
    nh.param("tf_sfc/max_target_overshoot",
             tf_sfc_parameters_.max_target_overshoot, 0.15);
    nh.param("tf_sfc/objective_compliance_fd_step",
             tf_sfc_parameters_.objective_compliance_fd_step, 0.02);
    nh.param("tf_sfc/objective_compliance_eigenvalue_floor_ratio",
             tf_sfc_parameters_
                 .objective_compliance_eigenvalue_floor_ratio,
             1.0e-4);
    nh.param("tf_sfc/objective_compliance_absolute_floor",
             tf_sfc_parameters_.objective_compliance_absolute_floor,
             1.0e-6);
    nh.param("tf_sfc/objective_compliance_transport_speed_reference",
             tf_sfc_parameters_
                 .objective_compliance_transport_speed_reference,
             1.0);
    nh.param("tf_sfc/objective_compliance_transport_weight_max",
             tf_sfc_parameters_.objective_compliance_transport_weight_max,
             2.0);
    if (tf_sfc_parameters_.seed_clearance_astar_time_limit <= 0.0)
    {
      ROS_WARN("tf_sfc/seed_clearance_astar_time_limit must be positive; "
               "using 0.20 s.");
      tf_sfc_parameters_.seed_clearance_astar_time_limit = 0.20;
    }
    if (tf_sfc_parameters_.trajectory_repair_max_passes < 0)
    {
      ROS_WARN("tf_sfc/trajectory_repair_max_passes must be non-negative; "
               "using 0.");
      tf_sfc_parameters_.trajectory_repair_max_passes = 0;
    }
    if (tf_sfc_parameters_.post_optimization_repair_max_passes < 0)
    {
      ROS_WARN("tf_sfc/post_optimization_repair_max_passes must be "
               "non-negative; using 0.");
      tf_sfc_parameters_.post_optimization_repair_max_passes = 0;
    }
    if (tf_sfc_parameters_
            .post_optimization_repair_max_seed_junction_shift < 0.0)
    {
      ROS_WARN("tf_sfc/post_optimization_repair_max_seed_junction_shift "
               "must be non-negative; using 1e-5 m.");
      tf_sfc_parameters_
          .post_optimization_repair_max_seed_junction_shift = 1.0e-5;
    }
    if (tf_sfc_parameters_.trajectory_repair_trigger < 0.0)
    {
      ROS_WARN("tf_sfc/trajectory_repair_trigger must be non-negative; "
               "using 0.001 m.");
      tf_sfc_parameters_.trajectory_repair_trigger = 1.0e-3;
    }
    if (tf_sfc_parameters_.trajectory_repair_min_improvement < 0.0)
    {
      ROS_WARN("tf_sfc/trajectory_repair_min_improvement must be "
               "non-negative; using 0.0001 m.");
      tf_sfc_parameters_.trajectory_repair_min_improvement = 1.0e-4;
    }

    int direction_mode = static_cast<int>(tf_sfc::DirectionMode::PCA);
    nh.param("tf_sfc/direction_mode", direction_mode,
             static_cast<int>(tf_sfc::DirectionMode::PCA));
    if (direction_mode < static_cast<int>(tf_sfc::DirectionMode::FRENET) ||
        direction_mode >
            static_cast<int>(
                tf_sfc::DirectionMode::FULL_OBJECTIVE_COMPLIANCE))
    {
      ROS_WARN("Invalid tf_sfc/direction_mode=%d; using PCA (1).", direction_mode);
      direction_mode = static_cast<int>(tf_sfc::DirectionMode::PCA);
    }
    tf_sfc_parameters_.direction_mode = static_cast<tf_sfc::DirectionMode>(direction_mode);

    if (tf_sfc_parameters_.max_faces < 6)
    {
      ROS_WARN("tf_sfc/max_faces must be at least 6; disabling TF-SFC.");
      tf_sfc_parameters_.enabled = false;
    }
    tf_sfc_parameters_.max_obs_faces =
        std::max(0, std::min(tf_sfc_parameters_.max_obs_faces,
                             tf_sfc_parameters_.max_faces - 6));
    tf_sfc_parameters_.samples_per_piece = std::max(tf_sfc_parameters_.samples_per_piece, 2);
    tf_sfc_parameters_.projection_passes = std::max(tf_sfc_parameters_.projection_passes, 1);
    tf_sfc_parameters_.min_valid_pieces = std::max(tf_sfc_parameters_.min_valid_pieces, 1);
    tf_sfc_parameters_.max_enforcement_passes =
        std::max(tf_sfc_parameters_.max_enforcement_passes, 0);
    tf_sfc_parameters_.hard_max_vertices =
        std::max(tf_sfc_parameters_.hard_max_vertices, 4);
    tf_sfc_parameters_.safety_margin = std::max(tf_sfc_parameters_.safety_margin, 0.0);
    tf_sfc_parameters_.interior_sample_margin =
        std::max(tf_sfc_parameters_.interior_sample_margin, 0.0);
    tf_sfc_parameters_.min_overlap_radius =
        std::max(tf_sfc_parameters_.min_overlap_radius, 0.0);
    tf_sfc_parameters_.junction_refine_radius =
        std::max(tf_sfc_parameters_.junction_refine_radius, 0.0);
    tf_sfc_parameters_.junction_refine_step =
        std::max(tf_sfc_parameters_.junction_refine_step, 1.0e-3);
    tf_sfc_parameters_.max_inflation_distance = std::max(tf_sfc_parameters_.max_inflation_distance, 0.0);
    tf_sfc_parameters_.inflation_step = std::max(tf_sfc_parameters_.inflation_step, 1.0e-3);
    tf_sfc_parameters_.face_quality_weight =
        std::max(tf_sfc_parameters_.face_quality_weight, 0.0);
    tf_sfc_parameters_.progress_guard_horizon_s =
        std::max(tf_sfc_parameters_.progress_guard_horizon_s, 0.0);
    tf_sfc_parameters_.max_initial_progress_regression =
        std::max(tf_sfc_parameters_.max_initial_progress_regression, 0.0);
    tf_sfc_parameters_.max_target_axis_progress_drop =
        std::max(tf_sfc_parameters_.max_target_axis_progress_drop, 0.0);
    tf_sfc_parameters_.max_target_overshoot =
        std::max(tf_sfc_parameters_.max_target_overshoot, 0.0);
    tf_sfc_parameters_.objective_compliance_fd_step =
        std::max(tf_sfc_parameters_.objective_compliance_fd_step, 1.0e-5);
    tf_sfc_parameters_.objective_compliance_eigenvalue_floor_ratio =
        std::max(
            tf_sfc_parameters_
                .objective_compliance_eigenvalue_floor_ratio,
            1.0e-12);
    tf_sfc_parameters_.objective_compliance_absolute_floor =
        std::max(tf_sfc_parameters_.objective_compliance_absolute_floor,
                 1.0e-12);
    tf_sfc_parameters_.objective_compliance_transport_speed_reference =
        std::max(
            tf_sfc_parameters_
                .objective_compliance_transport_speed_reference,
            1.0e-3);
    tf_sfc_parameters_.objective_compliance_transport_weight_max =
        std::max(
            tf_sfc_parameters_.objective_compliance_transport_weight_max,
            0.0);
    tf_sfc_parameters_.weight = std::max(tf_sfc_parameters_.weight, 0.0);
    tf_sfc_parameters_.enforcement_weight_multiplier =
        std::max(tf_sfc_parameters_.enforcement_weight_multiplier, 1.0);
    tf_sfc_parameters_.enforcement_min_improvement =
        std::max(tf_sfc_parameters_.enforcement_min_improvement, 0.0);
    tf_sfc_parameters_.max_final_violation =
        std::max(tf_sfc_parameters_.max_final_violation, 0.0);
    tf_sfc_parameters_.hard_vertex_tolerance =
        std::max(tf_sfc_parameters_.hard_vertex_tolerance, 1.0e-10);
    tf_sfc_parameters_.penalty_epsilon = std::max(tf_sfc_parameters_.penalty_epsilon, 0.0);
    tf_sfc_parameters_.decomp_local_bbox_forward =
        std::max(tf_sfc_parameters_.decomp_local_bbox_forward, 1.0e-3);
    tf_sfc_parameters_.decomp_local_bbox_lateral =
        std::max(tf_sfc_parameters_.decomp_local_bbox_lateral, 1.0e-3);
    tf_sfc_parameters_.decomp_local_bbox_vertical =
        std::max(tf_sfc_parameters_.decomp_local_bbox_vertical, 1.0e-3);
    tf_sfc_parameters_.decomp_overlap_extension =
        std::max(tf_sfc_parameters_.decomp_overlap_extension, 0.0);
    tf_sfc_parameters_.decomp_initial_velocity_segment =
        std::max(tf_sfc_parameters_.decomp_initial_velocity_segment, 0.0);
    tf_sfc_parameters_.decomp_initial_velocity_threshold =
        std::max(tf_sfc_parameters_.decomp_initial_velocity_threshold, 0.0);
    tf_sfc_parameters_.decomp_degenerate_seed_length =
        std::max(tf_sfc_parameters_.decomp_degenerate_seed_length, 1.0e-3);
    if (tf_sfc_parameters_.enforce_final_corridor &&
        !tf_sfc_parameters_.use_soft_penalty)
    {
      ROS_WARN("TF-SFC final corridor enforcement is enabled without the soft "
               "corridor penalty; violations will be rejected without weight "
               "continuation.");
    }
    if (tf_sfc_parameters_.corridor_method != "obb" &&
        tf_sfc_parameters_.corridor_method != "tf_sfc" &&
        tf_sfc_parameters_.corridor_method != "ellipsoid_decomp")
    {
      ROS_ERROR("Invalid tf_sfc/corridor_method='%s'; corridor generation will be rejected.",
                tf_sfc_parameters_.corridor_method.c_str());
    }
    tf_sfc_experiment_logger_.reset(new tf_sfc::ExperimentLogger(
        tf_sfc_parameters_.log_enabled,
        tf_sfc_parameters_.log_directory,
        tf_sfc_parameters_.experiment_tag));
#ifdef TF_SFC_WITH_DECOMP_ROS
    if (tf_sfc_parameters_.visualization_enabled)
    {
      tf_sfc_polyhedron_pub_ =
          nh.advertise<decomp_ros_msgs::PolyhedronArray>(
              "tf_sfc/polyhedron_array", 1, true);
    }
#else
    if (tf_sfc_parameters_.visualization_enabled)
    {
      ROS_WARN("TF-SFC visualization requested, but decomp_ros_msgs was not found at build time.");
    }
#endif
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    if (tf_sfc_parameters_.enabled)
    {
      tf_sfc_manager_.reset(new tf_sfc::TfSfcManager(grid_map_, tf_sfc_parameters_));
    }
    else
    {
      tf_sfc_manager_.reset();
    }
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void PolyTrajOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  void PolyTrajOptimizer::setIfTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }

  void PolyTrajOptimizer::setExperimentContext(
      const std::uint64_t goal_id,
      const std::uint64_t replan_id,
      const int retry_index,
      const int attempt_id,
      const Eigen::Vector3d &commanded_goal,
      const bool commanded_goal_valid)
  {
    experiment_goal_id_ = goal_id;
    experiment_replan_id_ = replan_id;
    experiment_retry_index_ = retry_index;
    experiment_attempt_id_ = attempt_id;
    experiment_commanded_goal_ = commanded_goal;
    experiment_commanded_goal_valid_ =
        commanded_goal_valid && commanded_goal.allFinite();
  }

  void PolyTrajOptimizer::setConstraintPoints(ConstraintPoints cps) { cps_ = cps; }

  void PolyTrajOptimizer::setUseMultitopologyTrajs(bool use_multitopology_trajs) { multitopology_data_.use_multitopology_trajs = use_multitopology_trajs; }

} // namespace ego_planner
