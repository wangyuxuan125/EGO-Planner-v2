#include "optimizer/tf_sfc/directional_inflator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <vector>

namespace ego_planner
{
namespace tf_sfc
{

DirectionalInflator::DirectionalInflator(const Parameters &parameters)
    : parameters_(parameters)
{
}

bool DirectionalInflator::inflate(const PointVector &samples,
                                  const DirectionSet &directions,
                                  const double map_resolution,
                                  const OccupancyQuery &is_occupied,
                                  Corridor &corridor,
                                  FailureReason &failure_reason) const
{
  failure_reason = FailureReason::NONE;
  if (samples.empty() || map_resolution <= 0.0 || !is_occupied ||
      parameters_.max_faces < 6 || parameters_.max_obs_faces < 0)
  {
    failure_reason = FailureReason::INVALID_INPUT;
    return false;
  }

  Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d &sample : samples)
  {
    anchor += sample;
  }
  anchor /= static_cast<double>(samples.size());

  Eigen::Vector3d lower =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const Eigen::Vector3d &sample : samples)
  {
    const Eigen::Vector3d local =
        directions.frame.transpose() * (sample - anchor);
    lower = lower.cwiseMin(local);
    upper = upper.cwiseMax(local);
  }
  lower.array() -= parameters_.safety_margin;
  upper.array() += parameters_.safety_margin;

  HPoly accepted_hpoly;
  int accepted_obstacle_faces = 0;
  int accepted_obstacle_points = 0;
  bool accepted_budget_saturated = false;
  double accepted_min_obstacle_sample_distance_m =
      std::numeric_limits<double>::quiet_NaN();
  int accepted_separation_failure_sample_id = -1;
  bool accepted_separation_failure_at_endpoint = false;
  FailureReason initial_failure = FailureReason::NONE;
  const SpaceState initial_state = buildFaceBoundedCandidate(
      samples, anchor, directions.frame, lower, upper, map_resolution,
      is_occupied, accepted_hpoly, accepted_obstacle_faces,
      accepted_obstacle_points, accepted_budget_saturated,
      accepted_min_obstacle_sample_distance_m,
      accepted_separation_failure_sample_id,
      accepted_separation_failure_at_endpoint, initial_failure);
  if (initial_state != SpaceState::FREE)
  {
    corridor.metrics.obstacle_face_count = accepted_obstacle_faces;
    corridor.metrics.obstacle_point_count = accepted_obstacle_points;
    corridor.metrics.face_budget_saturated = accepted_budget_saturated;
    corridor.metrics.min_obstacle_sample_distance_m =
        accepted_min_obstacle_sample_distance_m;
    corridor.metrics.separation_failure_sample_id =
        accepted_separation_failure_sample_id;
    corridor.metrics.separation_failure_at_endpoint =
        accepted_separation_failure_at_endpoint;
    failure_reason =
        initial_failure != FailureReason::NONE
            ? initial_failure
            : (initial_state == SpaceState::OUTSIDE_MAP
                   ? FailureReason::OUTSIDE_LOCAL_MAP
                   : FailureReason::INITIAL_OBB_OCCUPIED);
    return false;
  }

  std::array<int, 3> axis_order{{0, 1, 2}};
  std::sort(axis_order.begin(), axis_order.end(),
            [&directions](const int lhs, const int rhs) {
              return directions.utility(lhs) > directions.utility(rhs);
            });

  const double step = std::max(parameters_.inflation_step, map_resolution);
  for (const int axis : axis_order)
  {
    for (int side = 0; side < 2; ++side)
    {
      double expanded = 0.0;
      while (expanded + step <=
             parameters_.max_inflation_distance + 1.0e-9)
      {
        Eigen::Vector3d candidate_lower = lower;
        Eigen::Vector3d candidate_upper = upper;
        if (side == 0)
        {
          candidate_lower(axis) -= step;
        }
        else
        {
          candidate_upper(axis) += step;
        }

        HPoly candidate_hpoly;
        int candidate_obstacle_faces = 0;
        int candidate_obstacle_points = 0;
        bool candidate_budget_saturated = false;
        double candidate_min_obstacle_sample_distance_m =
            std::numeric_limits<double>::quiet_NaN();
        int candidate_separation_failure_sample_id = -1;
        bool candidate_separation_failure_at_endpoint = false;
        FailureReason candidate_failure = FailureReason::NONE;
        const SpaceState candidate_state = buildFaceBoundedCandidate(
            samples, anchor, directions.frame, candidate_lower,
            candidate_upper, map_resolution, is_occupied, candidate_hpoly,
            candidate_obstacle_faces, candidate_obstacle_points,
            candidate_budget_saturated,
            candidate_min_obstacle_sample_distance_m,
            candidate_separation_failure_sample_id,
            candidate_separation_failure_at_endpoint,
            candidate_failure);
        if (candidate_state != SpaceState::FREE)
        {
          break;
        }

        lower = candidate_lower;
        upper = candidate_upper;
        accepted_hpoly = candidate_hpoly;
        accepted_obstacle_faces = candidate_obstacle_faces;
        accepted_obstacle_points = candidate_obstacle_points;
        accepted_budget_saturated = candidate_budget_saturated;
        accepted_min_obstacle_sample_distance_m =
            candidate_min_obstacle_sample_distance_m;
        accepted_separation_failure_sample_id =
            candidate_separation_failure_sample_id;
        accepted_separation_failure_at_endpoint =
            candidate_separation_failure_at_endpoint;
        expanded += step;
      }
    }
  }

  corridor.anchor = anchor;
  corridor.frame = directions.frame;
  corridor.utility = directions.utility;
  corridor.hpoly = accepted_hpoly;
  corridor.metrics.face_count = corridor.hpoly.rows();
  corridor.metrics.obstacle_face_count = accepted_obstacle_faces;
  corridor.metrics.obstacle_point_count = accepted_obstacle_points;
  corridor.metrics.face_budget_saturated = accepted_budget_saturated;
  corridor.metrics.min_obstacle_sample_distance_m =
      accepted_min_obstacle_sample_distance_m;
  corridor.metrics.separation_failure_sample_id =
      accepted_separation_failure_sample_id;
  corridor.metrics.separation_failure_at_endpoint =
      accepted_separation_failure_at_endpoint;
  corridor.metrics.direction_fallback = directions.used_fallback;
  corridor.metrics.valid = true;

  double min_slack = std::numeric_limits<double>::infinity();
  for (const Eigen::Vector3d &sample : samples)
  {
    min_slack = std::min(min_slack, pointSlack(corridor.hpoly, sample));
  }
  corridor.metrics.min_sample_slack = min_slack;
  corridor.metrics.anchor_clearance_radius =
      pointSlack(corridor.hpoly, anchor);

  const Eigen::Vector3d widths = upper - lower;
  corridor.metrics.weighted_width =
      directions.utility.dot(widths.array().square().matrix());
  return true;
}

bool DirectionalInflator::contains(const HPoly &hpoly,
                                   const Eigen::Vector3d &point,
                                   const double tolerance)
{
  if (hpoly.rows() == 0)
  {
    return false;
  }
  return (hpoly.leftCols<3>() * point - hpoly.col(3)).maxCoeff() <=
         tolerance;
}

double DirectionalInflator::pointSlack(const HPoly &hpoly,
                                       const Eigen::Vector3d &point)
{
  if (hpoly.rows() == 0)
  {
    return -std::numeric_limits<double>::infinity();
  }

  double min_slack = std::numeric_limits<double>::infinity();
  for (int row = 0; row < hpoly.rows(); ++row)
  {
    const double normal_norm = hpoly.row(row).head<3>().norm();
    if (normal_norm <= 1.0e-12)
    {
      return -std::numeric_limits<double>::infinity();
    }
    min_slack =
        std::min(min_slack,
                 (hpoly(row, 3) -
                  hpoly.row(row).head<3>().dot(point)) /
                     normal_norm);
  }
  return min_slack;
}

DirectionalInflator::SpaceState DirectionalInflator::candidateState(
    const Eigen::Vector3d &anchor,
    const Eigen::Matrix3d &frame,
    const Eigen::Vector3d &lower,
    const Eigen::Vector3d &upper,
    const double map_resolution,
    const OccupancyQuery &is_occupied) const
{
  PointVector obstacles;
  return collectCandidateObstacles(anchor, frame, lower, upper,
                                   map_resolution, is_occupied, obstacles);
}

DirectionalInflator::SpaceState
DirectionalInflator::collectCandidateObstacles(
    const Eigen::Vector3d &anchor,
    const Eigen::Matrix3d &frame,
    const Eigen::Vector3d &lower,
    const Eigen::Vector3d &upper,
    const double map_resolution,
    const OccupancyQuery &is_occupied,
    PointVector &obstacles) const
{
  obstacles.clear();
  Eigen::Vector3d aabb_min =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d aabb_max =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (int mask = 0; mask < 8; ++mask)
  {
    Eigen::Vector3d local;
    for (int axis = 0; axis < 3; ++axis)
    {
      local(axis) = (mask & (1 << axis)) ? upper(axis) : lower(axis);
    }
    const Eigen::Vector3d world = anchor + frame * local;
    aabb_min = aabb_min.cwiseMin(world);
    aabb_max = aabb_max.cwiseMax(world);
  }

  // Enlarge the scan AABB so occupied voxels touching the oriented box are
  // included as candidates. This padding is only for candidate collection.
  const double voxel_padding =
      0.5 * std::sqrt(3.0) * map_resolution;
  aabb_min.array() -= voxel_padding;
  aabb_max.array() += voxel_padding;
  const Eigen::Vector3i begin =
      (aabb_min / map_resolution).array().floor().cast<int>();
  const Eigen::Vector3i end =
      (aabb_max / map_resolution).array().ceil().cast<int>();
  constexpr double kInsideTolerance = 1.0e-9;
  bool touched_outside = false;
  for (int x = begin.x(); x <= end.x(); ++x)
  {
    for (int y = begin.y(); y <= end.y(); ++y)
    {
      for (int z = begin.z(); z <= end.z(); ++z)
      {
        const Eigen::Vector3d world =
            (Eigen::Vector3d(x, y, z).array() + 0.5).matrix() *
            map_resolution;
        const Eigen::Vector3d local =
            frame.transpose() * (world - anchor);
        if ((local.array() >=
             lower.array() - voxel_padding - kInsideTolerance)
                .all() &&
            (local.array() <=
             upper.array() + voxel_padding + kInsideTolerance)
                .all())
        {
          const SpaceState state = is_occupied(world);
          if (state == SpaceState::OCCUPIED)
          {
            obstacles.push_back(world);
          }
          touched_outside =
              touched_outside || state == SpaceState::OUTSIDE_MAP;
        }
      }
    }
  }
  if (touched_outside)
  {
    return SpaceState::OUTSIDE_MAP;
  }
  return obstacles.empty() ? SpaceState::FREE : SpaceState::OCCUPIED;
}

DirectionalInflator::SpaceState
DirectionalInflator::buildFaceBoundedCandidate(
    const PointVector &samples,
    const Eigen::Vector3d &anchor,
    const Eigen::Matrix3d &frame,
    const Eigen::Vector3d &lower,
    const Eigen::Vector3d &upper,
    const double map_resolution,
    const OccupancyQuery &is_occupied,
    HPoly &hpoly,
    int &obstacle_face_count,
    int &obstacle_point_count,
    bool &face_budget_saturated,
    double &min_obstacle_sample_distance_m,
    int &separation_failure_sample_id,
    bool &separation_failure_at_endpoint,
    FailureReason &failure_reason) const
{
  hpoly = boundsToHPoly(anchor, frame, lower, upper);
  obstacle_face_count = 0;
  obstacle_point_count = 0;
  face_budget_saturated = false;
  min_obstacle_sample_distance_m =
      std::numeric_limits<double>::quiet_NaN();
  separation_failure_sample_id = -1;
  separation_failure_at_endpoint = false;
  failure_reason = FailureReason::NONE;

  PointVector obstacles;
  const SpaceState scan_state = collectCandidateObstacles(
      anchor, frame, lower, upper, map_resolution, is_occupied, obstacles);
  if (scan_state == SpaceState::OUTSIDE_MAP)
  {
    failure_reason = FailureReason::OUTSIDE_LOCAL_MAP;
    return SpaceState::OUTSIDE_MAP;
  }
  obstacle_point_count = static_cast<int>(obstacles.size());
  if (obstacles.empty())
  {
    return SpaceState::FREE;
  }

  if (parameters_.corridor_method != "tf_sfc")
  {
    failure_reason = FailureReason::INITIAL_OBB_OCCUPIED;
    return SpaceState::OCCUPIED;
  }

  const int face_capacity =
      std::max(0, std::min(parameters_.max_obs_faces,
                           parameters_.max_faces - 6));
  if (face_capacity == 0)
  {
    face_budget_saturated = true;
    failure_reason = FailureReason::FACE_BUDGET_EXHAUSTED;
    return SpaceState::OCCUPIED;
  }

  std::vector<int> obstacle_order(obstacles.size());
  std::iota(obstacle_order.begin(), obstacle_order.end(), 0);
  std::sort(obstacle_order.begin(), obstacle_order.end(),
            [&obstacles, &samples](const int lhs, const int rhs) {
              double lhs_distance =
                  std::numeric_limits<double>::infinity();
              double rhs_distance =
                  std::numeric_limits<double>::infinity();
              for (const Eigen::Vector3d &sample : samples)
              {
                lhs_distance =
                    std::min(lhs_distance,
                             (obstacles[lhs] - sample).squaredNorm());
                rhs_distance =
                    std::min(rhs_distance,
                             (obstacles[rhs] - sample).squaredNorm());
              }
              return lhs_distance < rhs_distance;
            });

  PointVector cut_normals;
  std::vector<double> cut_offsets;
  const auto sampleMargin = [this, &samples](const size_t sample_id) {
    const bool endpoint =
        sample_id == 0 || sample_id + 1 == samples.size();
    return endpoint ? parameters_.min_overlap_radius
                    : parameters_.interior_sample_margin;
  };
  constexpr double kNormalMergeCosine = 0.985;
  constexpr double kTolerance = 1.0e-9;

  for (const int obstacle_id : obstacle_order)
  {
    const Eigen::Vector3d &obstacle = obstacles[obstacle_id];
    bool already_excluded = false;
    for (size_t face_id = 0; face_id < cut_normals.size(); ++face_id)
    {
      if (cut_normals[face_id].dot(obstacle) >
          cut_offsets[face_id] + kTolerance)
      {
        already_excluded = true;
        break;
      }
    }
    if (already_excluded)
    {
      continue;
    }

    int nearest_sample_id = -1;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t sample_id = 0; sample_id < samples.size(); ++sample_id)
    {
      const double distance =
          (obstacle - samples[sample_id]).squaredNorm();
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest_sample_id = static_cast<int>(sample_id);
      }
    }
    if (nearest_sample_id >= 0)
    {
      const double distance_m = std::sqrt(nearest_distance);
      min_obstacle_sample_distance_m =
          std::isfinite(min_obstacle_sample_distance_m)
              ? std::min(min_obstacle_sample_distance_m, distance_m)
              : distance_m;
    }
    if (nearest_sample_id < 0 || nearest_distance <= 1.0e-12)
    {
      separation_failure_sample_id = nearest_sample_id;
      separation_failure_at_endpoint =
          nearest_sample_id == 0 ||
          nearest_sample_id + 1 == static_cast<int>(samples.size());
      failure_reason = FailureReason::OBSTACLE_SEPARATION_FAILURE;
      return SpaceState::OCCUPIED;
    }

    Eigen::Vector3d normal =
        (obstacle - samples[nearest_sample_id]).normalized();
    // Exact support of an axis-aligned occupied voxel in the plane-normal
    // direction.  The previous half-diagonal sphere was safe but added an
    // unnecessary orientation-independent clearance on top of map inflation.
    const double voxel_padding =
        0.5 * map_resolution * normal.cwiseAbs().sum();
    double offset = normal.dot(obstacle) - voxel_padding;
    double sample_support =
        -std::numeric_limits<double>::infinity();
    int support_sample_id = -1;
    for (size_t sample_id = 0; sample_id < samples.size(); ++sample_id)
    {
      const double support =
          normal.dot(samples[sample_id]) + sampleMargin(sample_id);
      if (support > sample_support)
      {
        sample_support = support;
        support_sample_id = static_cast<int>(sample_id);
      }
    }
    if (sample_support > offset + kTolerance)
    {
      separation_failure_sample_id = support_sample_id;
      separation_failure_at_endpoint =
          support_sample_id == 0 ||
          support_sample_id + 1 == static_cast<int>(samples.size());
      failure_reason = FailureReason::OBSTACLE_SEPARATION_FAILURE;
      return SpaceState::OCCUPIED;
    }

    int merge_id = -1;
    for (size_t face_id = 0; face_id < cut_normals.size(); ++face_id)
    {
      if (cut_normals[face_id].dot(normal) >= kNormalMergeCosine)
      {
        merge_id = static_cast<int>(face_id);
        break;
      }
    }
    if (merge_id >= 0)
    {
      const double merged_offset =
          std::min(cut_offsets[merge_id], offset);
      double merged_support =
          -std::numeric_limits<double>::infinity();
      int merged_support_sample_id = -1;
      for (size_t sample_id = 0; sample_id < samples.size(); ++sample_id)
      {
        const double support =
            cut_normals[merge_id].dot(samples[sample_id]) +
            sampleMargin(sample_id);
        if (support > merged_support)
        {
          merged_support = support;
          merged_support_sample_id = static_cast<int>(sample_id);
        }
      }
      if (merged_support > merged_offset + kTolerance)
      {
        separation_failure_sample_id = merged_support_sample_id;
        separation_failure_at_endpoint =
            merged_support_sample_id == 0 ||
            merged_support_sample_id + 1 ==
                static_cast<int>(samples.size());
        failure_reason = FailureReason::OBSTACLE_SEPARATION_FAILURE;
        return SpaceState::OCCUPIED;
      }
      cut_offsets[merge_id] = merged_offset;
    }
    else
    {
      if (static_cast<int>(cut_normals.size()) >= face_capacity)
      {
        face_budget_saturated = true;
        failure_reason = FailureReason::FACE_BUDGET_EXHAUSTED;
        return SpaceState::OCCUPIED;
      }
      cut_normals.push_back(normal);
      cut_offsets.push_back(offset);
    }
  }

  hpoly.conservativeResize(6 + static_cast<int>(cut_normals.size()), 4);
  for (size_t face_id = 0; face_id < cut_normals.size(); ++face_id)
  {
    hpoly.row(6 + static_cast<int>(face_id)).head<3>() =
        cut_normals[face_id].transpose();
    hpoly(6 + static_cast<int>(face_id), 3) = cut_offsets[face_id];
  }

  for (const Eigen::Vector3d &sample : samples)
  {
    if (!contains(hpoly, sample, kTolerance))
    {
      failure_reason = FailureReason::OBSTACLE_SEPARATION_FAILURE;
      return SpaceState::OCCUPIED;
    }
  }
  for (const Eigen::Vector3d &obstacle : obstacles)
  {
    if (contains(hpoly, obstacle, kTolerance))
    {
      face_budget_saturated =
          static_cast<int>(cut_normals.size()) >= face_capacity;
      failure_reason = face_budget_saturated
                           ? FailureReason::FACE_BUDGET_EXHAUSTED
                           : FailureReason::OBSTACLE_SEPARATION_FAILURE;
      return SpaceState::OCCUPIED;
    }
  }

  obstacle_face_count = static_cast<int>(cut_normals.size());
  face_budget_saturated = obstacle_face_count >= face_capacity;
  return SpaceState::FREE;
}

HPoly DirectionalInflator::boundsToHPoly(
    const Eigen::Vector3d &anchor,
    const Eigen::Matrix3d &frame,
    const Eigen::Vector3d &lower,
    const Eigen::Vector3d &upper)
{
  HPoly hpoly(6, 4);
  for (int axis = 0; axis < 3; ++axis)
  {
    const Eigen::Vector3d positive = frame.col(axis);
    hpoly.row(2 * axis).head<3>() = positive.transpose();
    hpoly(2 * axis, 3) = positive.dot(anchor) + upper(axis);

    const Eigen::Vector3d negative = -positive;
    hpoly.row(2 * axis + 1).head<3>() = negative.transpose();
    hpoly(2 * axis + 1, 3) = negative.dot(anchor) - lower(axis);
  }
  return hpoly;
}

} // namespace tf_sfc
} // namespace ego_planner
