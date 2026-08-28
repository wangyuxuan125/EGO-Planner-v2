#include "optimizer/tf_sfc/directional_inflator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace ego_planner
{
namespace tf_sfc
{

namespace
{
// Exact closest pair between a line segment and an axis-aligned voxel box.
// The squared distance is a convex piecewise quadratic in the segment
// parameter.  Box-face crossing parameters partition [0, 1] into intervals
// with a fixed active set, so each interval needs at most one stationary test.
bool closestSegmentAabb(const Eigen::Vector3d &start,
                        const Eigen::Vector3d &finish,
                        const Eigen::Vector3d &box_center,
                        const Eigen::Vector3d &half_extent,
                        Eigen::Vector3d &segment_point,
                        Eigen::Vector3d &box_point)
{
  if (!start.allFinite() || !finish.allFinite() ||
      !box_center.allFinite() || !half_extent.allFinite())
  {
    return false;
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
      const double bound = box_center(axis) +
                           (side == 0 ? -half_extent(axis)
                                      : half_extent(axis));
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
    const Eigen::Vector3d candidate_segment = start + ratio * delta;
    Eigen::Vector3d candidate_box;
    for (int axis = 0; axis < 3; ++axis)
    {
      candidate_box(axis) = std::max(
          box_center(axis) - half_extent(axis),
          std::min(box_center(axis) + half_extent(axis),
                   candidate_segment(axis)));
    }
    const double distance_squared =
        (candidate_box - candidate_segment).squaredNorm();
    if (distance_squared < best_distance_squared)
    {
      best_distance_squared = distance_squared;
      segment_point = candidate_segment;
      box_point = candidate_box;
    }
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
      const double lower =
          box_center(axis) - half_extent(axis);
      const double upper =
          box_center(axis) + half_extent(axis);
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
  return std::isfinite(best_distance_squared);
}
} // namespace

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
  int candidate_evaluation_count = 1;
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
    corridor.metrics.inflation_candidate_evaluation_count =
        candidate_evaluation_count;
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
  const int max_expansion_steps = static_cast<int>(std::floor(
      (parameters_.max_inflation_distance + 1.0e-9) / step));
  const auto regionQuality = [&](const Eigen::Vector3d &candidate_lower,
                                 const Eigen::Vector3d &candidate_upper,
                                 const int face_count) {
    const Eigen::Vector3d widths = candidate_upper - candidate_lower;
    return directions.utility.dot(widths.array().square().matrix()) -
           parameters_.face_quality_weight *
               static_cast<double>(face_count);
  };
  for (const int axis : axis_order)
  {
    for (int side = 0; side < 2; ++side)
    {
      if (max_expansion_steps <= 0)
      {
        continue;
      }

      const Eigen::Vector3d base_lower = lower;
      const Eigen::Vector3d base_upper = upper;
      int best_steps = 0;
      int largest_feasible_steps = 0;
      Eigen::Vector3d best_lower = lower;
      Eigen::Vector3d best_upper = upper;
      HPoly best_hpoly = accepted_hpoly;
      int best_obstacle_faces = accepted_obstacle_faces;
      int best_obstacle_points = accepted_obstacle_points;
      bool best_budget_saturated = accepted_budget_saturated;
      double best_min_obstacle_sample_distance_m =
          accepted_min_obstacle_sample_distance_m;
      int best_separation_failure_sample_id =
          accepted_separation_failure_sample_id;
      bool best_separation_failure_at_endpoint =
          accepted_separation_failure_at_endpoint;
      double best_quality = regionQuality(
          best_lower, best_upper, accepted_hpoly.rows());
      std::vector<int> evaluated_steps;

      const auto evaluateExpansion = [&](const int expansion_steps) {
        if (expansion_steps <= 0 ||
            std::find(evaluated_steps.begin(), evaluated_steps.end(),
                      expansion_steps) != evaluated_steps.end())
        {
          return expansion_steps <= largest_feasible_steps;
        }
        evaluated_steps.push_back(expansion_steps);
        Eigen::Vector3d candidate_lower = base_lower;
        Eigen::Vector3d candidate_upper = base_upper;
        const double expansion =
            static_cast<double>(expansion_steps) * step;
        if (side == 0)
        {
          candidate_lower(axis) -= expansion;
        }
        else
        {
          candidate_upper(axis) += expansion;
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
        ++candidate_evaluation_count;
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
          return false;
        }

        largest_feasible_steps =
            std::max(largest_feasible_steps, expansion_steps);
        const double candidate_quality = regionQuality(
            candidate_lower, candidate_upper, candidate_hpoly.rows());
        const bool better_quality = candidate_quality > best_quality + 1.0e-9;
        const bool equal_quality_fewer_faces =
            std::abs(candidate_quality - best_quality) <= 1.0e-9 &&
            candidate_hpoly.rows() < best_hpoly.rows();
        if (better_quality || equal_quality_fewer_faces)
        {
          best_quality = candidate_quality;
          best_steps = expansion_steps;
          best_lower = candidate_lower;
          best_upper = candidate_upper;
          best_hpoly = candidate_hpoly;
          best_obstacle_faces = candidate_obstacle_faces;
          best_obstacle_points = candidate_obstacle_points;
          best_budget_saturated = candidate_budget_saturated;
          best_min_obstacle_sample_distance_m =
              candidate_min_obstacle_sample_distance_m;
          best_separation_failure_sample_id =
              candidate_separation_failure_sample_id;
          best_separation_failure_at_endpoint =
              candidate_separation_failure_at_endpoint;
        }
        return true;
      };

      // Search the same discrete inflation grid as the former step-by-step
      // loop.  Candidate feasibility is monotone with expansion: a larger OBB
      // contains every voxel of a smaller one.  Trying the limit first and
      // then bisecting a failing range removes repeated scans and obstacle
      // sorts while retaining the largest feasible discrete expansion.
      if (!evaluateExpansion(max_expansion_steps))
      {
        int feasible_steps = largest_feasible_steps;
        int infeasible_steps = max_expansion_steps;
        while (infeasible_steps - feasible_steps > 1)
        {
          const int trial_steps =
              feasible_steps + (infeasible_steps - feasible_steps) / 2;
          if (evaluateExpansion(trial_steps))
          {
            feasible_steps = trial_steps;
          }
          else
          {
            infeasible_steps = trial_steps;
          }
        }
      }

      // The largest feasible box is not necessarily the best region once the
      // number of retained faces is part of region quality. Probe two bounded
      // smaller alternatives; this adds at most two candidate evaluations per
      // side and avoids an unbounded combinatorial search.
      if (largest_feasible_steps > 1)
      {
        evaluateExpansion(std::max(1, largest_feasible_steps / 2));
      }
      if (largest_feasible_steps > 3)
      {
        evaluateExpansion(std::max(1, largest_feasible_steps / 4));
      }

      if (best_steps > 0)
      {
        lower = best_lower;
        upper = best_upper;
        accepted_hpoly = best_hpoly;
        accepted_obstacle_faces = best_obstacle_faces;
        accepted_obstacle_points = best_obstacle_points;
        accepted_budget_saturated = best_budget_saturated;
        accepted_min_obstacle_sample_distance_m =
            best_min_obstacle_sample_distance_m;
        accepted_separation_failure_sample_id =
            best_separation_failure_sample_id;
        accepted_separation_failure_at_endpoint =
            best_separation_failure_at_endpoint;
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
  corridor.metrics.inflation_candidate_evaluation_count =
      candidate_evaluation_count;
  corridor.metrics.face_budget_saturated = accepted_budget_saturated;
  corridor.metrics.min_obstacle_sample_distance_m =
      accepted_min_obstacle_sample_distance_m;
  corridor.metrics.separation_failure_sample_id =
      accepted_separation_failure_sample_id;
  corridor.metrics.separation_failure_at_endpoint =
      accepted_separation_failure_at_endpoint;
  corridor.metrics.direction_fallback = directions.used_fallback;
  corridor.metrics.direction_metric_source = directions.metric_source;
  corridor.metrics.direction_velocity_alignment_cosine =
      directions.velocity_alignment_cosine;
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
  corridor.metrics.face_quality_penalty =
      parameters_.face_quality_weight *
      static_cast<double>(corridor.metrics.face_count);
  corridor.metrics.region_quality_score =
      corridor.metrics.weighted_width -
      corridor.metrics.face_quality_penalty;
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
    Eigen::Vector3d nearest_seed_point = Eigen::Vector3d::Zero();
    if (samples.size() == 1)
    {
      nearest_sample_id = 0;
      nearest_seed_point = samples.front();
      nearest_distance = (obstacle - nearest_seed_point).squaredNorm();
    }
    else
    {
      // Use the closest point on the continuous seed polyline. A normal from
      // the closest discrete sample can point along the segment and reject
      // another sample even when the complete line is collision-free.
      for (size_t segment_id = 0; segment_id + 1 < samples.size();
           ++segment_id)
      {
        const Eigen::Vector3d segment =
            samples[segment_id + 1] - samples[segment_id];
        const double squared_length = segment.squaredNorm();
        double ratio = 0.0;
        if (squared_length > 1.0e-12)
        {
          ratio = (obstacle - samples[segment_id]).dot(segment) /
                  squared_length;
          ratio = std::max(0.0, std::min(1.0, ratio));
        }
        const Eigen::Vector3d closest =
            samples[segment_id] + ratio * segment;
        const double distance = (obstacle - closest).squaredNorm();
        if (distance < nearest_distance)
        {
          nearest_distance = distance;
          nearest_seed_point = closest;
          nearest_sample_id =
              ratio <= 0.5 ? static_cast<int>(segment_id)
                           : static_cast<int>(segment_id + 1);
        }
      }
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

    // Build a small deterministic separating-axis set.  Centre directions
    // retain the successful v12.4 behaviour for a complete seed segment.
    // Exact segment--voxel-AABB closest-pair directions repair the endpoint
    // cases for which a centre direction is not the maximum-margin support
    // plane.  The certified-gap test below selects between them.
    PointVector normal_candidates;
    const auto append_normal_candidate =
        [&normal_candidates](const Eigen::Vector3d &candidate) {
          if (!candidate.allFinite() || candidate.norm() <= 1.0e-12)
          {
            return;
          }
          const Eigen::Vector3d unit = candidate.normalized();
          for (const Eigen::Vector3d &existing : normal_candidates)
          {
            if (existing.normalized().dot(unit) > 0.9995)
            {
              return;
            }
          }
          normal_candidates.push_back(candidate);
        };

    append_normal_candidate(obstacle - nearest_seed_point);
    for (const Eigen::Vector3d &sample : samples)
    {
      append_normal_candidate(obstacle - sample);
    }

    const Eigen::Vector3d voxel_half_extent =
        Eigen::Vector3d::Constant(0.5 * map_resolution);
    double nearest_voxel_distance_squared =
        std::numeric_limits<double>::infinity();
    if (samples.size() == 1)
    {
      Eigen::Vector3d closest_seed;
      Eigen::Vector3d closest_voxel;
      if (closestSegmentAabb(samples.front(), samples.front(), obstacle,
                             voxel_half_extent, closest_seed,
                             closest_voxel))
      {
        nearest_voxel_distance_squared =
            (closest_voxel - closest_seed).squaredNorm();
        append_normal_candidate(closest_voxel - closest_seed);
      }
    }
    else
    {
      for (size_t segment_id = 0;
           segment_id + 1 < samples.size(); ++segment_id)
      {
        Eigen::Vector3d closest_seed;
        Eigen::Vector3d closest_voxel;
        if (closestSegmentAabb(samples[segment_id],
                               samples[segment_id + 1], obstacle,
                               voxel_half_extent, closest_seed,
                               closest_voxel))
        {
          nearest_voxel_distance_squared = std::min(
              nearest_voxel_distance_squared,
              (closest_voxel - closest_seed).squaredNorm());
          append_normal_candidate(closest_voxel - closest_seed);
        }
      }
    }
    if (std::isfinite(nearest_voxel_distance_squared))
    {
      const double distance_m =
          std::sqrt(std::max(nearest_voxel_distance_squared, 0.0));
      min_obstacle_sample_distance_m =
          std::isfinite(min_obstacle_sample_distance_m)
              ? std::min(min_obstacle_sample_distance_m, distance_m)
              : distance_m;
    }

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double offset = -std::numeric_limits<double>::infinity();
    double best_gap = -std::numeric_limits<double>::infinity();
    int support_sample_id = -1;
    for (const Eigen::Vector3d &candidate : normal_candidates)
    {
      if (!candidate.allFinite() || candidate.norm() <= 1.0e-12)
      {
        continue;
      }
      const Eigen::Vector3d candidate_normal = candidate.normalized();
      const double candidate_voxel_padding =
          0.5 * map_resolution * candidate_normal.cwiseAbs().sum();
      const double candidate_offset =
          candidate_normal.dot(obstacle) - candidate_voxel_padding;
      double candidate_support =
          -std::numeric_limits<double>::infinity();
      int candidate_support_sample_id = -1;
      for (size_t sample_id = 0; sample_id < samples.size(); ++sample_id)
      {
        const double support =
            candidate_normal.dot(samples[sample_id]) +
            sampleMargin(sample_id);
        if (support > candidate_support)
        {
          candidate_support = support;
          candidate_support_sample_id = static_cast<int>(sample_id);
        }
      }
      const double gap = candidate_offset - candidate_support;
      if (gap > best_gap)
      {
        best_gap = gap;
        normal = candidate_normal;
        offset = candidate_offset;
        support_sample_id = candidate_support_sample_id;
      }
    }
    if (!normal.allFinite() || normal.norm() <= 1.0e-12 ||
        best_gap < -kTolerance)
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
