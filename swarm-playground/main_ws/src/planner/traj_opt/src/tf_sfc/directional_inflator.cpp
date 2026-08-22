#include "optimizer/tf_sfc/directional_inflator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

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
                                  Corridor &corridor) const
{
  if (samples.empty() || map_resolution <= 0.0 || !is_occupied || parameters_.max_faces < 6)
  {
    return false;
  }

  Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d &sample : samples)
  {
    anchor += sample;
  }
  anchor /= static_cast<double>(samples.size());

  Eigen::Vector3d lower = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (const Eigen::Vector3d &sample : samples)
  {
    const Eigen::Vector3d local = directions.frame.transpose() * (sample - anchor);
    lower = lower.cwiseMin(local);
    upper = upper.cwiseMax(local);
  }
  lower.array() -= parameters_.safety_margin;
  upper.array() += parameters_.safety_margin;

  if (!candidateIsFree(anchor, directions.frame, lower, upper, map_resolution, is_occupied))
  {
    return false;
  }

  std::array<int, 3> axis_order{{0, 1, 2}};
  std::sort(axis_order.begin(), axis_order.end(), [&directions](const int lhs, const int rhs) {
    return directions.utility(lhs) > directions.utility(rhs);
  });

  const double step = std::max(parameters_.inflation_step, map_resolution);
  for (const int axis : axis_order)
  {
    for (int side = 0; side < 2; ++side)
    {
      double expanded = 0.0;
      while (expanded + step <= parameters_.max_inflation_distance + 1.0e-9)
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

        if (!candidateIsFree(anchor, directions.frame, candidate_lower, candidate_upper,
                             map_resolution, is_occupied))
        {
          break;
        }
        lower = candidate_lower;
        upper = candidate_upper;
        expanded += step;
      }
    }
  }

  corridor.anchor = anchor;
  corridor.frame = directions.frame;
  corridor.utility = directions.utility;
  corridor.hpoly = boundsToHPoly(anchor, directions.frame, lower, upper);
  corridor.metrics.face_count = corridor.hpoly.rows();
  corridor.metrics.direction_fallback = directions.used_fallback;
  corridor.metrics.valid = true;

  double min_slack = std::numeric_limits<double>::infinity();
  for (const Eigen::Vector3d &sample : samples)
  {
    min_slack = std::min(min_slack, pointSlack(corridor.hpoly, sample));
  }
  corridor.metrics.min_sample_slack = min_slack;

  Eigen::Vector3d widths = upper - lower;
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
  return (hpoly.leftCols<3>() * point - hpoly.col(3)).maxCoeff() <= tolerance;
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
    min_slack = std::min(min_slack,
                         (hpoly(row, 3) - hpoly.row(row).head<3>().dot(point)) /
                             normal_norm);
  }
  return min_slack;
}

bool DirectionalInflator::candidateIsFree(const Eigen::Vector3d &anchor,
                                          const Eigen::Matrix3d &frame,
                                          const Eigen::Vector3d &lower,
                                          const Eigen::Vector3d &upper,
                                          const double map_resolution,
                                          const OccupancyQuery &is_occupied) const
{
  Eigen::Vector3d aabb_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d aabb_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
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

  const double voxel_padding = 0.5 * std::sqrt(3.0) * map_resolution;
  aabb_min.array() -= voxel_padding;
  aabb_max.array() += voxel_padding;
  const Eigen::Vector3i begin = (aabb_min / map_resolution).array().floor().cast<int>();
  const Eigen::Vector3i end = (aabb_max / map_resolution).array().ceil().cast<int>();
  constexpr double kInsideTolerance = 1.0e-9;
  for (int x = begin.x(); x <= end.x(); ++x)
  {
    for (int y = begin.y(); y <= end.y(); ++y)
    {
      for (int z = begin.z(); z <= end.z(); ++z)
      {
        const Eigen::Vector3d world =
            (Eigen::Vector3d(x, y, z).array() + 0.5).matrix() * map_resolution;
        const Eigen::Vector3d local = frame.transpose() * (world - anchor);
        if ((local.array() >= lower.array() - voxel_padding - kInsideTolerance).all() &&
            (local.array() <= upper.array() + voxel_padding + kInsideTolerance).all() &&
            is_occupied(world))
        {
          return false;
        }
      }
    }
  }
  return true;
}

HPoly DirectionalInflator::boundsToHPoly(const Eigen::Vector3d &anchor,
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
