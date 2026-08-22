#pragma once

#include "optimizer/tf_sfc/tf_sfc_types.h"

#include <functional>

namespace ego_planner
{
namespace tf_sfc
{

class DirectionalInflator
{
public:
  enum class SpaceState
  {
    FREE,
    OCCUPIED,
    OUTSIDE_MAP
  };
  typedef std::function<SpaceState(const Eigen::Vector3d &)> OccupancyQuery;

  explicit DirectionalInflator(const Parameters &parameters);

  bool inflate(const PointVector &samples,
               const DirectionSet &directions,
               const double map_resolution,
               const OccupancyQuery &is_occupied,
               Corridor &corridor,
               FailureReason &failure_reason) const;

  static bool contains(const HPoly &hpoly,
                       const Eigen::Vector3d &point,
                       const double tolerance = 1.0e-8);

  static double pointSlack(const HPoly &hpoly,
                           const Eigen::Vector3d &point);

private:
  SpaceState candidateState(const Eigen::Vector3d &anchor,
                            const Eigen::Matrix3d &frame,
                            const Eigen::Vector3d &lower,
                            const Eigen::Vector3d &upper,
                            const double map_resolution,
                            const OccupancyQuery &is_occupied) const;

  static HPoly boundsToHPoly(const Eigen::Vector3d &anchor,
                            const Eigen::Matrix3d &frame,
                            const Eigen::Vector3d &lower,
                            const Eigen::Vector3d &upper);

  Parameters parameters_;
};

} // namespace tf_sfc
} // namespace ego_planner
