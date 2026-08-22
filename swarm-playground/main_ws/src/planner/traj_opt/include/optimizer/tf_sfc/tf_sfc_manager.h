#pragma once

#include "optimizer/tf_sfc/direction_provider.h"
#include "optimizer/tf_sfc/directional_inflator.h"

#include <plan_env/grid_map.h>

namespace ego_planner
{
namespace tf_sfc
{

class TfSfcManager
{
public:
  TfSfcManager(const GridMap::Ptr &grid_map, const Parameters &parameters);

  bool generate(const poly_traj::Trajectory &trajectory, CorridorVector &corridors);

  void clearCorridors();

  bool projectJunctions(Eigen::MatrixXd &inner_points,
                        const CorridorVector &corridors) const;

  bool corridorGradCost(const int piece_id,
                        const Eigen::Vector3d &point,
                        Eigen::Vector3d &gradient,
                        double &cost) const;

  void setPieceSensitivityGramians(
      const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &gramians);

  const CorridorVector &corridors() const { return corridors_; }
  const Parameters &parameters() const { return parameters_; }

private:
  bool computeDirections(const poly_traj::Piece &piece,
                         const PointVector &samples,
                         int piece_id,
                         DirectionSet &directions) const;

  PointVector samplePiece(const poly_traj::Piece &piece) const;
  double overlapRadius(const Corridor &lhs, const Corridor &rhs,
                       const Eigen::Vector3d &shared_point) const;

  GridMap::Ptr grid_map_;
  Parameters parameters_;
  FrenetDirectionProvider frenet_provider_;
  PcaDirectionProvider pca_provider_;
  SensitivityDirectionProvider sensitivity_provider_;
  DirectionalInflator inflator_;
  CorridorVector corridors_;
};

} // namespace tf_sfc
} // namespace ego_planner
