#include "optimizer/tf_sfc/tf_sfc_manager.h"

#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace ego_planner
{
namespace tf_sfc
{

TfSfcManager::TfSfcManager(const GridMap::Ptr &grid_map, const Parameters &parameters)
    : grid_map_(grid_map), parameters_(parameters), inflator_(parameters)
{
}

bool TfSfcManager::generate(const poly_traj::Trajectory &trajectory,
                            CorridorVector &corridors)
{
  corridors.clear();
  corridors_.clear();
  if (!grid_map_ || trajectory.getPieceNum() <= 0 || parameters_.max_faces < 6)
  {
    return false;
  }

  corridors.resize(trajectory.getPieceNum());
  int valid_count = 0;
  bool prefix_active = true;
  for (int piece_id = 0; piece_id < trajectory.getPieceNum(); ++piece_id)
  {
    Corridor &corridor = corridors[piece_id];
    corridor.metrics.piece_id = piece_id;
    if (!prefix_active)
    {
      corridor.metrics.failure_reason = FailureReason::SKIPPED_AFTER_FAILURE;
      continue;
    }

    const auto started = std::chrono::steady_clock::now();
    const poly_traj::Piece &piece = trajectory[piece_id];
    const PointVector samples = samplePiece(piece);
    DirectionSet directions;
    directions.requested_mode = parameters_.direction_mode;
    if (!computeDirections(piece, samples, piece_id, directions))
    {
      corridor.metrics.failure_reason = FailureReason::DIRECTION_FAILURE;
      prefix_active = false;
      continue;
    }

    FailureReason failure_reason = FailureReason::NONE;
    const bool inflated = inflator_.inflate(
        samples, directions, grid_map_->getResolution(),
        [this](const Eigen::Vector3d &point) {
          if (!grid_map_->isInInflatedMap(point))
          {
            return DirectionalInflator::SpaceState::OUTSIDE_MAP;
          }
          return grid_map_->getInflateOccupancy(point) != 0
                     ? DirectionalInflator::SpaceState::OCCUPIED
                     : DirectionalInflator::SpaceState::FREE;
        },
        corridor, failure_reason);
    const auto finished = std::chrono::steady_clock::now();
    corridor.metrics.generation_time_ms =
        std::chrono::duration<double, std::milli>(finished - started).count();
    if (!inflated)
    {
      corridor.metrics.failure_reason = failure_reason;
      prefix_active = false;
      continue;
    }
    corridor.metrics.failure_reason = FailureReason::NONE;

    if (piece_id > 0 && corridors[piece_id - 1].metrics.valid)
    {
      const Eigen::Vector3d junction = trajectory.getJuncPos(piece_id);
      const double radius = overlapRadius(corridors[piece_id - 1], corridor, junction);
      corridors[piece_id - 1].metrics.overlap_radius_to_next = radius;
      if (radius + 1.0e-9 < parameters_.min_overlap_radius)
      {
        corridor.metrics.valid = false;
        corridor.metrics.failure_reason = FailureReason::OVERLAP_TOO_SMALL;
        prefix_active = false;
        continue;
      }
    }
    ++valid_count;
  }

  corridors_ = corridors;
  const bool enough_valid = valid_count >= std::max(parameters_.min_valid_pieces, 1);
  return enough_valid &&
         (parameters_.allow_partial_corridors || valid_count == trajectory.getPieceNum());
}

void TfSfcManager::clearCorridors()
{
  corridors_.clear();
}

bool TfSfcManager::projectJunctions(Eigen::MatrixXd &inner_points,
                                    const CorridorVector &corridors) const
{
  if (inner_points.rows() != 3 ||
      inner_points.cols() + 1 != static_cast<int>(corridors.size()))
  {
    return false;
  }

  Eigen::MatrixXd projected = inner_points;
  for (int junction_id = 0; junction_id < projected.cols(); ++junction_id)
  {
    if (!corridors[junction_id].metrics.valid ||
        !corridors[junction_id + 1].metrics.valid)
    {
      continue;
    }
    Eigen::Vector3d point = projected.col(junction_id);
    for (int pass = 0; pass < parameters_.projection_passes; ++pass)
    {
      for (int corridor_id = junction_id; corridor_id <= junction_id + 1; ++corridor_id)
      {
        const HPoly &hpoly = corridors[corridor_id].hpoly;
        for (int row = 0; row < hpoly.rows(); ++row)
        {
          const Eigen::Vector3d normal = hpoly.row(row).head<3>().transpose();
          const double violation = normal.dot(point) - hpoly(row, 3);
          if (violation > 0.0)
          {
            point -= violation / normal.squaredNorm() * normal;
          }
        }
      }
    }

    if (!DirectionalInflator::contains(corridors[junction_id].hpoly, point, 1.0e-6) ||
        !DirectionalInflator::contains(corridors[junction_id + 1].hpoly, point, 1.0e-6))
    {
      return false;
    }
    projected.col(junction_id) = point;
  }

  inner_points = projected;
  return true;
}

bool TfSfcManager::corridorGradCost(const int piece_id,
                                    const Eigen::Vector3d &point,
                                    Eigen::Vector3d &gradient,
                                    double &cost) const
{
  gradient.setZero();
  cost = 0.0;
  if (!parameters_.use_soft_penalty || piece_id < 0 ||
      piece_id >= static_cast<int>(corridors_.size()) ||
      !corridors_[piece_id].metrics.valid)
  {
    return false;
  }

  bool active = false;
  const HPoly &hpoly = corridors_[piece_id].hpoly;
  for (int row = 0; row < hpoly.rows(); ++row)
  {
    const Eigen::Vector3d normal = hpoly.row(row).head<3>().transpose();
    const double violation = normal.dot(point) - hpoly(row, 3) + parameters_.penalty_epsilon;
    if (violation > 0.0)
    {
      active = true;
      cost += parameters_.weight * violation * violation;
      gradient.noalias() += 2.0 * parameters_.weight * violation * normal;
    }
  }
  return active;
}

void TfSfcManager::setPieceSensitivityGramians(
    const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &gramians)
{
  sensitivity_provider_.setPieceGramians(gramians);
}

bool TfSfcManager::computeDirections(const poly_traj::Piece &piece,
                                     const PointVector &samples,
                                     const int piece_id,
                                     DirectionSet &directions) const
{
  directions.used_fallback = false;
  const DirectionProvider *requested = &pca_provider_;
  if (parameters_.direction_mode == DirectionMode::FRENET)
  {
    requested = &frenet_provider_;
  }
  else if (parameters_.direction_mode == DirectionMode::SENSITIVITY)
  {
    requested = &sensitivity_provider_;
  }

  if (requested->computeDirections(piece, samples, piece_id, directions))
  {
    return true;
  }

  directions.used_fallback = true;
  if (parameters_.direction_mode == DirectionMode::SENSITIVITY)
  {
    ROS_WARN_THROTTLE(1.0, "TF-SFC sensitivity Gramian unavailable; falling back to PCA directions.");
  }
  if (pca_provider_.computeDirections(piece, samples, piece_id, directions))
  {
    directions.used_fallback = true;
    return true;
  }
  if (frenet_provider_.computeDirections(piece, samples, piece_id, directions))
  {
    directions.used_fallback = true;
    return true;
  }
  return false;
}

PointVector TfSfcManager::samplePiece(const poly_traj::Piece &piece) const
{
  const int count = std::max(parameters_.samples_per_piece, 2);
  PointVector samples;
  samples.reserve(count + 1);
  for (int i = 0; i <= count; ++i)
  {
    samples.push_back(piece.getPos(piece.getDuration() * static_cast<double>(i) /
                                   static_cast<double>(count)));
  }
  return samples;
}

double TfSfcManager::overlapRadius(const Corridor &lhs, const Corridor &rhs,
                                   const Eigen::Vector3d &shared_point) const
{
  return std::min(DirectionalInflator::pointSlack(lhs.hpoly, shared_point),
                  DirectionalInflator::pointSlack(rhs.hpoly, shared_point));
}

} // namespace tf_sfc
} // namespace ego_planner
