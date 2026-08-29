#include "optimizer/tf_sfc/tf_sfc_manager.h"

#include <ros/ros.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#ifdef TF_SFC_WITH_DECOMP_UTIL
#include <decomp_util/ellipsoid_decomp.h>
#endif

namespace ego_planner
{
namespace tf_sfc
{

namespace
{
double maxNormalizedHalfspaceViolation(const HPoly &hpoly,
                                       const Eigen::Vector3d &point)
{
  if (hpoly.rows() <= 0 || !hpoly.allFinite() || !point.allFinite())
  {
    return std::numeric_limits<double>::infinity();
  }
  double max_violation = -std::numeric_limits<double>::infinity();
  for (int face_id = 0; face_id < hpoly.rows(); ++face_id)
  {
    const Eigen::Vector3d normal =
        hpoly.row(face_id).head<3>().transpose();
    const double normal_norm = normal.norm();
    if (!std::isfinite(normal_norm) || normal_norm <= 1.0e-12)
    {
      return std::numeric_limits<double>::infinity();
    }
    max_violation = std::max(
        max_violation,
        (normal.dot(point) - hpoly(face_id, 3)) / normal_norm);
  }
  return max_violation;
}
} // namespace

TfSfcManager::TfSfcManager(const GridMap::Ptr &grid_map, const Parameters &parameters)
    : grid_map_(grid_map), parameters_(parameters), inflator_(parameters)
{
  objective_compliance_provider_.setTransportConditioning(
      parameters_.objective_compliance_transport_speed_reference,
      parameters_.objective_compliance_transport_weight_max);
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
    corridor.metrics.requested_direction_mode =
        static_cast<int>(parameters_.direction_mode);
    if (!computeDirections(piece, samples, piece_id, directions))
    {
      corridor.metrics.failure_reason = FailureReason::DIRECTION_FAILURE;
      prefix_active = false;
      continue;
    }
    corridor.metrics.used_direction_mode =
        static_cast<int>(directions.used_mode);

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


bool TfSfcManager::generateFromSeedPath(
    const poly_traj::Trajectory &direction_trajectory,
    const PointVector &seed_path,
    const FailureReason uncovered_failure_reason,
    CorridorVector &corridors)
{
  corridors.clear();
  corridors_.clear();
  const int trajectory_piece_count = direction_trajectory.getPieceNum();
  const int seed_piece_count = static_cast<int>(seed_path.size()) - 1;
  if (!grid_map_ || trajectory_piece_count <= 0 || seed_piece_count <= 0 ||
      seed_piece_count > trajectory_piece_count || parameters_.max_faces < 6)
  {
    return false;
  }

  corridors.resize(trajectory_piece_count);
  int valid_count = 0;
  bool prefix_active = true;
  for (int piece_id = 0; piece_id < trajectory_piece_count; ++piece_id)
  {
    Corridor &corridor = corridors[piece_id];
    corridor.metrics.piece_id = piece_id;
    if (!prefix_active)
    {
      corridor.metrics.failure_reason = FailureReason::SKIPPED_AFTER_FAILURE;
      continue;
    }
    if (piece_id >= seed_piece_count)
    {
      corridor.metrics.failure_reason =
          uncovered_failure_reason != FailureReason::NONE
              ? uncovered_failure_reason
              : FailureReason::PIECE_BUDGET_TAIL;
      prefix_active = false;
      continue;
    }

    const Eigen::Vector3d &segment_start = seed_path[piece_id];
    const Eigen::Vector3d &segment_finish = seed_path[piece_id + 1];
    if (!segment_start.allFinite() || !segment_finish.allFinite())
    {
      corridor.metrics.failure_reason = FailureReason::INVALID_INPUT;
      prefix_active = false;
      continue;
    }

    const auto started = std::chrono::steady_clock::now();
    const poly_traj::Piece &direction_piece = direction_trajectory[piece_id];
    const PointVector samples = sampleSegment(segment_start, segment_finish);
    DirectionSet directions;
    directions.requested_mode = parameters_.direction_mode;
    corridor.metrics.requested_direction_mode =
        static_cast<int>(parameters_.direction_mode);
    if (!computeDirections(direction_piece, samples, piece_id, directions))
    {
      corridor.metrics.failure_reason = FailureReason::DIRECTION_FAILURE;
      prefix_active = false;
      continue;
    }
    corridor.metrics.used_direction_mode =
        static_cast<int>(directions.used_mode);

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
      const Eigen::Vector3d &junction = seed_path[piece_id];
      const double radius =
          overlapRadius(corridors[piece_id - 1], corridor, junction);
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
  const bool enough_valid =
      valid_count >= std::max(parameters_.min_valid_pieces, 1);
  return enough_valid &&
         (parameters_.allow_partial_corridors ||
          valid_count == trajectory_piece_count);
}

bool TfSfcManager::ellipsoidDecompAvailable() const
{
#ifdef TF_SFC_WITH_DECOMP_UTIL
  return true;
#else
  return false;
#endif
}

bool TfSfcManager::generateEllipsoidDecomp(const PointVector &seed_path,
                                           const int expected_piece_count,
                                           const FailureReason uncovered_failure_reason,
                                           CorridorVector &corridors)
{
  corridors.clear();
  corridors_.clear();
#ifndef TF_SFC_WITH_DECOMP_UTIL
  Corridor failed;
  failed.metrics.piece_id = 0;
  failed.metrics.failure_reason = FailureReason::DECOMP_UTIL_UNAVAILABLE;
  corridors.push_back(failed);
  corridors_ = corridors;
  return false;
#else
  if (!grid_map_ || seed_path.size() < 2 || expected_piece_count <= 0 ||
      static_cast<int>(seed_path.size()) - 1 > expected_piece_count)
  {
    Corridor failed;
    failed.metrics.piece_id = 0;
    failed.metrics.failure_reason = FailureReason::INVALID_INPUT;
    corridors.push_back(failed);
    corridors_ = corridors;
    return false;
  }

  vec_Vec3f path;
  path.reserve(seed_path.size());
  for (const Eigen::Vector3d &point : seed_path)
  {
    if (!point.allFinite())
    {
      Corridor failed;
      failed.metrics.piece_id = static_cast<int>(path.size());
      failed.metrics.failure_reason = FailureReason::INVALID_INPUT;
      corridors.push_back(failed);
      corridors_ = corridors;
      return false;
    }
    if (!grid_map_->isInInflatedMap(point))
    {
      Corridor failed;
      failed.metrics.piece_id = static_cast<int>(path.size());
      failed.metrics.failure_reason = FailureReason::SEED_PATH_OUTSIDE_MAP;
      corridors.push_back(failed);
      corridors_ = corridors;
      return false;
    }
    if (grid_map_->getInflateOccupancy(point) != 0)
    {
      Corridor failed;
      failed.metrics.piece_id = static_cast<int>(path.size());
      failed.metrics.failure_reason = FailureReason::SEED_PATH_OCCUPIED;
      corridors.push_back(failed);
      corridors_ = corridors;
      return false;
    }
    if (path.empty() || (point - path.back()).norm() > 1.0e-6)
    {
      path.push_back(point);
    }
  }
  if (path.size() < 2)
  {
    Corridor failed;
    failed.metrics.piece_id = 0;
    failed.metrics.failure_reason = FailureReason::SEED_PATH_FAILURE;
    corridors.push_back(failed);
    corridors_ = corridors;
    return false;
  }

  Eigen::Vector3d map_low, map_high;
  grid_map_->getInflatedMapBounds(map_low, map_high);
  const double padding = std::max(
      parameters_.decomp_local_bbox_forward,
      std::max(parameters_.decomp_local_bbox_lateral,
               parameters_.decomp_local_bbox_vertical));
  Eigen::Vector3d query_low = path.front();
  Eigen::Vector3d query_high = path.front();
  for (const Vec3f &point : path)
  {
    query_low = query_low.cwiseMin(point);
    query_high = query_high.cwiseMax(point);
  }
  query_low = (query_low.array() - padding).matrix().cwiseMax(map_low);
  query_high = (query_high.array() + padding).matrix().cwiseMin(map_high);

  vec_Vec3f obstacles;
  const double resolution = grid_map_->getResolution();
  const Eigen::Vector3d first =
      (query_low.array() / resolution).ceil().matrix() * resolution;
  for (double x = first.x(); x <= query_high.x() + 1.0e-9; x += resolution)
  {
    for (double y = first.y(); y <= query_high.y() + 1.0e-9; y += resolution)
    {
      for (double z = first.z(); z <= query_high.z() + 1.0e-9; z += resolution)
      {
        const Eigen::Vector3d point(x, y, z);
        if (grid_map_->isInInflatedMap(point) &&
            grid_map_->getInflateOccupancy(point) != 0)
        {
          obstacles.push_back(point);
        }
      }
    }
  }

  const auto started = std::chrono::steady_clock::now();
  Vec3f local_bbox;
  local_bbox << std::max(parameters_.decomp_local_bbox_forward, resolution),
                std::max(parameters_.decomp_local_bbox_lateral, resolution),
                std::max(parameters_.decomp_local_bbox_vertical, resolution);
  vec_E<Polyhedron3D> polyhedrons;
  polyhedrons.reserve(path.size() - 1);
  bool decomp_ok = true;
  const auto segment_is_free = [&](const Vec3f &start, const Vec3f &finish) {
    const auto point_is_free = [&](const Vec3f &point) {
      return grid_map_->isInInflatedMap(point) &&
             grid_map_->getInflateOccupancy(point) == 0;
    };
    if (!point_is_free(start) || !point_is_free(finish))
    {
      return false;
    }
    RayCaster raycaster;
    Eigen::Vector3d voxel;
    if (raycaster.setInput(start / resolution, finish / resolution))
    {
      while (raycaster.step(voxel))
      {
        const Vec3f center =
            ((voxel.array() + 0.5) * resolution).matrix();
        if (!point_is_free(center))
        {
          return false;
        }
      }
    }
    return true;
  };
  for (int piece_id = 0; piece_id + 1 < static_cast<int>(path.size()); ++piece_id)
  {
    Vec3f segment_start = path[piece_id];
    Vec3f segment_finish = path[piece_id + 1];
    const double requested_extension =
        std::max(parameters_.decomp_overlap_extension, 0.0);
    const Vec3f segment_delta = segment_finish - segment_start;
    const double segment_length = segment_delta.norm();
    const Vec3f segment_direction =
        segment_delta / std::max(segment_length, 1.0e-9);

    // Make each shared junction an interior point of both neighboring seed
    // segments. Extend along the segment tangent, collision-check the added
    // portion, and back off rather than weakening the overlap certificate.
    if (piece_id > 0 && requested_extension > 0.0 && segment_length > 1.0e-9)
    {
      double extension = std::min(requested_extension, 0.45 * segment_length);
      while (extension >= 0.25 * resolution)
      {
        const Vec3f candidate = path[piece_id] - extension * segment_direction;
        if (segment_is_free(candidate, path[piece_id]))
        {
          segment_start = candidate;
          break;
        }
        extension *= 0.5;
      }
    }
    if (piece_id + 2 < static_cast<int>(path.size()) &&
        requested_extension > 0.0 && segment_length > 1.0e-9)
    {
      double extension = std::min(requested_extension, 0.45 * segment_length);
      while (extension >= 0.25 * resolution)
      {
        const Vec3f candidate =
            path[piece_id + 1] + extension * segment_direction;
        if (segment_is_free(path[piece_id + 1], candidate))
        {
          segment_finish = candidate;
          break;
        }
        extension *= 0.5;
      }
    }

    vec_Vec3f segment_path;
    segment_path.push_back(segment_start);
    segment_path.push_back(segment_finish);
    EllipsoidDecomp3D segment_decomp;
    segment_decomp.set_obs(obstacles);
    segment_decomp.set_local_bbox(local_bbox);
    segment_decomp.dilate(segment_path);
    const vec_E<Polyhedron3D> segment_polyhedrons =
        segment_decomp.get_polyhedrons();
    if (segment_polyhedrons.size() != 1)
    {
      decomp_ok = false;
      break;
    }
    polyhedrons.push_back(segment_polyhedrons.front());
  }
  const double total_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - started)
                              .count();
  if (!decomp_ok || polyhedrons.size() + 1 != path.size())
  {
    Corridor failed;
    failed.metrics.piece_id = 0;
    failed.metrics.failure_reason = FailureReason::DECOMP_FAILURE;
    failed.metrics.generation_time_ms = total_ms;
    corridors.push_back(failed);
    corridors_ = corridors;
    return false;
  }

  corridors.resize(expected_piece_count);
  for (int piece_id = 0; piece_id < expected_piece_count; ++piece_id)
  {
    corridors[piece_id].metrics.piece_id = piece_id;
    if (piece_id == static_cast<int>(polyhedrons.size()))
    {
      corridors[piece_id].metrics.failure_reason =
          uncovered_failure_reason == FailureReason::NONE
              ? FailureReason::OUTSIDE_LOCAL_MAP
              : uncovered_failure_reason;
    }
    else if (piece_id > static_cast<int>(polyhedrons.size()))
    {
      corridors[piece_id].metrics.failure_reason = FailureReason::SKIPPED_AFTER_FAILURE;
    }
  }
  int valid_count = 0;
  const auto finish_prefix = [&]() {
    corridors_ = corridors;
    const bool enough_valid =
        valid_count >= std::max(parameters_.min_valid_pieces, 1);
    return enough_valid &&
           (parameters_.allow_partial_corridors ||
            valid_count == expected_piece_count);
  };
  const auto mark_remaining_skipped = [&](const int first_piece_id) {
    for (int skipped_id = first_piece_id;
         skipped_id < expected_piece_count; ++skipped_id)
    {
      corridors[skipped_id].metrics.piece_id = skipped_id;
      corridors[skipped_id].metrics.failure_reason =
          FailureReason::SKIPPED_AFTER_FAILURE;
    }
  };
  for (int piece_id = 0; piece_id < static_cast<int>(polyhedrons.size()); ++piece_id)
  {
    Corridor &corridor = corridors[piece_id];
    corridor.metrics.piece_id = piece_id;
    corridor.metrics.generation_time_ms =
        total_ms / static_cast<double>(polyhedrons.size());
    const vec_E<Hyperplane3D> planes = polyhedrons[piece_id].hyperplanes();
    corridor.hpoly.resize(planes.size() + 6, 4);
    int row = 0;
    for (const Hyperplane3D &plane : planes)
    {
      Eigen::Vector3d normal = plane.n_;
      const double norm = normal.norm();
      if (!normal.allFinite() || norm <= 1.0e-9)
      {
        corridor.metrics.failure_reason = FailureReason::DECOMP_FAILURE;
        mark_remaining_skipped(piece_id + 1);
        return finish_prefix();
      }
      normal /= norm;
      corridor.hpoly.row(row).head<3>() = normal.transpose();
      corridor.hpoly(row++, 3) = normal.dot(plane.p_);
    }
    corridor.hpoly.row(row++) << 1.0, 0.0, 0.0, map_high.x();
    corridor.hpoly.row(row++) << -1.0, 0.0, 0.0, -map_low.x();
    corridor.hpoly.row(row++) << 0.0, 1.0, 0.0, map_high.y();
    corridor.hpoly.row(row++) << 0.0, -1.0, 0.0, -map_low.y();
    corridor.hpoly.row(row++) << 0.0, 0.0, 1.0, map_high.z();
    corridor.hpoly.row(row++) << 0.0, 0.0, -1.0, -map_low.z();

    corridor.anchor = 0.5 * (path[piece_id] + path[piece_id + 1]);
    corridor.metrics.seed_containment_evaluated = true;
    corridor.metrics.seed_containment_max_violation_m = std::max(
        maxNormalizedHalfspaceViolation(corridor.hpoly, path[piece_id]),
        maxNormalizedHalfspaceViolation(corridor.hpoly, path[piece_id + 1]));
    corridor.metrics.seed_contained =
        corridor.metrics.seed_containment_max_violation_m <= 1.0e-6;
    // A convex polytope containing both endpoints contains the complete line
    // seed. This is the manageability check used by the corridor benchmark.
    const bool valid = corridor.hpoly.allFinite() &&
                       corridor.metrics.seed_contained;
    if (!valid)
    {
      corridor.metrics.failure_reason = FailureReason::DECOMP_FAILURE;
      mark_remaining_skipped(piece_id + 1);
      return finish_prefix();
    }
    const Eigen::Vector3d midpoint = 0.5 * (path[piece_id] + path[piece_id + 1]);
    corridor.metrics.min_sample_slack = std::min(
        DirectionalInflator::pointSlack(corridor.hpoly, path[piece_id]),
        std::min(DirectionalInflator::pointSlack(corridor.hpoly, midpoint),
                 DirectionalInflator::pointSlack(corridor.hpoly, path[piece_id + 1])));
    corridor.metrics.weighted_width = 2.0 * corridor.metrics.min_sample_slack;
    corridor.metrics.face_count = corridor.hpoly.rows();
    corridor.metrics.valid = true;
    corridor.metrics.failure_reason = FailureReason::NONE;

    if (piece_id > 0)
    {
      const double radius = overlapRadius(corridors[piece_id - 1], corridor,
                                          path[piece_id]);
      corridors[piece_id - 1].metrics.overlap_radius_to_next = radius;
      if (radius + 1.0e-9 < parameters_.min_overlap_radius)
      {
        corridor.metrics.valid = false;
        corridor.metrics.failure_reason = FailureReason::OVERLAP_TOO_SMALL;
        mark_remaining_skipped(piece_id + 1);
        return finish_prefix();
      }
    }
    ++valid_count;
  }

  return finish_prefix();
#endif
}

void TfSfcManager::clearCorridors()
{
  corridors_.clear();
  corridor_penalty_scale_ = 1.0;
}

void TfSfcManager::setCorridorPenaltyScale(const double scale)
{
  corridor_penalty_scale_ = std::max(scale, 1.0);
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
      const double effective_weight =
          parameters_.weight * corridor_penalty_scale_;
      cost += effective_weight * violation * violation;
      gradient.noalias() += 2.0 * effective_weight * violation * normal;
    }
  }
  return active;
}

CorridorEvaluation TfSfcManager::evaluateTrajectory(
    const poly_traj::Trajectory &trajectory) const
{
  CorridorEvaluation evaluation;
  const int piece_count = std::min(
      trajectory.getPieceNum(), static_cast<int>(corridors_.size()));
  const int sample_count = std::max(parameters_.samples_per_piece, 2);
  for (int piece_id = 0; piece_id < piece_count; ++piece_id)
  {
    const Corridor &corridor = corridors_[piece_id];
    if (!corridor.metrics.valid || corridor.hpoly.rows() == 0)
    {
      continue;
    }
    ++evaluation.constrained_piece_count;
    const poly_traj::Piece &piece = trajectory[piece_id];
    const double step = piece.getDuration() /
                        static_cast<double>(sample_count);
    for (int sample_id = 0; sample_id <= sample_count; ++sample_id)
    {
      const double time = step * static_cast<double>(sample_id);
      const Eigen::Vector3d point = piece.getPos(time);
      double sample_penalty = 0.0;
      for (int face_id = 0; face_id < corridor.hpoly.rows(); ++face_id)
      {
        const Eigen::Vector3d normal =
            corridor.hpoly.row(face_id).head<3>().transpose();
        const double normal_norm = normal.norm();
        if (normal_norm <= 1.0e-12)
        {
          continue;
        }
        ++evaluation.evaluated_face_sample_count;
        const double signed_violation =
            normal.dot(point) - corridor.hpoly(face_id, 3);
        const double normalized_violation =
            signed_violation / normal_norm;
        if (normalized_violation > 0.0)
        {
          ++evaluation.violating_face_sample_count;
        }
        if (normalized_violation > evaluation.max_violation_m)
        {
          evaluation.max_violation_m = normalized_violation;
          evaluation.max_violation_piece_id = piece_id;
          evaluation.max_violation_face_id = face_id;
          evaluation.max_violation_sample_id = sample_id;
          evaluation.max_violation_time_ratio =
              static_cast<double>(sample_id) /
              static_cast<double>(sample_count);
        }
        const double buffered_violation =
            signed_violation + parameters_.penalty_epsilon;
        if (parameters_.use_soft_penalty && buffered_violation > 0.0)
        {
          sample_penalty += parameters_.weight * corridor_penalty_scale_ *
                            buffered_violation * buffered_violation;
        }
      }
      const double quadrature_weight =
          (sample_id == 0 || sample_id == sample_count) ? 0.5 : 1.0;
      evaluation.penalty_cost += quadrature_weight * step * sample_penalty;
    }
  }
  return evaluation;
}

double TfSfcManager::pieceViolation(const poly_traj::Piece &piece,
                                    const Corridor &corridor) const
{
  if (!corridor.metrics.valid || corridor.hpoly.rows() <= 0)
  {
    return 0.0;
  }
  double max_violation = 0.0;
  const PointVector samples = samplePiece(piece);
  for (const Eigen::Vector3d &point : samples)
  {
    max_violation = std::max(
        max_violation,
        maxNormalizedHalfspaceViolation(corridor.hpoly, point));
  }
  return max_violation;
}

bool TfSfcManager::repairWorstCorridorForTrajectory(
    const poly_traj::Trajectory &trajectory,
    const PointVector &seed_path,
    CorridorVector &corridors,
    TrajectoryRepairResult &result)
{
  result = TrajectoryRepairResult();
  result.evaluated = true;
  if (!parameters_.trajectory_repair_enabled)
  {
    result.reason = "disabled";
    return false;
  }
  if (parameters_.corridor_method != "tf_sfc")
  {
    result.reason = "method_not_tf_sfc";
    return false;
  }
  if (!grid_map_ || trajectory.getPieceNum() <= 0 ||
      static_cast<int>(corridors.size()) != trajectory.getPieceNum())
  {
    result.reason = "invalid_input";
    return false;
  }

  corridors_ = corridors;
  const CorridorEvaluation before = evaluateTrajectory(trajectory);
  result.global_violation_before_m = before.max_violation_m;
  result.global_violation_after_m = before.max_violation_m;
  result.piece_id = before.max_violation_piece_id;
  if (before.max_violation_m <= parameters_.trajectory_repair_trigger)
  {
    result.reason = "below_trigger";
    return false;
  }
  if (result.piece_id < 0 || result.piece_id >= trajectory.getPieceNum() ||
      !corridors[result.piece_id].metrics.valid)
  {
    result.reason = "invalid_worst_piece";
    return false;
  }

  const int piece_id = result.piece_id;
  const poly_traj::Piece &piece = trajectory[piece_id];
  result.piece_violation_before_m =
      pieceViolation(piece, corridors[piece_id]);
  result.piece_violation_after_m = result.piece_violation_before_m;
  result.attempted = true;

  const auto started = std::chrono::steady_clock::now();
  const PointVector trajectory_samples = samplePiece(piece);
  DirectionSet directions;
  directions.requested_mode = parameters_.direction_mode;
  Corridor candidate;
  if (!computeDirections(piece, trajectory_samples, piece_id, directions))
  {
    result.reason = "direction_failure";
    result.generation_time_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
    return false;
  }
  const auto inflate_candidate =
      [&](const PointVector &inflation_samples,
          FailureReason &failure_reason) {
        candidate = Corridor();
        candidate.metrics.piece_id = piece_id;
        candidate.metrics.requested_direction_mode =
            static_cast<int>(parameters_.direction_mode);
        candidate.metrics.used_direction_mode =
            static_cast<int>(directions.used_mode);
        const bool inflated = inflator_.inflate(
            inflation_samples, directions, grid_map_->getResolution(),
            [this](const Eigen::Vector3d &point) {
              if (!grid_map_->isInInflatedMap(point))
              {
                return DirectionalInflator::SpaceState::OUTSIDE_MAP;
              }
              return grid_map_->getInflateOccupancy(point) != 0
                         ? DirectionalInflator::SpaceState::OCCUPIED
                         : DirectionalInflator::SpaceState::FREE;
            },
            candidate, failure_reason);
        candidate.metrics.seed_containment_evaluated = true;
        candidate.metrics.seed_containment_max_violation_m = 0.0;
        for (const Eigen::Vector3d &sample : inflation_samples)
        {
          candidate.metrics.seed_containment_max_violation_m = std::max(
              candidate.metrics.seed_containment_max_violation_m,
              maxNormalizedHalfspaceViolation(candidate.hpoly, sample));
        }
        candidate.metrics.seed_contained =
            inflated &&
            candidate.metrics.seed_containment_max_violation_m <= 1.0e-6;
        return inflated;
      };

  FailureReason failure_reason = FailureReason::NONE;
  bool inflated = inflate_candidate(trajectory_samples, failure_reason);
  result.primary_failure_reason = failureReasonName(failure_reason);
  const bool retryable_with_seed =
      failure_reason == FailureReason::INITIAL_OBB_OCCUPIED ||
      failure_reason == FailureReason::OBSTACLE_SEPARATION_FAILURE ||
      failure_reason == FailureReason::FACE_BUDGET_EXHAUSTED;
  if (!inflated && parameters_.trajectory_repair_seed_fallback_enabled &&
      retryable_with_seed && piece_id + 1 < static_cast<int>(seed_path.size()) &&
      seed_path[piece_id].allFinite() && seed_path[piece_id + 1].allFinite())
  {
    result.seed_fallback_used = true;
    result.sample_source = "collision_free_seed";
    const PointVector seed_samples =
        sampleSegment(seed_path[piece_id], seed_path[piece_id + 1]);
    failure_reason = FailureReason::NONE;
    inflated = inflate_candidate(seed_samples, failure_reason);
  }
  result.generation_time_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();
  candidate.metrics.generation_time_ms = result.generation_time_ms;
  result.candidate_face_count = candidate.metrics.face_count;
  if (!inflated)
  {
    result.reason = result.seed_fallback_used
                        ? std::string("seed_inflation_") +
                              failureReasonName(failure_reason)
                        : std::string("trajectory_inflation_") +
                              failureReasonName(failure_reason);
    return false;
  }
  if (candidate.hpoly.rows() > parameters_.max_faces)
  {
    result.reason = "face_budget_exceeded";
    return false;
  }

  result.candidate_weighted_width_m = candidate.metrics.weighted_width;
  const bool seed_geometry_candidate =
      result.seed_fallback_used &&
      parameters_.trajectory_repair_seed_geometric_acceptance_enabled;
  result.overlap_anchor_source = seed_geometry_candidate
                                     ? "collision_free_seed_junction"
                                     : "trajectory_junction";

  if (piece_id > 0 && corridors[piece_id - 1].metrics.valid)
  {
    // A seed-based candidate intentionally need not contain the colliding
    // pre-repair MINCO curve. Its common seed junction is nevertheless inside
    // both neighboring seed corridors and is the correct hard-parameterization
    // anchor for the next optimization.
    const Eigen::Vector3d junction =
        seed_geometry_candidate ? seed_path[piece_id]
                                : trajectory.getJuncPos(piece_id);
    result.overlap_previous_m =
        overlapRadius(corridors[piece_id - 1], candidate, junction);
    if (result.overlap_previous_m + 1.0e-9 <
        parameters_.min_overlap_radius)
    {
      result.reason = "overlap_previous_too_small";
      return false;
    }
  }
  if (piece_id + 1 < trajectory.getPieceNum() &&
      corridors[piece_id + 1].metrics.valid)
  {
    const Eigen::Vector3d junction =
        seed_geometry_candidate ? seed_path[piece_id + 1]
                                : trajectory.getJuncPos(piece_id + 1);
    result.overlap_next_m =
        overlapRadius(candidate, corridors[piece_id + 1], junction);
    if (result.overlap_next_m + 1.0e-9 < parameters_.min_overlap_radius)
    {
      result.reason = "overlap_next_too_small";
      return false;
    }
  }

  CorridorVector trial = corridors;
  trial[piece_id] = candidate;
  if (piece_id > 0 && trial[piece_id - 1].metrics.valid)
  {
    trial[piece_id - 1].metrics.overlap_radius_to_next =
        result.overlap_previous_m;
  }
  if (piece_id + 1 < trajectory.getPieceNum() &&
      trial[piece_id + 1].metrics.valid)
  {
    trial[piece_id].metrics.overlap_radius_to_next =
        result.overlap_next_m;
  }

  corridors_ = trial;
  const CorridorEvaluation after = evaluateTrajectory(trajectory);
  result.global_violation_after_m = after.max_violation_m;
  result.piece_violation_after_m = pieceViolation(piece, candidate);
  if (seed_geometry_candidate)
  {
    const bool bounded_geometry =
        candidate.metrics.valid && candidate.metrics.seed_contained &&
        candidate.hpoly.rows() >= 6 &&
        candidate.hpoly.rows() <= parameters_.max_faces &&
        std::isfinite(candidate.metrics.weighted_width) &&
        candidate.metrics.weighted_width > 0.0;
    if (!bounded_geometry)
    {
      corridors_ = corridors;
      result.reason = "seed_geometry_quality_failure";
      return false;
    }
    // Do not require an obstacle-intersecting old curve to improve inside a
    // corridor built around the collision-free seed. Hard junction mapping,
    // one bounded optimization and the unchanged final safety gates decide
    // whether this geometry can become an executable trajectory.
    result.accepted = true;
    result.geometric_acceptance_used = true;
    result.reason = "accepted_seed_geometry";
    corridors = trial;
    corridors_ = corridors;
    return true;
  }
  const bool piece_improved =
      result.piece_violation_after_m +
              parameters_.trajectory_repair_min_improvement <
          result.piece_violation_before_m ||
      result.piece_violation_after_m <= parameters_.max_final_violation;
  const bool globally_nonworsening =
      result.global_violation_after_m <=
      result.global_violation_before_m + 1.0e-9;
  if (!piece_improved)
  {
    corridors_ = corridors;
    result.reason = "piece_not_improved";
    return false;
  }
  if (!globally_nonworsening)
  {
    corridors_ = corridors;
    result.reason = "global_violation_worsened";
    return false;
  }

  result.accepted = true;
  result.reason = "accepted";
  corridors = trial;
  corridors_ = corridors;
  return true;
}

void TfSfcManager::setPieceSensitivityGramians(
    const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &gramians)
{
  sensitivity_provider_.setPieceGramians(gramians);
}

void TfSfcManager::setPieceObjectiveCompliances(
    const std::vector<Eigen::Matrix3d,
                      Eigen::aligned_allocator<Eigen::Matrix3d>> &compliances)
{
  objective_compliance_provider_.setPieceCompliances(compliances);
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
  else if (parameters_.direction_mode ==
           DirectionMode::FULL_OBJECTIVE_COMPLIANCE)
  {
    requested = &objective_compliance_provider_;
  }

  if (requested->computeDirections(piece, samples, piece_id, directions))
  {
    return true;
  }

  if (!parameters_.allow_direction_fallback)
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "TF-SFC direction provider %d failed and direction fallback is disabled.",
        static_cast<int>(parameters_.direction_mode));
    return false;
  }

  directions.used_fallback = true;
  if (parameters_.direction_mode == DirectionMode::SENSITIVITY)
  {
    ROS_WARN_THROTTLE(1.0, "TF-SFC sensitivity Gramian unavailable; falling back to PCA directions.");
  }
  else if (parameters_.direction_mode ==
           DirectionMode::FULL_OBJECTIVE_COMPLIANCE)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "TF-SFC full-objective compliance unavailable; falling back to PCA directions.");
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

PointVector TfSfcManager::sampleSegment(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &finish) const
{
  const int count = std::max(parameters_.samples_per_piece, 2);
  PointVector samples;
  samples.reserve(count + 1);
  for (int i = 0; i <= count; ++i)
  {
    const double ratio =
        static_cast<double>(i) / static_cast<double>(count);
    samples.push_back(start + ratio * (finish - start));
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
