#include "optimizer/tf_sfc/direction_provider.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>

namespace ego_planner
{
namespace tf_sfc
{
namespace
{

constexpr double kDirectionEpsilon = 1.0e-8;

bool makeRightHandedFrame(const Eigen::Vector3d &primary,
                          const Eigen::Vector3d &secondary_hint,
                          Eigen::Matrix3d &frame)
{
  if (primary.norm() < kDirectionEpsilon)
  {
    return false;
  }

  Eigen::Vector3d e0 = primary.normalized();
  Eigen::Vector3d e1 = secondary_hint - e0 * e0.dot(secondary_hint);
  if (e1.norm() < kDirectionEpsilon)
  {
    const Eigen::Vector3d helper = std::abs(e0.z()) < 0.9
                                       ? Eigen::Vector3d::UnitZ()
                                       : Eigen::Vector3d::UnitY();
    e1 = helper - e0 * e0.dot(helper);
  }
  if (e1.norm() < kDirectionEpsilon)
  {
    return false;
  }

  e1.normalize();
  const Eigen::Vector3d e2 = e0.cross(e1).normalized();
  frame.col(0) = e0;
  frame.col(1) = e1;
  frame.col(2) = e2;
  return true;
}

void orientPrimaryWithTrajectory(const poly_traj::Piece &piece, Eigen::Matrix3d &frame)
{
  Eigen::Vector3d chord = piece.getPos(piece.getDuration()) - piece.getPos(0.0);
  if (chord.norm() > kDirectionEpsilon && frame.col(0).dot(chord) < 0.0)
  {
    frame.col(0) *= -1.0;
    frame.col(2) *= -1.0;
  }
}

double velocityAlignmentCosine(const poly_traj::Piece &piece,
                               const Eigen::Matrix3d &frame)
{
  Eigen::Vector3d velocity = piece.getVel(0.5 * piece.getDuration());
  if (velocity.norm() < kDirectionEpsilon)
  {
    velocity = piece.getPos(piece.getDuration()) - piece.getPos(0.0);
  }
  return velocity.norm() < kDirectionEpsilon
             ? 0.0
             : std::abs(frame.col(0).dot(velocity.normalized()));
}

Eigen::Matrix3d mincoDifferentialStateGramian(
    const poly_traj::Piece &piece)
{
  constexpr int kSamples = 8;
  constexpr double kVelocityWeight = 0.55;
  constexpr double kAccelerationWeight = 0.30;
  constexpr double kJerkWeight = 0.15;
  const double duration = std::max(piece.getDuration(), 1.0e-3);
  double length_scale =
      (piece.getPos(duration) - piece.getPos(0.0)).norm();
  for (int sample_id = 0; sample_id < kSamples; ++sample_id)
  {
    const double time = duration *
                        (static_cast<double>(sample_id) + 0.5) /
                        static_cast<double>(kSamples);
    length_scale += duration * piece.getVel(time).norm() /
                    static_cast<double>(kSamples);
  }
  length_scale = std::max(0.5 * length_scale, 1.0e-3);

  Eigen::Matrix3d gramian = 1.0e-6 * Eigen::Matrix3d::Identity();
  for (int sample_id = 0; sample_id < kSamples; ++sample_id)
  {
    const double time = duration *
                        (static_cast<double>(sample_id) + 0.5) /
                        static_cast<double>(kSamples);
    const Eigen::Vector3d velocity =
        duration * piece.getVel(time) / length_scale;
    const Eigen::Vector3d acceleration =
        duration * duration * piece.getAcc(time) / length_scale;
    const Eigen::Vector3d jerk =
        duration * duration * duration * piece.getJer(time) / length_scale;
    gramian.noalias() +=
        (kVelocityWeight * velocity * velocity.transpose() +
         kAccelerationWeight * acceleration * acceleration.transpose() +
         kJerkWeight * jerk * jerk.transpose()) /
        static_cast<double>(kSamples);
  }
  return gramian;
}

bool eigendirectionsDescending(const Eigen::Matrix3d &matrix,
                               Eigen::Matrix3d &frame,
                               Eigen::Vector3d &values)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(0.5 * (matrix + matrix.transpose()));
  if (solver.info() != Eigen::Success)
  {
    return false;
  }

  for (int i = 0; i < 3; ++i)
  {
    const int source = 2 - i;
    frame.col(i) = solver.eigenvectors().col(source).normalized();
    values(i) = std::max(solver.eigenvalues()(source), kDirectionEpsilon);
  }
  if (frame.determinant() < 0.0)
  {
    frame.col(2) *= -1.0;
  }
  return true;
}

} // namespace

bool FrenetDirectionProvider::computeDirections(const poly_traj::Piece &piece,
                                                const PointVector &samples,
                                                const int piece_id,
                                                DirectionSet &directions) const
{
  (void)samples;
  (void)piece_id;
  const double mid_time = 0.5 * piece.getDuration();
  Eigen::Vector3d tangent = piece.getVel(mid_time);
  if (tangent.norm() < kDirectionEpsilon)
  {
    tangent = piece.getPos(piece.getDuration()) - piece.getPos(0.0);
  }

  if (!makeRightHandedFrame(tangent, piece.getAcc(mid_time), directions.frame))
  {
    return false;
  }
  directions.utility = Eigen::Vector3d(1.0, 0.5, 0.5);
  directions.metric_eigenvalues = directions.utility;
  directions.metric_source = "frenet_velocity";
  directions.velocity_alignment_cosine =
      velocityAlignmentCosine(piece, directions.frame);
  directions.used_mode = DirectionMode::FRENET;
  return true;
}

bool PcaDirectionProvider::computeDirections(const poly_traj::Piece &piece,
                                             const PointVector &samples,
                                             const int piece_id,
                                             DirectionSet &directions) const
{
  (void)piece_id;
  if (samples.size() < 2)
  {
    return false;
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d &sample : samples)
  {
    mean += sample;
  }
  mean /= static_cast<double>(samples.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const Eigen::Vector3d &sample : samples)
  {
    const Eigen::Vector3d centered = sample - mean;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(samples.size() - 1);

  if (!eigendirectionsDescending(covariance, directions.frame, directions.utility))
  {
    return false;
  }
  orientPrimaryWithTrajectory(piece, directions.frame);
  directions.metric_eigenvalues = directions.utility;
  directions.utility /= directions.utility.maxCoeff();
  directions.metric_source = "trajectory_pca";
  directions.velocity_alignment_cosine =
      velocityAlignmentCosine(piece, directions.frame);
  directions.used_mode = DirectionMode::PCA;
  return true;
}

void SensitivityDirectionProvider::setPieceGramians(
    const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> &gramians)
{
  gramians_ = gramians;
}

bool SensitivityDirectionProvider::computeDirections(const poly_traj::Piece &piece,
                                                     const PointVector &samples,
                                                     const int piece_id,
                                                     DirectionSet &directions) const
{
  (void)samples;
  Eigen::Matrix3d gramian;
  const bool external_gramian =
      piece_id >= 0 && piece_id < static_cast<int>(gramians_.size()) &&
      gramians_[piece_id].allFinite();
  if (external_gramian)
  {
    gramian = gramians_[piece_id];
  }
  else
  {
    gramian = mincoDifferentialStateGramian(piece);
  }
  if (!gramian.allFinite() ||
      !eigendirectionsDescending(gramian, directions.frame,
                                 directions.utility))
  {
    return false;
  }
  orientPrimaryWithTrajectory(piece, directions.frame);
  directions.metric_eigenvalues = directions.utility;
  directions.utility /= directions.utility.maxCoeff();
  directions.metric_source = external_gramian
                                 ? "external_sensitivity_gramian"
                                 : "minco_differential_state_gramian";
  directions.velocity_alignment_cosine =
      velocityAlignmentCosine(piece, directions.frame);
  directions.used_mode = DirectionMode::SENSITIVITY;
  return true;
}

void FullObjectiveComplianceDirectionProvider::setPieceCompliances(
    const std::vector<Eigen::Matrix3d,
                      Eigen::aligned_allocator<Eigen::Matrix3d>> &compliances)
{
  compliances_ = compliances;
}

bool FullObjectiveComplianceDirectionProvider::computeDirections(
    const poly_traj::Piece &piece,
    const PointVector &samples,
    const int piece_id,
    DirectionSet &directions) const
{
  (void)samples;
  if (piece_id < 0 || piece_id >= static_cast<int>(compliances_.size()) ||
      !compliances_[piece_id].allFinite() ||
      !eigendirectionsDescending(compliances_[piece_id], directions.frame,
                                 directions.utility))
  {
    return false;
  }
  orientPrimaryWithTrajectory(piece, directions.frame);
  directions.metric_eigenvalues = directions.utility;
  directions.utility /= directions.utility.maxCoeff();
  directions.metric_source = "minco_full_objective_compliance";
  directions.velocity_alignment_cosine =
      velocityAlignmentCosine(piece, directions.frame);
  directions.used_mode = DirectionMode::FULL_OBJECTIVE_COMPLIANCE;
  return true;
}

std::unique_ptr<DirectionProvider> makeDirectionProvider(DirectionMode mode)
{
  if (mode == DirectionMode::FRENET)
  {
    return std::unique_ptr<DirectionProvider>(new FrenetDirectionProvider());
  }
  if (mode == DirectionMode::SENSITIVITY)
  {
    return std::unique_ptr<DirectionProvider>(new SensitivityDirectionProvider());
  }
  if (mode == DirectionMode::FULL_OBJECTIVE_COMPLIANCE)
  {
    return std::unique_ptr<DirectionProvider>(
        new FullObjectiveComplianceDirectionProvider());
  }
  return std::unique_ptr<DirectionProvider>(new PcaDirectionProvider());
}

} // namespace tf_sfc
} // namespace ego_planner
