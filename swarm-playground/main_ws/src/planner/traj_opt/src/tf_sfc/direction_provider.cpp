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
  directions.utility /= directions.utility.maxCoeff();
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
  if (piece_id < 0 || piece_id >= static_cast<int>(gramians_.size()))
  {
    return false;
  }

  if (!gramians_[piece_id].allFinite() ||
      !eigendirectionsDescending(gramians_[piece_id], directions.frame, directions.utility))
  {
    return false;
  }
  orientPrimaryWithTrajectory(piece, directions.frame);
  directions.utility /= directions.utility.maxCoeff();
  directions.used_mode = DirectionMode::SENSITIVITY;
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
  return std::unique_ptr<DirectionProvider>(new PcaDirectionProvider());
}

} // namespace tf_sfc
} // namespace ego_planner
