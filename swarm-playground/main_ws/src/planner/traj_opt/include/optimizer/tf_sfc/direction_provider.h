#pragma once

#include "optimizer/poly_traj_utils.hpp"
#include "optimizer/tf_sfc/tf_sfc_types.h"

#include <memory>

namespace ego_planner
{
namespace tf_sfc
{

class DirectionProvider
{
public:
  virtual ~DirectionProvider() {}

  virtual DirectionMode mode() const = 0;
  virtual bool computeDirections(const poly_traj::Piece &piece,
                                 const PointVector &samples,
                                 const int piece_id,
                                 DirectionSet &directions) const = 0;
};

class FrenetDirectionProvider final : public DirectionProvider
{
public:
  DirectionMode mode() const override { return DirectionMode::FRENET; }
  bool computeDirections(const poly_traj::Piece &piece,
                         const PointVector &samples,
                         const int piece_id,
                         DirectionSet &directions) const override;
};

class PcaDirectionProvider final : public DirectionProvider
{
public:
  DirectionMode mode() const override { return DirectionMode::PCA; }
  bool computeDirections(const poly_traj::Piece &piece,
                         const PointVector &samples,
                         const int piece_id,
                         DirectionSet &directions) const override;
};

class SensitivityDirectionProvider final : public DirectionProvider
{
public:
  DirectionMode mode() const override { return DirectionMode::SENSITIVITY; }

  void setPieceGramians(const std::vector<Eigen::Matrix3d,
                                          Eigen::aligned_allocator<Eigen::Matrix3d>> &gramians);

  bool computeDirections(const poly_traj::Piece &piece,
                         const PointVector &samples,
                         const int piece_id,
                         DirectionSet &directions) const override;

private:
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> gramians_;
};

std::unique_ptr<DirectionProvider> makeDirectionProvider(DirectionMode mode);

} // namespace tf_sfc
} // namespace ego_planner
