#pragma once

#include "optimizer/tf_sfc/tf_sfc_types.h"

#include <Eigen/Eigen>

#include <vector>

namespace ego_planner
{
namespace tf_sfc
{

// GCOPTER-style unconstrained parameterization of MINCO junctions. Each
// constrained junction is represented as a convex combination of vertices of
// the adjacent-corridor intersection, so decoding can never leave that set.
class HardCorridorParameterization
{
public:
  bool configure(const CorridorVector &corridors,
                 int piece_count,
                 int max_vertices,
                 double vertex_tolerance,
                 const Eigen::MatrixXd *preferred_junctions = nullptr);

  void clear();

  int spatialVariableCount() const { return spatial_variable_count_; }
  int constrainedJunctionCount() const { return constrained_junction_count_; }
  int totalJunctionCount() const { return static_cast<int>(junctions_.size()); }

  bool encode(const Eigen::MatrixXd &points, Eigen::VectorXd &variables) const;
  bool decode(const Eigen::Ref<const Eigen::VectorXd> &variables,
              Eigen::MatrixXd &points) const;
  bool backwardGradient(const Eigen::Ref<const Eigen::VectorXd> &variables,
                        const Eigen::MatrixXd &point_gradient,
                        Eigen::Ref<Eigen::VectorXd> variable_gradient) const;
  double addNormRestriction(const Eigen::Ref<const Eigen::VectorXd> &variables,
                            Eigen::Ref<Eigen::VectorXd> variable_gradient) const;

  double maxJunctionViolation(const Eigen::MatrixXd &points) const;

private:
  struct Junction
  {
    bool constrained = false;
    int variable_offset = 0;
    int variable_count = 3;
    HPoly hpoly;
    Eigen::MatrixXd vertices;
  };

  static bool normalizedHPoly(const HPoly &input,
                              double tolerance,
                              HPoly &output);
  static bool enumerateVertices(const HPoly &hpoly,
                                int max_vertices,
                                double tolerance,
                                Eigen::MatrixXd &vertices);
  static Eigen::VectorXd projectSimplex(const Eigen::VectorXd &value);
  static Eigen::VectorXd inverseWeights(const Eigen::MatrixXd &vertices,
                                        const Eigen::Vector3d &point);

  std::vector<Junction> junctions_;
  int spatial_variable_count_ = 0;
  int constrained_junction_count_ = 0;
  double vertex_tolerance_ = 1.0e-7;
};

} // namespace tf_sfc
} // namespace ego_planner
