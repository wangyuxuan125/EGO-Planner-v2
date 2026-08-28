#include "optimizer/tf_sfc/hard_corridor_parameterization.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace ego_planner
{
namespace tf_sfc
{

void HardCorridorParameterization::clear()
{
  junctions_.clear();
  spatial_variable_count_ = 0;
  constrained_junction_count_ = 0;
}

bool HardCorridorParameterization::normalizedHPoly(const HPoly &input,
                                                   const double tolerance,
                                                   HPoly &output)
{
  output.resize(input.rows(), 4);
  int write_row = 0;
  for (int row = 0; row < input.rows(); ++row)
  {
    const double normal_norm = input.row(row).head<3>().norm();
    if (!std::isfinite(normal_norm) || normal_norm <= tolerance ||
        !input.row(row).allFinite())
    {
      continue;
    }
    output.row(write_row) = input.row(row) / normal_norm;
    ++write_row;
  }
  output.conservativeResize(write_row, Eigen::NoChange);
  return write_row >= 4;
}

bool HardCorridorParameterization::enumerateVertices(
    const HPoly &hpoly,
    const int max_vertices,
    const double tolerance,
    Eigen::MatrixXd &vertices)
{
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> found;
  const double determinant_tolerance = std::max(1.0e-10, tolerance * 0.1);
  const double inside_tolerance = std::max(1.0e-8, tolerance * 10.0);
  const double duplicate_tolerance = std::max(1.0e-7, tolerance * 20.0);

  for (int i = 0; i < hpoly.rows(); ++i)
  {
    for (int j = i + 1; j < hpoly.rows(); ++j)
    {
      for (int k = j + 1; k < hpoly.rows(); ++k)
      {
        Eigen::Matrix3d normals;
        normals.row(0) = hpoly.row(i).head<3>();
        normals.row(1) = hpoly.row(j).head<3>();
        normals.row(2) = hpoly.row(k).head<3>();
        if (std::abs(normals.determinant()) <= determinant_tolerance)
        {
          continue;
        }
        const Eigen::Vector3d bounds(hpoly(i, 3), hpoly(j, 3), hpoly(k, 3));
        const Eigen::Vector3d candidate = normals.fullPivLu().solve(bounds);
        if (!candidate.allFinite())
        {
          continue;
        }
        bool inside = true;
        for (int face = 0; face < hpoly.rows(); ++face)
        {
          if (hpoly.row(face).head<3>().dot(candidate) >
              hpoly(face, 3) + inside_tolerance)
          {
            inside = false;
            break;
          }
        }
        if (!inside)
        {
          continue;
        }
        bool duplicate = false;
        for (const Eigen::Vector3d &existing : found)
        {
          if ((candidate - existing).norm() <= duplicate_tolerance)
          {
            duplicate = true;
            break;
          }
        }
        if (!duplicate)
        {
          found.push_back(candidate);
        }
      }
    }
  }

  if (found.size() < 4)
  {
    return false;
  }

  // A subset of vertices still forms a safe convex subset. Keep well-spread
  // vertices when an unusually complex corridor exceeds the variable budget.
  if (max_vertices >= 4 && static_cast<int>(found.size()) > max_vertices)
  {
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> selected;
    selected.reserve(max_vertices);
    selected.push_back(found.front());
    std::vector<double> nearest(found.size(),
                                std::numeric_limits<double>::infinity());
    std::vector<bool> used(found.size(), false);
    used.front() = true;
    while (static_cast<int>(selected.size()) < max_vertices)
    {
      int best_id = -1;
      double best_distance = -1.0;
      const Eigen::Vector3d &last = selected.back();
      for (int id = 0; id < static_cast<int>(found.size()); ++id)
      {
        if (used[id])
        {
          continue;
        }
        nearest[id] = std::min(nearest[id], (found[id] - last).squaredNorm());
        if (nearest[id] > best_distance)
        {
          best_distance = nearest[id];
          best_id = id;
        }
      }
      if (best_id < 0)
      {
        break;
      }
      used[best_id] = true;
      selected.push_back(found[best_id]);
    }
    found.swap(selected);
  }

  vertices.resize(3, found.size());
  for (int id = 0; id < static_cast<int>(found.size()); ++id)
  {
    vertices.col(id) = found[id];
  }
  return true;
}

bool HardCorridorParameterization::configure(const CorridorVector &corridors,
                                             const int piece_count,
                                             const int max_vertices,
                                             const double vertex_tolerance,
                                             const Eigen::MatrixXd *preferred_junctions)
{
  clear();
  vertex_tolerance_ = std::max(1.0e-10, vertex_tolerance);
  if (piece_count <= 1)
  {
    return piece_count == 1;
  }
  if (preferred_junctions &&
      (preferred_junctions->rows() != 3 ||
       preferred_junctions->cols() != piece_count - 1 ||
       !preferred_junctions->allFinite()))
  {
    return false;
  }
  junctions_.resize(piece_count - 1);
  for (int junction_id = 0; junction_id < piece_count - 1; ++junction_id)
  {
    Junction &junction = junctions_[junction_id];
    junction.variable_offset = spatial_variable_count_;

    int face_count = 0;
    for (int piece_id = junction_id; piece_id <= junction_id + 1; ++piece_id)
    {
      if (piece_id < static_cast<int>(corridors.size()) &&
          corridors[piece_id].metrics.valid)
      {
        face_count += corridors[piece_id].hpoly.rows();
      }
    }
    if (face_count == 0)
    {
      spatial_variable_count_ += 3;
      continue;
    }

    HPoly combined(face_count, 4);
    int row = 0;
    for (int piece_id = junction_id; piece_id <= junction_id + 1; ++piece_id)
    {
      if (piece_id < static_cast<int>(corridors.size()) &&
          corridors[piece_id].metrics.valid)
      {
        const HPoly &piece = corridors[piece_id].hpoly;
        combined.middleRows(row, piece.rows()) = piece;
        row += piece.rows();
      }
    }
    const int enumeration_budget =
        preferred_junctions && max_vertices > 4 ? max_vertices - 1
                                                : max_vertices;
    if (!normalizedHPoly(combined, vertex_tolerance_, junction.hpoly) ||
        !enumerateVertices(junction.hpoly, enumeration_budget,
                           vertex_tolerance_,
                           junction.vertices))
    {
      clear();
      return false;
    }
    if (preferred_junctions)
    {
      const Eigen::Vector3d preferred =
          preferred_junctions->col(junction_id);
      bool inside = true;
      for (int face = 0; face < junction.hpoly.rows(); ++face)
      {
        if (junction.hpoly.row(face).head<3>().dot(preferred) >
            junction.hpoly(face, 3) + 10.0 * vertex_tolerance_)
        {
          inside = false;
          break;
        }
      }
      if (inside)
      {
        int nearest_id = 0;
        double nearest_squared =
            (junction.vertices.col(0) - preferred).squaredNorm();
        for (int vertex_id = 1; vertex_id < junction.vertices.cols();
             ++vertex_id)
        {
          const double squared =
              (junction.vertices.col(vertex_id) - preferred).squaredNorm();
          if (squared < nearest_squared)
          {
            nearest_squared = squared;
            nearest_id = vertex_id;
          }
        }
        const double exact_tolerance =
            std::max(1.0e-12, vertex_tolerance_ * vertex_tolerance_);
        if (nearest_squared <= exact_tolerance)
        {
          junction.vertices.col(nearest_id) = preferred;
        }
        else if (max_vertices < 4 ||
                 junction.vertices.cols() < max_vertices)
        {
          const int old_count = junction.vertices.cols();
          junction.vertices.conservativeResize(Eigen::NoChange,
                                               old_count + 1);
          junction.vertices.col(old_count) = preferred;
        }
        else
        {
          // The preferred point is itself inside the intersection. Replacing
          // the nearest generator keeps the convex-hull parameterization safe
          // while honoring the configured variable budget.
          junction.vertices.col(nearest_id) = preferred;
        }
      }
    }
    junction.constrained = true;
    junction.variable_count = junction.vertices.cols();
    spatial_variable_count_ += junction.variable_count;
    ++constrained_junction_count_;
  }
  return true;
}

Eigen::VectorXd HardCorridorParameterization::projectSimplex(
    const Eigen::VectorXd &value)
{
  std::vector<double> sorted(value.data(), value.data() + value.size());
  std::sort(sorted.begin(), sorted.end(), std::greater<double>());
  double cumulative = 0.0;
  int rho = 0;
  for (int i = 0; i < static_cast<int>(sorted.size()); ++i)
  {
    cumulative += sorted[i];
    const double threshold = (cumulative - 1.0) / static_cast<double>(i + 1);
    if (sorted[i] > threshold)
    {
      rho = i + 1;
    }
  }
  cumulative = std::accumulate(sorted.begin(), sorted.begin() + rho, 0.0);
  const double threshold = (cumulative - 1.0) / static_cast<double>(rho);
  return (value.array() - threshold).max(0.0).matrix();
}

Eigen::VectorXd HardCorridorParameterization::inverseWeights(
    const Eigen::MatrixXd &vertices,
    const Eigen::Vector3d &point)
{
  for (int vertex_id = 0; vertex_id < vertices.cols(); ++vertex_id)
  {
    if ((vertices.col(vertex_id) - point).squaredNorm() <= 1.0e-20)
    {
      Eigen::VectorXd exact = Eigen::VectorXd::Zero(vertices.cols());
      exact(vertex_id) = 1.0;
      return exact;
    }
  }
  Eigen::VectorXd weights = Eigen::VectorXd::Constant(
      vertices.cols(), 1.0 / static_cast<double>(vertices.cols()));
  const Eigen::MatrixXd offsets = vertices.colwise() - point;
  const double step = 1.0 / std::max(1.0e-9, offsets.squaredNorm());
  for (int iteration = 0; iteration < 1024; ++iteration)
  {
    const Eigen::VectorXd next = projectSimplex(
        weights - step * offsets.transpose() * (offsets * weights));
    if ((next - weights).squaredNorm() <= 1.0e-24 ||
        (offsets * next).squaredNorm() <= 1.0e-20)
    {
      weights = next;
      break;
    }
    weights = next;
  }
  return weights;
}

bool HardCorridorParameterization::encode(const Eigen::MatrixXd &points,
                                          Eigen::VectorXd &variables) const
{
  if (points.rows() != 3 || points.cols() != totalJunctionCount())
  {
    return false;
  }
  variables.resize(spatial_variable_count_);
  for (int junction_id = 0; junction_id < totalJunctionCount(); ++junction_id)
  {
    const Junction &junction = junctions_[junction_id];
    if (!junction.constrained)
    {
      variables.segment<3>(junction.variable_offset) = points.col(junction_id);
      continue;
    }
    const Eigen::VectorXd weights = inverseWeights(junction.vertices,
                                                    points.col(junction_id));
    variables.segment(junction.variable_offset, junction.variable_count) =
        weights.cwiseMax(0.0).cwiseSqrt();
  }
  return variables.allFinite();
}

bool HardCorridorParameterization::decode(
    const Eigen::Ref<const Eigen::VectorXd> &variables,
    Eigen::MatrixXd &points) const
{
  if (variables.size() != spatial_variable_count_)
  {
    return false;
  }
  points.resize(3, totalJunctionCount());
  for (int junction_id = 0; junction_id < totalJunctionCount(); ++junction_id)
  {
    const Junction &junction = junctions_[junction_id];
    if (!junction.constrained)
    {
      points.col(junction_id) = variables.segment<3>(junction.variable_offset);
      continue;
    }
    const Eigen::VectorXd raw = variables.segment(junction.variable_offset,
                                                   junction.variable_count);
    const double norm = raw.norm();
    if (!std::isfinite(norm) || norm <= 1.0e-12)
    {
      return false;
    }
    const Eigen::VectorXd unit = raw / norm;
    points.col(junction_id) = junction.vertices * unit.array().square().matrix();
  }
  return points.allFinite();
}

bool HardCorridorParameterization::backwardGradient(
    const Eigen::Ref<const Eigen::VectorXd> &variables,
    const Eigen::MatrixXd &point_gradient,
    Eigen::Ref<Eigen::VectorXd> variable_gradient) const
{
  if (variables.size() != spatial_variable_count_ ||
      variable_gradient.size() != spatial_variable_count_ ||
      point_gradient.rows() != 3 ||
      point_gradient.cols() != totalJunctionCount())
  {
    return false;
  }
  variable_gradient.setZero();
  for (int junction_id = 0; junction_id < totalJunctionCount(); ++junction_id)
  {
    const Junction &junction = junctions_[junction_id];
    if (!junction.constrained)
    {
      variable_gradient.segment<3>(junction.variable_offset) =
          point_gradient.col(junction_id);
      continue;
    }
    const Eigen::VectorXd raw = variables.segment(junction.variable_offset,
                                                   junction.variable_count);
    const double norm = raw.norm();
    if (!std::isfinite(norm) || norm <= 1.0e-12)
    {
      return false;
    }
    const Eigen::VectorXd unit = raw / norm;
    const Eigen::VectorXd unit_gradient =
        (2.0 * unit.array() *
         (junction.vertices.transpose() *
          point_gradient.col(junction_id)).array()).matrix();
    variable_gradient.segment(junction.variable_offset, junction.variable_count) =
        (unit_gradient - unit * unit.dot(unit_gradient)) / norm;
  }
  return variable_gradient.allFinite();
}

double HardCorridorParameterization::addNormRestriction(
    const Eigen::Ref<const Eigen::VectorXd> &variables,
    Eigen::Ref<Eigen::VectorXd> variable_gradient) const
{
  double cost = 0.0;
  for (const Junction &junction : junctions_)
  {
    if (!junction.constrained)
    {
      continue;
    }
    const Eigen::VectorXd raw = variables.segment(junction.variable_offset,
                                                   junction.variable_count);
    const double violation = raw.squaredNorm() - 1.0;
    if (violation > 0.0)
    {
      cost += violation * violation * violation;
      variable_gradient.segment(junction.variable_offset,
                                junction.variable_count) +=
          6.0 * violation * violation * raw;
    }
  }
  return cost;
}

double HardCorridorParameterization::maxJunctionViolation(
    const Eigen::MatrixXd &points) const
{
  if (points.rows() != 3 || points.cols() != totalJunctionCount())
  {
    return std::numeric_limits<double>::infinity();
  }
  double maximum = 0.0;
  for (int junction_id = 0; junction_id < totalJunctionCount(); ++junction_id)
  {
    const Junction &junction = junctions_[junction_id];
    if (!junction.constrained)
    {
      continue;
    }
    for (int face = 0; face < junction.hpoly.rows(); ++face)
    {
      maximum = std::max(maximum,
                         junction.hpoly.row(face).head<3>().dot(
                             points.col(junction_id)) - junction.hpoly(face, 3));
    }
  }
  return std::max(0.0, maximum);
}

} // namespace tf_sfc
} // namespace ego_planner
