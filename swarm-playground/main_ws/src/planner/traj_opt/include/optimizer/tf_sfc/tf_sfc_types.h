#pragma once

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <limits>
#include <string>
#include <vector>

namespace ego_planner
{
namespace tf_sfc
{

typedef Eigen::Matrix<double, Eigen::Dynamic, 4> HPoly;

enum class DirectionMode
{
  FRENET = 0,
  PCA = 1,
  SENSITIVITY = 2
};

struct Parameters
{
  bool enabled = false;
  bool use_projection = false;
  bool use_soft_penalty = false;
  DirectionMode direction_mode = DirectionMode::PCA;
  int max_faces = 12;
  int samples_per_piece = 8;
  int projection_passes = 4;
  double safety_margin = 0.25;
  double min_overlap_radius = 0.15;
  double max_inflation_distance = 1.0;
  double inflation_step = 0.10;
  double weight = 1000.0;
  double penalty_epsilon = 0.02;
};

struct DirectionSet
{
  Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
  Eigen::Vector3d utility = Eigen::Vector3d::Ones();
  DirectionMode requested_mode = DirectionMode::PCA;
  DirectionMode used_mode = DirectionMode::PCA;
  bool used_fallback = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CorridorMetrics
{
  int piece_id = -1;
  int face_count = 0;
  double generation_time_ms = 0.0;
  double weighted_width = 0.0;
  double min_sample_slack = -std::numeric_limits<double>::infinity();
  double overlap_radius_to_next = -1.0;
  bool valid = false;
  bool direction_fallback = false;
};

// Each row of hpoly stores [n_x, n_y, n_z, b] for n.dot(x) <= b.
struct Corridor
{
  HPoly hpoly;
  Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
  Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
  Eigen::Vector3d utility = Eigen::Vector3d::Ones();
  CorridorMetrics metrics;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::vector<Corridor, Eigen::aligned_allocator<Corridor>> CorridorVector;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> PointVector;

} // namespace tf_sfc
} // namespace ego_planner
