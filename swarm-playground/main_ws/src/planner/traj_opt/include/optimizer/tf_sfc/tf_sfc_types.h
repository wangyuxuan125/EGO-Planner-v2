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

// Converts n.dot(x) <= b into the point/outer-normal plane representation
// expected by decomp_ros_msgs/Polyhedron.
inline bool hpolyFaceToPointNormal(const HPoly &hpoly,
                                   const int face_id,
                                   Eigen::Vector3d &point,
                                   Eigen::Vector3d &normal)
{
  if (face_id < 0 || face_id >= hpoly.rows())
  {
    return false;
  }
  normal = hpoly.row(face_id).head<3>().transpose();
  const double squared_norm = normal.squaredNorm();
  if (squared_norm <= 1.0e-12)
  {
    return false;
  }
  point = normal * hpoly(face_id, 3) / squared_norm;
  return true;
}

enum class DirectionMode
{
  FRENET = 0,
  PCA = 1,
  SENSITIVITY = 2
};

enum class FailureReason
{
  NONE = 0,
  INVALID_INPUT = 1,
  DIRECTION_FAILURE = 2,
  OUTSIDE_LOCAL_MAP = 3,
  INITIAL_OBB_OCCUPIED = 4,
  OVERLAP_TOO_SMALL = 5,
  SKIPPED_AFTER_FAILURE = 6,
  SEED_PATH_FAILURE = 7,
  DECOMP_UTIL_UNAVAILABLE = 8,
  DECOMP_FAILURE = 9
};

inline const char *failureReasonName(const FailureReason reason)
{
  switch (reason)
  {
  case FailureReason::NONE: return "none";
  case FailureReason::INVALID_INPUT: return "invalid_input";
  case FailureReason::DIRECTION_FAILURE: return "direction_failure";
  case FailureReason::OUTSIDE_LOCAL_MAP: return "outside_local_map";
  case FailureReason::INITIAL_OBB_OCCUPIED: return "initial_obb_occupied";
  case FailureReason::OVERLAP_TOO_SMALL: return "overlap_too_small";
  case FailureReason::SKIPPED_AFTER_FAILURE: return "skipped_after_failure";
  case FailureReason::SEED_PATH_FAILURE: return "seed_path_failure";
  case FailureReason::DECOMP_UTIL_UNAVAILABLE: return "decomp_util_unavailable";
  case FailureReason::DECOMP_FAILURE: return "decomp_failure";
  }
  return "unknown";
}

struct Parameters
{
  bool enabled = false;
  bool use_projection = false;
  bool use_soft_penalty = false;
  bool allow_partial_corridors = true;
  bool allow_ego_fallback = true;
  bool visualization_enabled = true;
  bool log_enabled = true;
  std::string visualization_frame = "world";
  std::string log_directory = "/tmp/tf_sfc_results/ego";
  std::string experiment_tag = "default";
  // obb: TF-SFC MVP; ellipsoid_decomp: Liu et al. / DecompUtil baseline.
  std::string corridor_method = "obb";
  DirectionMode direction_mode = DirectionMode::PCA;
  int max_faces = 12;
  int samples_per_piece = 8;
  int projection_passes = 4;
  int min_valid_pieces = 1;
  double safety_margin = 0.25;
  double min_overlap_radius = 0.15;
  double max_inflation_distance = 1.0;
  double inflation_step = 0.10;
  double weight = 1000.0;
  double penalty_epsilon = 0.02;
  double decomp_local_bbox_forward = 0.5;
  double decomp_local_bbox_lateral = 1.0;
  double decomp_local_bbox_vertical = 1.0;
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
  FailureReason failure_reason = FailureReason::NONE;
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
