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
  DECOMP_FAILURE = 9,
  INSUFFICIENT_PIECES = 10,
  SEED_START_INVALID = 11,
  SEED_ASTAR_FAILURE = 12,
  SEED_PATH_OUTSIDE_MAP = 13,
  SEED_PATH_OCCUPIED = 14,
  PIECE_BUDGET_TAIL = 15,
  HARD_PARAMETERIZATION_FAILURE = 16,
  FACE_BUDGET_EXHAUSTED = 17,
  OBSTACLE_SEPARATION_FAILURE = 18,
  SEED_CLEARANCE_FAILURE = 19
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
  case FailureReason::INSUFFICIENT_PIECES: return "insufficient_pieces";
  case FailureReason::SEED_START_INVALID: return "seed_start_invalid";
  case FailureReason::SEED_ASTAR_FAILURE: return "seed_astar_failure";
  case FailureReason::SEED_PATH_OUTSIDE_MAP: return "seed_path_outside_map";
  case FailureReason::SEED_PATH_OCCUPIED: return "seed_path_occupied";
  case FailureReason::PIECE_BUDGET_TAIL: return "piece_budget_tail";
  case FailureReason::HARD_PARAMETERIZATION_FAILURE: return "hard_parameterization_failure";
  case FailureReason::FACE_BUDGET_EXHAUSTED: return "face_budget_exhausted";
  case FailureReason::OBSTACLE_SEPARATION_FAILURE: return "obstacle_separation_failure";
  case FailureReason::SEED_CLEARANCE_FAILURE: return "seed_clearance_failure";
  }
  return "unknown";
}

struct Parameters
{
  bool enabled = false;
  bool use_projection = true;
  bool use_soft_penalty = false;
  bool allow_partial_corridors = true;
  bool allow_ego_fallback = true;
  // When false, the requested direction provider must succeed. Formal
  // sensitivity experiments use this gate to prevent silent PCA/Frenet reuse.
  bool allow_direction_fallback = true;
  bool enforce_final_corridor = true;
  bool hard_corridor_parameterization = true;
  bool decomp_retry_seed_validation_without_velocity = true;
  // Shared by TF-SFC and EllipsoidDecomp: if a velocity-prefixed seed fails
  // clearance/corner certification, rebuild it once with ordinary A*.
  bool seed_retry_without_velocity_on_clearance_failure = true;
  bool visualization_enabled = true;
  bool log_enabled = true;
  std::string visualization_frame = "world";
  std::string log_directory = "/tmp/tf_sfc_results/ego";
  std::string experiment_tag = "default";
  // obb: six-face directional OBB baseline; tf_sfc: proposed trajectory-favorable,
  // face-bounded inflation; ellipsoid_decomp: Liu et al. / DecompUtil baseline.
  std::string corridor_method = "obb";
  DirectionMode direction_mode = DirectionMode::PCA;
  int max_faces = 12;
  int max_obs_faces = 6;
  int samples_per_piece = 8;
  int projection_passes = 4;
  int min_valid_pieces = 1;
  int max_enforcement_passes = 2;
  int hard_max_vertices = 64;
  double safety_margin = 0.25;
  // Required slack for non-junction trajectory samples against obstacle cuts.
  // Junction samples use min_overlap_radius instead.
  double interior_sample_margin = 0.0;
  // Hard full-dimensional overlap acceptance floor. Larger overlap is a
  // reported quality objective, not a mandatory tube radius.
  double min_overlap_radius = 0.02;
  // Radius and grid step for relocating simplified A* bends while preserving
  // collision-free visibility to both neighboring seed segments.
  double junction_refine_radius = 0.50;
  double junction_refine_step = 0.05;
  double max_inflation_distance = 1.0;
  double inflation_step = 0.10;
  double weight = 1000.0;
  double enforcement_weight_multiplier = 3.0;
  double enforcement_min_improvement = 1.0e-5;
  double max_final_violation = 1.0e-3;
  double hard_vertex_tolerance = 1.0e-7;
  double penalty_epsilon = 0.02;
  double decomp_local_bbox_forward = 0.5;
  double decomp_local_bbox_lateral = 1.0;
  double decomp_local_bbox_vertical = 1.0;
  double decomp_overlap_extension = 0.20;
  double decomp_initial_velocity_segment = 0.40;
  double decomp_initial_velocity_threshold = 0.20;
  double decomp_degenerate_seed_length = 0.10;
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
  int obstacle_face_count = 0;
  int obstacle_point_count = 0;
  int inflation_candidate_evaluation_count = 0;
  bool face_budget_saturated = false;
  double generation_time_ms = 0.0;
  double weighted_width = 0.0;
  double min_sample_slack = -std::numeric_limits<double>::infinity();
  double anchor_clearance_radius = -std::numeric_limits<double>::infinity();
  double min_obstacle_sample_distance_m =
      std::numeric_limits<double>::quiet_NaN();
  int separation_failure_sample_id = -1;
  bool separation_failure_at_endpoint = false;
  double overlap_radius_to_next = -1.0;
  bool seed_containment_evaluated = false;
  bool seed_contained = false;
  double seed_containment_max_violation_m =
      std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool direction_fallback = false;
  int requested_direction_mode = static_cast<int>(DirectionMode::PCA);
  // -1 means no direction was produced (e.g. an upstream or provider failure).
  int used_direction_mode = -1;
  FailureReason failure_reason = FailureReason::NONE;
};

struct CorridorEvaluation
{
  int constrained_piece_count = 0;
  double penalty_cost = 0.0;
  double max_violation_m = 0.0;
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
