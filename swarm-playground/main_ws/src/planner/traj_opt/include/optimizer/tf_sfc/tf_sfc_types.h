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
  SENSITIVITY = 2,
  FULL_OBJECTIVE_COMPLIANCE = 3
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
  // Shared by TF-SFC and EllipsoidDecomp. On a certified-seed clearance
  // failure, retry once without the velocity prefix. When enabled, the retry
  // rejects clearance-invalid A* edges globally instead of relying on a local
  // post-search detour.
  bool seed_retry_without_velocity_on_clearance_failure = true;
  bool seed_clearance_astar_enabled = true;
  // Proposed-method-only bounded repair.  The trajectory samples are tried
  // first.  If they cannot seed an obstacle-free convex region, a certified
  // A* seed segment may be used while retaining the MINCO-derived directions.
  bool trajectory_repair_enabled = true;
  bool trajectory_repair_seed_fallback_enabled = true;
  // Experimental v22 gate. Disabled by default because accepting a seed
  // corridor without trajectory improvement caused over-acceptance and
  // degraded closed-loop progress in the first v22 run.
  bool trajectory_repair_seed_geometric_acceptance_enabled = false;
  // One outer repair/re-optimization is allowed only after ordinary strict
  // corridor continuation would otherwise reject a collision-free candidate.
  bool post_optimization_repair_enabled = false;
  // Reject a collision-free optimized trajectory when it initially moves
  // opposite the certified seed tangent, makes a large target-axis U-turn,
  // or overshoots the local target before returning.
  bool progress_guard_enabled = true;
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
  int trajectory_repair_max_passes = 1;
  int post_optimization_repair_max_passes = 1;
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
  double seed_clearance_astar_time_limit = 0.20;
  double trajectory_repair_trigger = 1.0e-3;
  double trajectory_repair_min_improvement = 1.0e-4;
  // Region quality uses trajectory-weighted squared width [m^2] minus this
  // cost for every retained half-space face. This makes the FIRI face-count
  // trade-off explicit while max_faces remains a hard upper bound.
  double face_quality_weight = 0.20;
  bool exact_face_subset_pruning_enabled = true;
  double progress_guard_horizon_s = 1.0;
  double max_initial_progress_regression = 0.05;
  double max_target_axis_progress_drop = 0.25;
  double max_target_overshoot = 0.15;
  // Mode 3 estimates the fixed-duration spatial Hessian of the complete
  // pre-corridor EGO/MINCO objective by central differences of its analytic
  // waypoint gradient. Its absolute-curvature inverse is projected to one
  // scale-free 3x3 workspace compliance matrix per piece, then conditioned
  // by the actual MINCO transport velocity at high speed.
  double objective_compliance_fd_step = 0.02;
  double objective_compliance_eigenvalue_floor_ratio = 1.0e-4;
  double objective_compliance_absolute_floor = 1.0e-6;
  double objective_compliance_transport_speed_reference = 1.0;
  double objective_compliance_transport_weight_max = 2.0;
  // Retained for launch-file compatibility with v22. v23 restores strict
  // re-encoding and never projects a post-optimization junction by this amount.
  double post_optimization_repair_max_seed_junction_shift = 1.0e-5;
};

struct DirectionSet
{
  Eigen::Matrix3d frame = Eigen::Matrix3d::Identity();
  Eigen::Vector3d utility = Eigen::Vector3d::Ones();
  std::string metric_source = "not_evaluated";
  Eigen::Vector3d metric_eigenvalues = Eigen::Vector3d::Ones();
  double velocity_alignment_cosine = 0.0;
  double transport_conditioning_weight = 0.0;
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
  int obstacle_face_count_before_pruning = 0;
  int obstacle_face_prune_count = 0;
  int face_subset_evaluation_count = 0;
  int obstacle_point_count = 0;
  int inflation_candidate_evaluation_count = 0;
  bool face_budget_saturated = false;
  double generation_time_ms = 0.0;
  double weighted_width = 0.0;
  double region_quality_score = 0.0;
  double face_quality_penalty = 0.0;
  std::string direction_metric_source = "not_evaluated";
  double direction_metric_eigenvalue_max = 0.0;
  double direction_metric_eigenvalue_mid = 0.0;
  double direction_metric_eigenvalue_min = 0.0;
  double direction_metric_condition_number = 0.0;
  double direction_velocity_alignment_cosine = 0.0;
  double direction_transport_conditioning_weight = 0.0;
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
  int evaluated_face_sample_count = 0;
  int violating_face_sample_count = 0;
  int max_violation_piece_id = -1;
  int max_violation_face_id = -1;
  int max_violation_sample_id = -1;
  double max_violation_time_ratio = 0.0;
  double penalty_cost = 0.0;
  double max_violation_m = 0.0;
};

struct TrajectoryRepairResult
{
  bool evaluated = false;
  bool attempted = false;
  bool accepted = false;
  int piece_id = -1;
  int candidate_face_count = 0;
  double generation_time_ms = 0.0;
  double global_violation_before_m = 0.0;
  double global_violation_after_m = 0.0;
  double piece_violation_before_m = 0.0;
  double piece_violation_after_m = 0.0;
  double overlap_previous_m = -1.0;
  double overlap_next_m = -1.0;
  bool seed_fallback_used = false;
  bool geometric_acceptance_used = false;
  std::string sample_source = "trajectory_samples";
  std::string overlap_anchor_source = "trajectory_junction";
  std::string primary_failure_reason = "none";
  std::string reason = "not_evaluated";
  double candidate_weighted_width_m = 0.0;
  double junction_reencode_shift_m = 0.0;
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
