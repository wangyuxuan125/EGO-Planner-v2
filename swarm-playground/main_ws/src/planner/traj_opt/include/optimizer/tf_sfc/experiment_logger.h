#pragma once

#include "optimizer/tf_sfc/tf_sfc_types.h"

#include <cstdint>
#include <mutex>
#include <string>

namespace ego_planner
{
namespace tf_sfc
{

struct ExperimentRunRecord
{
  std::string run_id;
  std::string planning_event_id;
  std::string experiment_tag;
  std::string status;
  std::string requested_method = "obb";
  std::string method = "obb";
  double timestamp_s = 0.0;
  int drone_id = -1;
  std::uint64_t goal_id = 0;
  std::uint64_t replan_id = 0;
  int retry_index = 0;
  int attempt_id = 0;
  bool touch_goal = false;
  bool commanded_goal_valid = false;
  double commanded_goal_x_m = 0.0;
  double commanded_goal_y_m = 0.0;
  double commanded_goal_z_m = 0.0;
  double planning_start_x_m = 0.0;
  double planning_start_y_m = 0.0;
  double planning_start_z_m = 0.0;
  double planning_target_x_m = 0.0;
  double planning_target_y_m = 0.0;
  double planning_target_z_m = 0.0;
  std::string seed_path_strategy = "not_applicable";
  bool initial_velocity_seed_attempted = false;
  bool initial_velocity_seed_used = false;
  bool velocity_seed_fallback_used = false;
  std::string velocity_seed_fallback_reason = "none";
  bool clearance_retry_attempted = false;
  bool clearance_retry_success = false;
  std::string clearance_retry_initial_strategy = "none";
  int clearance_retry_first_failure_point_id = -1;
  bool clearance_astar_attempted = false;
  bool clearance_astar_success = false;
  int clearance_astar_call_count = 0;
  double clearance_astar_ms = 0.0;
  int seed_validation_failure_point_id = -1;
  bool astar_search_attempted = false;
  bool astar_search_success = false;
  int astar_search_call_count = 0;
  double astar_search_ms = 0.0;
  int raw_seed_path_point_count = 0;
  double raw_seed_path_length_m = 0.0;
  int seed_path_point_count = 0;
  double seed_path_length_m = 0.0;
  bool seed_path_edge_valid = false;
  double seed_path_coverage_ratio = 0.0;
  bool allow_partial_corridors = false;
  bool seed_start_in_map = false;
  bool seed_finish_in_map = false;
  bool partial_target_search_attempted = false;
  bool partial_target_found = false;
  double partial_boundary_ratio = 0.0;
  double inflated_map_low_x_m = 0.0;
  double inflated_map_low_y_m = 0.0;
  double inflated_map_low_z_m = 0.0;
  double inflated_map_high_x_m = 0.0;
  double inflated_map_high_y_m = 0.0;
  double inflated_map_high_z_m = 0.0;
  bool seed_frontend_evaluated = false;
  bool seed_frontend_success = false;
  bool corridor_generation_attempted = false;
  bool tf_sfc_enabled = false;
  int direction_mode = 1;
  int used_direction_mode = -1;
  bool direction_fallback_allowed = true;
  bool objective_compliance_attempted = false;
  bool objective_compliance_success = false;
  int objective_compliance_spatial_variable_count = 0;
  int objective_compliance_evaluation_count = 0;
  int objective_compliance_regularized_eigenvalue_count = 0;
  double objective_compliance_ms = 0.0;
  double objective_compliance_raw_min_eigenvalue = 0.0;
  double objective_compliance_raw_max_eigenvalue = 0.0;
  double objective_compliance_regularized_condition_number = 0.0;
  std::string objective_compliance_reason = "not_requested";
  bool success = false;
  bool collision_free = false;
  bool final_obstacle_collision = false;
  bool final_swarm_clearance_failure = false;
  std::string terminal_failure_reason = "none";
  bool tf_sfc_generated = false;
  bool fallback_to_ego = false;
  bool projection_applied = false;
  bool hard_parameterization_enabled = false;
  bool hard_parameterization_active = false;
  int hard_constrained_junction_count = 0;
  int hard_total_junction_count = 0;
  int direct_spatial_variable_count = 0;
  int hard_spatial_variable_count = 0;
  // hard/direct; meaningful when hard_parameterization_active is true.
  double hard_spatial_variable_overhead_ratio = 0.0;
  // Number of valid (face, trajectory-sample) pairs in one corridor
  // constraint/penalty evaluation, excluding optimizer iteration count.
  int face_sample_pairs_per_evaluation = 0;
  int corridor_penalty_samples_per_piece = 0;
  double max_junction_violation_initial_m = 0.0;
  double max_junction_violation_final_m = 0.0;
  int lbfgs_result = 0;
  double total_planning_ms = 0.0;
  double optimizer_ms = 0.0;
  double corridor_generation_ms = 0.0;
  double seed_path_build_ms = 0.0;
  double corridor_inflation_ms = 0.0;
  int lbfgs_iterations = 0;
  int restart_count = 0;
  int rebound_count = 0;
  int piece_count = 0;
  int seed_piece_count = 0;
  int corridor_slot_count = 0;
  bool seed_minco_alignment_valid = false;
  int corridor_count = 0;
  int failed_piece_count = 0;
  std::string first_failure_reason = "none";
  int total_faces = 0;
  double mean_faces = 0.0;
  double mean_weighted_width = 0.0;
  double min_sample_slack = 0.0;
  double min_overlap_radius = 0.0;
  int seed_containment_evaluated_count = 0;
  int seed_contained_corridor_count = 0;
  double max_seed_containment_violation_m = 0.0;
  int direction_fallback_count = 0;
  int corridor_constrained_piece_count = 0;
  double corridor_penalty_cost_initial = 0.0;
  double corridor_penalty_cost_final = 0.0;
  double max_corridor_violation_initial_m = 0.0;
  double max_corridor_violation_final_m = 0.0;
  int initial_corridor_violating_face_sample_count = 0;
  int final_corridor_violating_face_sample_count = 0;
  int max_corridor_violation_initial_piece_id = -1;
  int max_corridor_violation_initial_face_id = -1;
  int max_corridor_violation_initial_sample_id = -1;
  double max_corridor_violation_initial_time_ratio = 0.0;
  int max_corridor_violation_final_piece_id = -1;
  int max_corridor_violation_final_face_id = -1;
  int max_corridor_violation_final_sample_id = -1;
  double max_corridor_violation_final_time_ratio = 0.0;
  bool final_corridor_enforcement_enabled = false;
  double max_final_violation_allowed_m = 0.0;
  int corridor_enforcement_passes = 0;
  double corridor_penalty_weight_initial = 0.0;
  double corridor_penalty_weight_final = 0.0;
  bool strict_corridor_rejected = false;
  bool progress_guard_enabled = false;
  bool progress_guard_evaluated = false;
  bool progress_guard_passed = true;
  double min_initial_seed_progress_m = 0.0;
  double max_target_axis_progress_drop_m = 0.0;
  double max_target_overshoot_m = 0.0;
  std::string progress_guard_reason = "not_evaluated";
  bool trajectory_repair_enabled = false;
  int trajectory_repair_attempt_count = 0;
  int trajectory_repair_accept_count = 0;
  int trajectory_repair_piece_id = -1;
  int trajectory_repair_candidate_face_count = 0;
  double trajectory_repair_ms = 0.0;
  double trajectory_repair_global_violation_before_m = 0.0;
  double trajectory_repair_global_violation_after_m = 0.0;
  double trajectory_repair_piece_violation_before_m = 0.0;
  double trajectory_repair_piece_violation_after_m = 0.0;
  double trajectory_repair_overlap_previous_m = -1.0;
  double trajectory_repair_overlap_next_m = -1.0;
  bool trajectory_repair_seed_fallback_used = false;
  bool trajectory_repair_geometric_acceptance_used = false;
  double trajectory_repair_candidate_weighted_width_m = 0.0;
  std::string trajectory_repair_sample_source = "not_evaluated";
  std::string trajectory_repair_overlap_anchor_source = "not_evaluated";
  std::string trajectory_repair_primary_failure_reason = "none";
  std::string trajectory_repair_reason = "not_evaluated";
  bool post_optimization_repair_enabled = false;
  int post_optimization_repair_attempt_count = 0;
  int post_optimization_repair_accept_count = 0;
  int post_optimization_repair_piece_id = -1;
  int post_optimization_repair_candidate_face_count = 0;
  double post_optimization_repair_ms = 0.0;
  double post_optimization_repair_violation_before_m = 0.0;
  double post_optimization_repair_violation_after_m = 0.0;
  bool post_optimization_repair_geometric_acceptance_used = false;
  double post_optimization_repair_candidate_weighted_width_m = 0.0;
  double post_optimization_repair_junction_reencode_shift_m = 0.0;
  std::string post_optimization_repair_sample_source = "not_evaluated";
  std::string post_optimization_repair_overlap_anchor_source =
      "not_evaluated";
  std::string post_optimization_repair_reason = "not_evaluated";
  int corridor_candidate_count = 0;
  int corridor_candidate_accept_count = 0;
  bool corridor_rollback_applied = false;
  std::string corridor_rollback_reason = "none";
  double best_corridor_violation_m = 0.0;
  double final_cost = 0.0;
  double trajectory_duration_s = 0.0;
  double trajectory_length_m_sampled = 0.0;
};

class ExperimentLogger
{
public:
  ExperimentLogger(bool enabled,
                   const std::string &directory,
                   const std::string &experiment_tag);

  std::string makeRunId(int drone_id);
  bool log(const ExperimentRunRecord &record,
           const CorridorVector &corridors);

  bool enabled() const { return enabled_; }
  const std::string &experimentTag() const { return experiment_tag_; }

private:
  bool ensureDirectory() const;
  bool appendRun(const ExperimentRunRecord &record) const;
  bool appendCorridors(const ExperimentRunRecord &record,
                       const CorridorVector &corridors) const;

  static std::string csv(const std::string &value);
  static bool fileNeedsHeader(const std::string &path);

  bool enabled_;
  std::string directory_;
  std::string experiment_tag_;
  std::uint64_t sequence_;
  mutable std::mutex mutex_;
};

} // namespace tf_sfc
} // namespace ego_planner
