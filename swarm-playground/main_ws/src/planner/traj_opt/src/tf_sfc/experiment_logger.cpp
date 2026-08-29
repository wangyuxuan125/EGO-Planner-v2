#include "optimizer/tf_sfc/experiment_logger.h"

#include <ros/ros.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace ego_planner
{
namespace tf_sfc
{

namespace
{
const char *kRunHeader =
    "schema_version,run_id,planning_event_id,timestamp_s,experiment_tag,"
    "drone_id,goal_id,replan_id,retry_index,attempt_id,touch_goal,"
    "commanded_goal_valid,commanded_goal_x_m,commanded_goal_y_m,"
    "commanded_goal_z_m,"
    "planning_start_x_m,planning_start_y_m,planning_start_z_m,"
    "planning_target_x_m,planning_target_y_m,planning_target_z_m,"
    "seed_path_strategy,initial_velocity_seed_attempted,"
    "initial_velocity_seed_used,velocity_seed_fallback_used,"
    "velocity_seed_fallback_reason,clearance_retry_attempted,"
    "clearance_retry_success,clearance_retry_initial_strategy,"
    "clearance_retry_first_failure_point_id,clearance_astar_attempted,"
    "clearance_astar_success,clearance_astar_call_count,clearance_astar_ms,"
    "seed_validation_failure_point_id,"
    "astar_search_attempted,astar_search_success,astar_search_call_count,"
    "astar_search_ms,raw_seed_path_point_count,raw_seed_path_length_m,"
    "seed_path_point_count,seed_path_length_m,seed_path_edge_valid,"
    "seed_path_coverage_ratio,allow_partial_corridors,seed_start_in_map,"
    "seed_finish_in_map,partial_target_search_attempted,partial_target_found,"
    "partial_boundary_ratio,inflated_map_low_x_m,inflated_map_low_y_m,"
    "inflated_map_low_z_m,inflated_map_high_x_m,inflated_map_high_y_m,"
    "inflated_map_high_z_m,seed_frontend_evaluated,seed_frontend_success,"
    "corridor_generation_attempted,status,requested_method,method,"
    "tf_sfc_enabled,direction_mode,used_direction_mode,"
    "direction_fallback_allowed,objective_compliance_attempted,"
    "objective_compliance_success,"
    "objective_compliance_spatial_variable_count,"
    "objective_compliance_evaluation_count,"
    "objective_compliance_regularized_eigenvalue_count,"
    "objective_compliance_ms,objective_compliance_raw_min_eigenvalue,"
    "objective_compliance_raw_max_eigenvalue,"
    "objective_compliance_regularized_condition_number,"
    "objective_compliance_reason,success,collision_free,tf_sfc_generated,"
    "final_obstacle_collision,final_swarm_clearance_failure,terminal_failure_reason,"
    "fallback_to_ego,projection_applied,hard_parameterization_enabled,"
    "hard_parameterization_active,hard_constrained_junction_count,"
    "hard_total_junction_count,direct_spatial_variable_count,"
    "hard_spatial_variable_count,hard_spatial_variable_overhead_ratio,"
    "face_sample_pairs_per_evaluation,corridor_penalty_samples_per_piece,"
    "max_junction_violation_initial_m,max_junction_violation_final_m,"
    "lbfgs_result,total_planning_ms,"
    "optimizer_ms,corridor_generation_ms,seed_path_build_ms,"
    "corridor_inflation_ms,lbfgs_iterations,restart_count,"
    "rebound_count,piece_count,seed_piece_count,corridor_slot_count,"
    "seed_minco_alignment_valid,corridor_count,failed_piece_count,first_failure_reason,"
    "total_faces,mean_faces,"
    "mean_weighted_width,min_sample_slack,min_overlap_radius,"
    "seed_containment_evaluated_count,seed_contained_corridor_count,"
    "max_seed_containment_violation_m,direction_fallback_count,"
    "corridor_constrained_piece_count,"
    "corridor_penalty_cost_initial,corridor_penalty_cost_final,"
    "max_corridor_violation_initial_m,max_corridor_violation_final_m,"
    "initial_corridor_violating_face_sample_count,"
    "final_corridor_violating_face_sample_count,"
    "max_corridor_violation_initial_piece_id,"
    "max_corridor_violation_initial_face_id,"
    "max_corridor_violation_initial_sample_id,"
    "max_corridor_violation_initial_time_ratio,"
    "max_corridor_violation_final_piece_id,"
    "max_corridor_violation_final_face_id,"
    "max_corridor_violation_final_sample_id,"
    "max_corridor_violation_final_time_ratio,"
    "final_corridor_enforcement_enabled,max_final_violation_allowed_m,"
    "corridor_enforcement_passes,corridor_penalty_weight_initial,"
    "corridor_penalty_weight_final,"
    "strict_corridor_rejected,progress_guard_enabled,"
    "progress_guard_evaluated,progress_guard_passed,"
    "min_initial_seed_progress_m,max_target_axis_progress_drop_m,"
    "max_target_overshoot_m,progress_guard_reason,"
    "trajectory_repair_enabled,"
    "trajectory_repair_attempt_count,trajectory_repair_accept_count,"
    "trajectory_repair_piece_id,trajectory_repair_candidate_face_count,"
    "trajectory_repair_ms,"
    "trajectory_repair_global_violation_before_m,"
    "trajectory_repair_global_violation_after_m,"
    "trajectory_repair_piece_violation_before_m,"
    "trajectory_repair_piece_violation_after_m,"
    "trajectory_repair_overlap_previous_m,trajectory_repair_overlap_next_m,"
    "trajectory_repair_seed_fallback_used,"
    "trajectory_repair_geometric_acceptance_used,"
    "trajectory_repair_candidate_weighted_width_m,"
    "trajectory_repair_sample_source,"
    "trajectory_repair_overlap_anchor_source,"
    "trajectory_repair_primary_failure_reason,trajectory_repair_reason,"
    "post_optimization_repair_enabled,"
    "post_optimization_repair_attempt_count,"
    "post_optimization_repair_accept_count,"
    "post_optimization_repair_piece_id,"
    "post_optimization_repair_candidate_face_count,"
    "post_optimization_repair_ms,"
    "post_optimization_repair_violation_before_m,"
    "post_optimization_repair_violation_after_m,"
    "post_optimization_repair_geometric_acceptance_used,"
    "post_optimization_repair_candidate_weighted_width_m,"
    "post_optimization_repair_junction_reencode_shift_m,"
    "post_optimization_repair_sample_source,"
    "post_optimization_repair_overlap_anchor_source,"
    "post_optimization_repair_reason,corridor_candidate_count,"
    "corridor_candidate_accept_count,corridor_rollback_applied,"
    "corridor_rollback_reason,best_corridor_violation_m,"
    "final_cost,trajectory_duration_s,"
    "trajectory_length_m_sampled";

const char *kCorridorHeader =
    "schema_version,run_id,timestamp_s,experiment_tag,drone_id,piece_id,"
    "requested_method,method,"
    "face_count,obstacle_face_count,obstacle_point_count,"
    "inflation_candidate_evaluation_count,face_budget_saturated,"
    "generation_time_ms,weighted_width,region_quality_score,"
    "face_quality_penalty,direction_metric_source,"
    "direction_metric_eigenvalue_max,direction_metric_eigenvalue_mid,"
    "direction_metric_eigenvalue_min,direction_metric_condition_number,"
    "direction_velocity_alignment_cosine,"
    "direction_transport_conditioning_weight,min_sample_slack,"
    "anchor_clearance_radius,"
    "min_obstacle_sample_distance_m,separation_failure_sample_id,"
    "separation_failure_at_endpoint,overlap_radius_to_next,"
    "seed_containment_evaluated,seed_contained,"
    "seed_containment_max_violation_m,valid,direction_fallback,"
    "requested_direction_mode,used_direction_mode,failure_reason";
}

ExperimentLogger::ExperimentLogger(const bool enabled,
                                   const std::string &directory,
                                   const std::string &experiment_tag)
    : enabled_(enabled),
      directory_(directory),
      experiment_tag_(experiment_tag.empty() ? "default" : experiment_tag),
      sequence_(0)
{
  while (directory_.size() > 1 && directory_.back() == '/')
  {
    directory_.pop_back();
  }
}

std::string ExperimentLogger::makeRunId(const int drone_id)
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now).count();
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream stream;
  stream << micros << "-p" << static_cast<long>(::getpid())
         << "-d" << drone_id << "-" << sequence_++;
  return stream.str();
}

bool ExperimentLogger::log(const ExperimentRunRecord &record,
                           const CorridorVector &corridors)
{
  if (!enabled_)
  {
    return true;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ensureDirectory())
  {
    ROS_ERROR_THROTTLE(1.0, "Cannot create TF-SFC experiment directory: %s",
                       directory_.c_str());
    return false;
  }
  return appendRun(record) && appendCorridors(record, corridors);
}

bool ExperimentLogger::ensureDirectory() const
{
  if (directory_.empty())
  {
    return false;
  }

  std::string current;
  if (directory_.front() == '/')
  {
    current = "/";
  }
  std::istringstream path(directory_);
  std::string part;
  while (std::getline(path, part, '/'))
  {
    if (part.empty())
    {
      continue;
    }
    if (!current.empty() && current.back() != '/')
    {
      current += '/';
    }
    current += part;
    if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
    {
      return false;
    }
  }
  return true;
}

bool ExperimentLogger::appendRun(const ExperimentRunRecord &record) const
{
  const std::string path = directory_ + "/ego_runs_v26_drone_" +
                           std::to_string(record.drone_id) + ".csv";
  const bool header = fileNeedsHeader(path);
  std::ofstream output(path, std::ios::out | std::ios::app);
  if (!output)
  {
    return false;
  }
  if (header)
  {
    output << kRunHeader << '\n';
  }
  output << std::setprecision(17)
         << 26 << ',' << csv(record.run_id) << ','
         << csv(record.planning_event_id) << ',' << record.timestamp_s << ','
         << csv(record.experiment_tag) << ',' << record.drone_id << ','
         << record.goal_id << ',' << record.replan_id << ','
         << record.retry_index << ',' << record.attempt_id << ','
         << record.touch_goal << ','
         << record.commanded_goal_valid << ','
         << record.commanded_goal_x_m << ','
         << record.commanded_goal_y_m << ','
         << record.commanded_goal_z_m << ','
         << record.planning_start_x_m << ','
         << record.planning_start_y_m << ','
         << record.planning_start_z_m << ','
         << record.planning_target_x_m << ','
         << record.planning_target_y_m << ','
         << record.planning_target_z_m << ','
         << csv(record.seed_path_strategy) << ','
         << record.initial_velocity_seed_attempted << ','
         << record.initial_velocity_seed_used << ','
         << record.velocity_seed_fallback_used << ','
         << csv(record.velocity_seed_fallback_reason) << ','
         << record.clearance_retry_attempted << ','
         << record.clearance_retry_success << ','
         << csv(record.clearance_retry_initial_strategy) << ','
         << record.clearance_retry_first_failure_point_id << ','
         << record.clearance_astar_attempted << ','
         << record.clearance_astar_success << ','
         << record.clearance_astar_call_count << ','
         << record.clearance_astar_ms << ','
         << record.seed_validation_failure_point_id << ','
         << record.astar_search_attempted << ','
         << record.astar_search_success << ','
         << record.astar_search_call_count << ','
         << record.astar_search_ms << ','
         << record.raw_seed_path_point_count << ','
         << record.raw_seed_path_length_m << ','
         << record.seed_path_point_count << ','
         << record.seed_path_length_m << ','
         << record.seed_path_edge_valid << ','
         << record.seed_path_coverage_ratio << ','
         << record.allow_partial_corridors << ','
         << record.seed_start_in_map << ','
         << record.seed_finish_in_map << ','
         << record.partial_target_search_attempted << ','
         << record.partial_target_found << ','
         << record.partial_boundary_ratio << ','
         << record.inflated_map_low_x_m << ','
         << record.inflated_map_low_y_m << ','
         << record.inflated_map_low_z_m << ','
         << record.inflated_map_high_x_m << ','
         << record.inflated_map_high_y_m << ','
         << record.inflated_map_high_z_m << ','
         << record.seed_frontend_evaluated << ','
         << record.seed_frontend_success << ','
         << record.corridor_generation_attempted << ','
         << csv(record.status) << ',' << csv(record.requested_method) << ','
         << csv(record.method) << ',' << record.tf_sfc_enabled << ','
         << record.direction_mode << ',' << record.used_direction_mode << ','
         << record.direction_fallback_allowed << ','
         << record.objective_compliance_attempted << ','
         << record.objective_compliance_success << ','
         << record.objective_compliance_spatial_variable_count << ','
         << record.objective_compliance_evaluation_count << ','
         << record.objective_compliance_regularized_eigenvalue_count << ','
         << record.objective_compliance_ms << ','
         << record.objective_compliance_raw_min_eigenvalue << ','
         << record.objective_compliance_raw_max_eigenvalue << ','
         << record.objective_compliance_regularized_condition_number << ','
         << csv(record.objective_compliance_reason) << ','
         << record.success << ','
         << record.collision_free << ',' << record.tf_sfc_generated << ','
         << record.final_obstacle_collision << ','
         << record.final_swarm_clearance_failure << ','
         << csv(record.terminal_failure_reason) << ','
         << record.fallback_to_ego << ',' << record.projection_applied << ','
         << record.hard_parameterization_enabled << ','
         << record.hard_parameterization_active << ','
         << record.hard_constrained_junction_count << ','
         << record.hard_total_junction_count << ','
         << record.direct_spatial_variable_count << ','
         << record.hard_spatial_variable_count << ','
         << record.hard_spatial_variable_overhead_ratio << ','
         << record.face_sample_pairs_per_evaluation << ','
         << record.corridor_penalty_samples_per_piece << ','
         << record.max_junction_violation_initial_m << ','
         << record.max_junction_violation_final_m << ','
         << record.lbfgs_result << ',' << record.total_planning_ms << ','
         << record.optimizer_ms << ',' << record.corridor_generation_ms << ','
         << record.seed_path_build_ms << ','
         << record.corridor_inflation_ms << ','
         << record.lbfgs_iterations << ',' << record.restart_count << ','
         << record.rebound_count << ',' << record.piece_count << ','
         << record.seed_piece_count << ',' << record.corridor_slot_count << ','
         << record.seed_minco_alignment_valid << ','
         << record.corridor_count << ',' << record.failed_piece_count << ','
         << csv(record.first_failure_reason) << ',' << record.total_faces << ','
         << record.mean_faces << ',' << record.mean_weighted_width << ','
         << record.min_sample_slack << ',' << record.min_overlap_radius << ','
         << record.seed_containment_evaluated_count << ','
         << record.seed_contained_corridor_count << ','
         << record.max_seed_containment_violation_m << ','
         << record.direction_fallback_count << ','
         << record.corridor_constrained_piece_count << ','
         << record.corridor_penalty_cost_initial << ','
         << record.corridor_penalty_cost_final << ','
         << record.max_corridor_violation_initial_m << ','
         << record.max_corridor_violation_final_m << ','
         << record.initial_corridor_violating_face_sample_count << ','
         << record.final_corridor_violating_face_sample_count << ','
         << record.max_corridor_violation_initial_piece_id << ','
         << record.max_corridor_violation_initial_face_id << ','
         << record.max_corridor_violation_initial_sample_id << ','
         << record.max_corridor_violation_initial_time_ratio << ','
         << record.max_corridor_violation_final_piece_id << ','
         << record.max_corridor_violation_final_face_id << ','
         << record.max_corridor_violation_final_sample_id << ','
         << record.max_corridor_violation_final_time_ratio << ','
         << record.final_corridor_enforcement_enabled << ','
         << record.max_final_violation_allowed_m << ','
         << record.corridor_enforcement_passes << ','
         << record.corridor_penalty_weight_initial << ','
         << record.corridor_penalty_weight_final << ','
         << record.strict_corridor_rejected << ','
         << record.progress_guard_enabled << ','
         << record.progress_guard_evaluated << ','
         << record.progress_guard_passed << ','
         << record.min_initial_seed_progress_m << ','
         << record.max_target_axis_progress_drop_m << ','
         << record.max_target_overshoot_m << ','
         << csv(record.progress_guard_reason) << ','
         << record.trajectory_repair_enabled << ','
         << record.trajectory_repair_attempt_count << ','
         << record.trajectory_repair_accept_count << ','
         << record.trajectory_repair_piece_id << ','
         << record.trajectory_repair_candidate_face_count << ','
         << record.trajectory_repair_ms << ','
         << record.trajectory_repair_global_violation_before_m << ','
         << record.trajectory_repair_global_violation_after_m << ','
         << record.trajectory_repair_piece_violation_before_m << ','
         << record.trajectory_repair_piece_violation_after_m << ','
         << record.trajectory_repair_overlap_previous_m << ','
         << record.trajectory_repair_overlap_next_m << ','
         << record.trajectory_repair_seed_fallback_used << ','
         << record.trajectory_repair_geometric_acceptance_used << ','
         << record.trajectory_repair_candidate_weighted_width_m << ','
         << csv(record.trajectory_repair_sample_source) << ','
         << csv(record.trajectory_repair_overlap_anchor_source) << ','
         << csv(record.trajectory_repair_primary_failure_reason) << ','
         << csv(record.trajectory_repair_reason) << ','
         << record.post_optimization_repair_enabled << ','
         << record.post_optimization_repair_attempt_count << ','
         << record.post_optimization_repair_accept_count << ','
         << record.post_optimization_repair_piece_id << ','
         << record.post_optimization_repair_candidate_face_count << ','
         << record.post_optimization_repair_ms << ','
         << record.post_optimization_repair_violation_before_m << ','
         << record.post_optimization_repair_violation_after_m << ','
         << record.post_optimization_repair_geometric_acceptance_used << ','
         << record.post_optimization_repair_candidate_weighted_width_m << ','
         << record.post_optimization_repair_junction_reencode_shift_m << ','
         << csv(record.post_optimization_repair_sample_source) << ','
         << csv(record.post_optimization_repair_overlap_anchor_source) << ','
         << csv(record.post_optimization_repair_reason) << ','
         << record.corridor_candidate_count << ','
         << record.corridor_candidate_accept_count << ','
         << record.corridor_rollback_applied << ','
         << csv(record.corridor_rollback_reason) << ','
         << record.best_corridor_violation_m << ','
         << record.final_cost << ','
         << record.trajectory_duration_s << ','
         << record.trajectory_length_m_sampled << '\n';
  return static_cast<bool>(output);
}

bool ExperimentLogger::appendCorridors(const ExperimentRunRecord &record,
                                       const CorridorVector &corridors) const
{
  if (corridors.empty())
  {
    return true;
  }
  const std::string path = directory_ + "/ego_corridors_v26_drone_" +
                           std::to_string(record.drone_id) + ".csv";
  const bool header = fileNeedsHeader(path);
  std::ofstream output(path, std::ios::out | std::ios::app);
  if (!output)
  {
    return false;
  }
  if (header)
  {
    output << kCorridorHeader << '\n';
  }
  output << std::setprecision(17);
  for (const Corridor &corridor : corridors)
  {
    const CorridorMetrics &metrics = corridor.metrics;
    output << 26 << ',' << csv(record.run_id) << ',' << record.timestamp_s << ','
           << csv(record.experiment_tag) << ',' << record.drone_id << ','
           << metrics.piece_id << ',' << csv(record.requested_method) << ','
           << csv(record.method) << ',' << metrics.face_count << ','
           << metrics.obstacle_face_count << ','
           << metrics.obstacle_point_count << ','
           << metrics.inflation_candidate_evaluation_count << ','
           << metrics.face_budget_saturated << ','
           << metrics.generation_time_ms << ',' << metrics.weighted_width << ','
           << metrics.region_quality_score << ','
           << metrics.face_quality_penalty << ','
           << csv(metrics.direction_metric_source) << ','
           << metrics.direction_metric_eigenvalue_max << ','
           << metrics.direction_metric_eigenvalue_mid << ','
           << metrics.direction_metric_eigenvalue_min << ','
           << metrics.direction_metric_condition_number << ','
           << metrics.direction_velocity_alignment_cosine << ','
           << metrics.direction_transport_conditioning_weight << ','
           << metrics.min_sample_slack << ','
           << metrics.anchor_clearance_radius << ','
           << metrics.min_obstacle_sample_distance_m << ','
           << metrics.separation_failure_sample_id << ','
           << metrics.separation_failure_at_endpoint << ','
           << metrics.overlap_radius_to_next << ','
           << metrics.seed_containment_evaluated << ','
           << metrics.seed_contained << ','
           << metrics.seed_containment_max_violation_m << ','
           << metrics.valid << ',' << metrics.direction_fallback << ','
           << metrics.requested_direction_mode << ','
           << metrics.used_direction_mode << ','
           << failureReasonName(metrics.failure_reason) << '\n';
  }
  return static_cast<bool>(output);
}

std::string ExperimentLogger::csv(const std::string &value)
{
  if (value.find_first_of(",\"\n\r") == std::string::npos)
  {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value)
  {
    escaped += c;
    if (c == '"')
    {
      escaped += '"';
    }
  }
  escaped += '"';
  return escaped;
}

bool ExperimentLogger::fileNeedsHeader(const std::string &path)
{
  std::ifstream input(path, std::ios::binary);
  return !input || input.peek() == std::ifstream::traits_type::eof();
}

} // namespace tf_sfc
} // namespace ego_planner
