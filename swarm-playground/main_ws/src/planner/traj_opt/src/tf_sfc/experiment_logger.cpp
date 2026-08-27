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
    "velocity_seed_fallback_reason,seed_validation_failure_point_id,"
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
    "direction_fallback_allowed,success,collision_free,tf_sfc_generated,"
    "final_obstacle_collision,final_swarm_clearance_failure,terminal_failure_reason,"
    "fallback_to_ego,projection_applied,hard_parameterization_enabled,"
    "hard_parameterization_active,hard_constrained_junction_count,"
    "hard_total_junction_count,direct_spatial_variable_count,"
    "hard_spatial_variable_count,hard_spatial_variable_overhead_ratio,"
    "face_sample_pairs_per_evaluation,"
    "max_junction_violation_initial_m,max_junction_violation_final_m,"
    "lbfgs_result,total_planning_ms,"
    "optimizer_ms,corridor_generation_ms,seed_path_build_ms,"
    "corridor_inflation_ms,lbfgs_iterations,restart_count,"
    "rebound_count,piece_count,corridor_count,failed_piece_count,first_failure_reason,"
    "total_faces,mean_faces,"
    "mean_weighted_width,min_sample_slack,min_overlap_radius,"
    "seed_containment_evaluated_count,seed_contained_corridor_count,"
    "max_seed_containment_violation_m,direction_fallback_count,"
    "corridor_constrained_piece_count,"
    "corridor_penalty_cost_initial,corridor_penalty_cost_final,"
    "max_corridor_violation_initial_m,max_corridor_violation_final_m,"
    "final_corridor_enforcement_enabled,max_final_violation_allowed_m,"
    "corridor_enforcement_passes,corridor_penalty_weight_initial,"
    "corridor_penalty_weight_final,"
    "strict_corridor_rejected,corridor_candidate_count,"
    "corridor_candidate_accept_count,corridor_rollback_applied,"
    "corridor_rollback_reason,best_corridor_violation_m,"
    "final_cost,trajectory_duration_s,"
    "trajectory_length_m_sampled";

const char *kCorridorHeader =
    "schema_version,run_id,timestamp_s,experiment_tag,drone_id,piece_id,"
    "requested_method,method,"
    "face_count,obstacle_face_count,obstacle_point_count,"
    "inflation_candidate_evaluation_count,face_budget_saturated,"
    "generation_time_ms,weighted_width,min_sample_slack,anchor_clearance_radius,"
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
  const std::string path = directory_ + "/ego_runs_v16_drone_" +
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
         << 16 << ',' << csv(record.run_id) << ','
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
         << record.direction_fallback_allowed << ',' << record.success << ','
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
         << record.max_junction_violation_initial_m << ','
         << record.max_junction_violation_final_m << ','
         << record.lbfgs_result << ',' << record.total_planning_ms << ','
         << record.optimizer_ms << ',' << record.corridor_generation_ms << ','
         << record.seed_path_build_ms << ','
         << record.corridor_inflation_ms << ','
         << record.lbfgs_iterations << ',' << record.restart_count << ','
         << record.rebound_count << ',' << record.piece_count << ','
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
         << record.final_corridor_enforcement_enabled << ','
         << record.max_final_violation_allowed_m << ','
         << record.corridor_enforcement_passes << ','
         << record.corridor_penalty_weight_initial << ','
         << record.corridor_penalty_weight_final << ','
         << record.strict_corridor_rejected << ','
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
  const std::string path = directory_ + "/ego_corridors_v16_drone_" +
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
    output << 16 << ',' << csv(record.run_id) << ',' << record.timestamp_s << ','
           << csv(record.experiment_tag) << ',' << record.drone_id << ','
           << metrics.piece_id << ',' << csv(record.requested_method) << ','
           << csv(record.method) << ',' << metrics.face_count << ','
           << metrics.obstacle_face_count << ','
           << metrics.obstacle_point_count << ','
           << metrics.inflation_candidate_evaluation_count << ','
           << metrics.face_budget_saturated << ','
           << metrics.generation_time_ms << ',' << metrics.weighted_width << ','
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
