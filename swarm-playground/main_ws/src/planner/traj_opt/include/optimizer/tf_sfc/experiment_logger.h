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
  std::string experiment_tag;
  std::string status;
  double timestamp_s = 0.0;
  int drone_id = -1;
  bool tf_sfc_enabled = false;
  int direction_mode = 1;
  bool success = false;
  bool collision_free = false;
  bool tf_sfc_generated = false;
  bool fallback_to_ego = false;
  bool projection_applied = false;
  int lbfgs_result = 0;
  double total_planning_ms = 0.0;
  double optimizer_ms = 0.0;
  double corridor_generation_ms = 0.0;
  int lbfgs_iterations = 0;
  int restart_count = 0;
  int rebound_count = 0;
  int piece_count = 0;
  int corridor_count = 0;
  int total_faces = 0;
  double mean_faces = 0.0;
  double mean_weighted_width = 0.0;
  double min_sample_slack = 0.0;
  double min_overlap_radius = 0.0;
  int direction_fallback_count = 0;
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
