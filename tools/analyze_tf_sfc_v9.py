#!/usr/bin/env python3
"""Aggregate TF-SFC schema-v9+ logs without conflating process restarts."""

import argparse
import csv
import math
import os
import re
import statistics
from collections import Counter, defaultdict


def validate_schema_provenance(path, rows, table_name):
    """Reject silently relabelled logs before producing paper statistics."""
    versions = set()
    for row in rows:
        try:
            versions.add(int(float(row.get("schema_version", "nan"))))
        except (TypeError, ValueError):
            raise SystemExit(f"{table_name}: invalid schema_version")
    if len(versions) != 1:
        raise SystemExit(
            f"{table_name}: mixed schema versions {sorted(versions)}"
        )
    version = next(iter(versions))
    match = re.search(r"_v(\d+)_", os.path.basename(path))
    if match and int(match.group(1)) != version:
        raise SystemExit(
            f"{table_name}: filename claims v{match.group(1)} but CSV "
            f"contains schema v{version}; rebuild/source the intended binary"
        )
    return version


def as_bool(row, key):
    return row.get(key, "0").strip().lower() in {"1", "true", "yes"}


def as_float(row, key):
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan
    return value


def percentile(values, probability):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return math.nan
    index = max(0, min(len(values) - 1,
                       math.ceil(probability * len(values)) - 1))
    return values[index]


def wilson(successes, trials, z=1.959963984540054):
    if trials <= 0:
        return math.nan, math.nan
    p = successes / trials
    denominator = 1.0 + z * z / trials
    center = (p + z * z / (2.0 * trials)) / denominator
    radius = z * math.sqrt(
        p * (1.0 - p) / trials + z * z / (4.0 * trials * trials)
    ) / denominator
    return max(0.0, center - radius), min(1.0, center + radius)


def rate_text(successes, trials):
    if trials <= 0:
        return "n/a"
    low, high = wilson(successes, trials)
    return f"{successes}/{trials}={successes / trials:.3%} " \
           f"(Wilson95% {low:.3%}..{high:.3%})"


def event_key(row):
    event_id = row.get("planning_event_id", "")
    if event_id:
        return event_id
    return "-".join((row.get("drone_id", ""),
                     row.get("goal_id", ""),
                     row.get("replan_id", "")))


def as_int(row, key):
    try:
        return int(float(row.get(key, "0") or 0))
    except (TypeError, ValueError):
        return 0


def sessionize(rows):
    """Assign a local process-session id whenever replan numbering restarts."""
    ordered = sorted(rows, key=lambda row: as_float(row, "timestamp_s"))
    session_id = 0
    previous_replan = None
    annotated = []
    for row in ordered:
        replan_id = as_int(row, "replan_id")
        if previous_replan is not None and replan_id <= previous_replan:
            session_id += 1
        annotated.append((session_id, row))
        previous_replan = replan_id
    return annotated


def closed_loop_regressions(rows, threshold):
    """Measure motion opposite each session's fixed start-to-goal axis."""
    annotated = sessionize(rows)
    sessions = defaultdict(list)
    for session_id, row in annotated:
        sessions[session_id].append(row)
    regressions = []
    transition_count = 0
    for group in sessions.values():
        if len(group) < 2:
            continue
        first = group[0]
        start = [as_float(first, f"planning_start_{axis}_m")
                 for axis in "xyz"]
        goal = [as_float(first, f"commanded_goal_{axis}_m")
                for axis in "xyz"]
        direction = [goal[i] - start[i] for i in range(3)]
        norm = math.sqrt(sum(value * value for value in direction))
        if not math.isfinite(norm) or norm <= 1.0e-9:
            continue
        direction = [value / norm for value in direction]
        for current, following in zip(group, group[1:]):
            if not as_bool(current, "success"):
                continue
            current_position = [
                as_float(current, f"planning_start_{axis}_m")
                for axis in "xyz"
            ]
            following_position = [
                as_float(following, f"planning_start_{axis}_m")
                for axis in "xyz"
            ]
            delta = sum((following_position[i] - current_position[i]) *
                        direction[i] for i in range(3))
            if not math.isfinite(delta):
                continue
            transition_count += 1
            if delta < -threshold:
                regressions.append(-delta)
    return transition_count, regressions


def finite_column(rows, key):
    return [value for value in (as_float(row, key) for row in rows)
            if math.isfinite(value)]


def summarize_corridors(tag, rows):
    valid_rows = [row for row in rows if as_bool(row, "valid")]
    if not valid_rows:
        print(f"corridor geometry [{tag}]: no valid corridors")
        return
    faces = finite_column(valid_rows, "face_count")
    widths = finite_column(valid_rows, "weighted_width")
    quality = finite_column(valid_rows, "region_quality_score")
    overlap = [value for value in finite_column(
        valid_rows, "overlap_radius_to_next") if value >= 0.0]
    alignment = finite_column(
        valid_rows, "direction_velocity_alignment_cosine"
    )
    candidate_evaluations = finite_column(
        valid_rows, "inflation_candidate_evaluation_count"
    )
    metric_condition = finite_column(
        valid_rows, "direction_metric_condition_number"
    )
    transport_weights = finite_column(
        valid_rows, "direction_transport_conditioning_weight"
    ) if "direction_transport_conditioning_weight" in valid_rows[0] else []
    pruned_faces = finite_column(
        valid_rows, "obstacle_face_prune_count"
    ) if "obstacle_face_prune_count" in valid_rows[0] else []
    subset_evaluations = finite_column(
        valid_rows, "face_subset_evaluation_count"
    ) if "face_subset_evaluation_count" in valid_rows[0] else []
    sources = Counter(row.get("direction_metric_source", "unknown")
                      for row in valid_rows)
    print(
        f"corridor geometry [{tag}]: valid={len(valid_rows)}/{len(rows)}; "
        f"faces mean={statistics.mean(faces):.3f} max={max(faces):.0f}; "
        f"weighted-width median={percentile(widths, 0.5):.3f}; "
        f"quality median={percentile(quality, 0.5):.3f}"
    )
    print(
        f"corridor overlap [{tag}]: min={min(overlap):.6f} m "
        f"median={percentile(overlap, 0.5):.6f} m"
        if overlap else f"corridor overlap [{tag}]: n/a"
    )
    print(
        f"direction evidence [{tag}]: sources={dict(sources)}; "
        f"velocity-alignment median={percentile(alignment, 0.5):.3f}; "
        f"metric-condition median={percentile(metric_condition, 0.5):.3f}; "
        + (f"transport-weight median="
           f"{percentile(transport_weights, 0.5):.3f}; "
           if transport_weights else "") +
        f"inflation candidate evaluations p90="
        f"{percentile(candidate_evaluations, 0.9):.0f}"
    )
    if pruned_faces:
        print(
            f"exact face-subset pruning [{tag}]: corridors with removals="
            f"{sum(value > 0 for value in pruned_faces)}/{len(pruned_faces)}; "
            f"removed total/median/p90={sum(pruned_faces):.0f}/"
            f"{percentile(pruned_faces, 0.5):.0f}/"
            f"{percentile(pruned_faces, 0.9):.0f}; "
            f"subset evaluations p90="
            f"{percentile(subset_evaluations, 0.9):.0f}"
        )


def summarize(tag, rows, progress_regression_threshold):
    events = defaultdict(list)
    goals = defaultdict(list)
    annotated = sessionize(rows)
    for session_id, row in annotated:
        events[(session_id, event_key(row))].append(row)
        goals[(session_id, row.get("drone_id", ""),
               row.get("goal_id", ""))].append(row)

    successful_calls = sum(as_bool(row, "success") for row in rows)
    successful_events = sum(any(as_bool(row, "success") for row in group)
                            for group in events.values())
    goals_with_planning_success = sum(
        any(as_bool(row, "success") for row in group)
        for group in goals.values()
    )

    search_rows = [row for row in rows
                   if as_bool(row, "astar_search_attempted")]
    search_successes = sum(as_bool(row, "astar_search_success")
                           for row in search_rows)
    generated_rows = [row for row in rows
                      if as_bool(row, "tf_sfc_generated")]
    conditional_successes = sum(as_bool(row, "success")
                                for row in generated_rows)

    successful_rows = [row for row in rows if as_bool(row, "success")]
    total_ms = [as_float(row, "total_planning_ms")
                for row in successful_rows]
    search_ms = [as_float(row, "astar_search_ms") for row in rows]
    inflation_ms = [as_float(row, "corridor_inflation_ms") for row in rows]

    containment_evaluated = sum(
        int(float(row.get("seed_containment_evaluated_count", "0") or 0))
        for row in rows
    )
    containment_success = sum(
        int(float(row.get("seed_contained_corridor_count", "0") or 0))
        for row in rows
    )
    failures = Counter(
        row.get("terminal_failure_reason", "unknown")
        for row in rows if not as_bool(row, "success")
    )

    print(f"\n[{tag}]")
    print(f"process sessions: {len({session_id for session_id, _ in annotated})}")
    print(f"optimizer-call success (diagnostic): "
          f"{rate_text(successful_calls, len(rows))}")
    print(f"planning-event success: "
          f"{rate_text(successful_events, len(events))}")
    print(f"goals with >=1 successful plan (not mission arrival): "
          f"{rate_text(goals_with_planning_success, len(goals))}")
    print(f"A* search success: {rate_text(search_successes, len(search_rows))}")
    print(f"corridor generation: "
          f"{rate_text(len(generated_rows), len(rows))}")
    print(f"optimizer success | valid corridor: "
          f"{rate_text(conditional_successes, len(generated_rows))}")
    print(f"seed containment: "
          f"{rate_text(containment_success, containment_evaluated)}")
    retry_rows = sum(int(float(row.get("retry_index", "0") or 0)) > 0
                     for row in rows)
    print(f"retry calls: {retry_rows}/{len(rows)}")
    progress_transitions, progress_regressions = closed_loop_regressions(
        rows, progress_regression_threshold
    )
    print(
        f"closed-loop regressions >{progress_regression_threshold:.3f} m "
        f"after success: {len(progress_regressions)}/{progress_transitions}; "
        f"max={max(progress_regressions, default=0.0):.3f} m"
    )
    if "seed_minco_alignment_valid" in rows[0]:
        evaluated_alignment = [
            row for row in rows if as_int(row, "seed_piece_count") > 0
        ]
        aligned = sum(as_bool(row, "seed_minco_alignment_valid")
                      for row in evaluated_alignment)
        print(f"seed/MINCO slot alignment: "
              f"{rate_text(aligned, len(evaluated_alignment))}")
    if "trajectory_repair_attempt_count" in rows[0]:
        repair_attempts = sum(as_int(row, "trajectory_repair_attempt_count")
                              for row in rows)
        repair_accepts = sum(as_int(row, "trajectory_repair_accept_count")
                             for row in rows)
        post_attempts = sum(
            as_int(row, "post_optimization_repair_attempt_count")
            for row in rows
        )
        post_accepts = sum(
            as_int(row, "post_optimization_repair_accept_count")
            for row in rows
        )
        print(f"trajectory repair acceptance: "
              f"{rate_text(repair_accepts, repair_attempts)}")
        print(f"post-optimization repair acceptance: "
              f"{rate_text(post_accepts, post_attempts)}")
    if "progress_guard_evaluated" in rows[0]:
        progress_rows = [row for row in rows
                         if as_bool(row, "progress_guard_evaluated")]
        progress_passes = sum(as_bool(row, "progress_guard_passed")
                              for row in progress_rows)
        progress_reasons = Counter(
            row.get("progress_guard_reason", "unknown")
            for row in progress_rows if not as_bool(
                row, "progress_guard_passed")
        )
        print(f"progress guard acceptance: "
              f"{rate_text(progress_passes, len(progress_rows))}; "
              f"rejections={dict(progress_reasons)}")
    if "objective_compliance_attempted" in rows[0]:
        compliance_rows = [row for row in rows
                           if as_bool(row, "objective_compliance_attempted")]
        compliance_successes = sum(
            as_bool(row, "objective_compliance_success")
            for row in compliance_rows
        )
        compliance_ms = finite_column(
            compliance_rows, "objective_compliance_ms"
        )
        compliance_evaluations = finite_column(
            compliance_rows, "objective_compliance_evaluation_count"
        )
        compliance_regularized = finite_column(
            compliance_rows,
            "objective_compliance_regularized_eigenvalue_count",
        )
        compliance_failures = Counter(
            row.get("objective_compliance_reason", "unknown")
            for row in compliance_rows
            if not as_bool(row, "objective_compliance_success")
        )
        print(
            "full-objective compliance: "
            f"{rate_text(compliance_successes, len(compliance_rows))}; "
            f"ms median/p90={percentile(compliance_ms, 0.5):.3f}/"
            f"{percentile(compliance_ms, 0.9):.3f}; "
            f"evaluations median={percentile(compliance_evaluations, 0.5):.0f}; "
            f"regularized eigenvalues median="
            f"{percentile(compliance_regularized, 0.5):.0f}; "
            f"failures={dict(compliance_failures)}"
        )
    if "face_sample_pairs_per_evaluation" in rows[0]:
        face_sample_pairs = finite_column(
            successful_rows, "face_sample_pairs_per_evaluation"
        )
        direct_variables = finite_column(
            successful_rows, "direct_spatial_variable_count"
        )
        hard_variables = finite_column(
            successful_rows, "hard_spatial_variable_count"
        )
        print(
            "successful constraint workload: face-sample pairs/evaluation "
            f"median={percentile(face_sample_pairs, 0.5):.1f} "
            f"p90={percentile(face_sample_pairs, 0.9):.1f}; "
            f"direct/hard spatial variables median="
            f"{percentile(direct_variables, 0.5):.1f}/"
            f"{percentile(hard_variables, 0.5):.1f}"
        )
    print(f"successful total planning ms: median="
          f"{statistics.median([v for v in total_ms if math.isfinite(v)]):.3f} "
          f"p95={percentile(total_ms, 0.95):.3f}"
          if any(math.isfinite(v) for v in total_ms)
          else "successful total planning ms: n/a")
    print(f"all-call A* ms: median={percentile(search_ms, 0.5):.3f} "
          f"p95={percentile(search_ms, 0.95):.3f}")
    print(f"all-call corridor inflation ms: "
          f"median={percentile(inflation_ms, 0.5):.3f} "
          f"p95={percentile(inflation_ms, 0.95):.3f}")
    print("failure decomposition:", dict(failures))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs_csv")
    parser.add_argument(
        "--max-progress-regression",
        type=float,
        default=0.05,
        help="closed-loop reverse-motion threshold in metres (default: 0.05)",
    )
    parser.add_argument(
        "--corridors-csv",
        help="optional matching schema-v24+ corridor CSV",
    )
    args = parser.parse_args()

    with open(args.runs_csv, newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("empty run CSV")
    if "planning_event_id" not in rows[0] or "retry_index" not in rows[0]:
        raise SystemExit("schema-v9+ CSV required")
    run_schema = validate_schema_provenance(args.runs_csv, rows, "runs CSV")
    if args.max_progress_regression < 0.0:
        raise SystemExit("--max-progress-regression must be non-negative")

    groups = defaultdict(list)
    for row in rows:
        groups[row.get("experiment_tag", "unknown")].append(row)
    for tag in sorted(groups):
        summarize(tag, groups[tag], args.max_progress_regression)
    if args.corridors_csv:
        with open(args.corridors_csv, newline="") as stream:
            corridor_rows = list(csv.DictReader(stream))
        if not corridor_rows:
            raise SystemExit("empty corridor CSV")
        corridor_schema = validate_schema_provenance(
            args.corridors_csv, corridor_rows, "corridors CSV"
        )
        if corridor_schema != run_schema:
            raise SystemExit(
                f"run/corridor schema mismatch: v{run_schema} versus "
                f"v{corridor_schema}"
            )
        if corridor_schema >= 26:
            required = "direction_transport_conditioning_weight"
            if required not in corridor_rows[0]:
                raise SystemExit(
                    f"schema v{corridor_schema} corridor CSV lacks {required}"
                )
            invalid_mode3 = [
                row for row in corridor_rows
                if as_bool(row, "valid")
                and int(float(row.get("used_direction_mode", "-1") or -1)) == 3
                and row.get("direction_metric_source") !=
                "minco_transport_conditioned_objective_compliance"
            ]
            if invalid_mode3:
                raise SystemExit(
                    f"schema v{corridor_schema}: {len(invalid_mode3)} valid "
                    "mode-3 corridors lack transport-conditioned provenance"
                )
        if corridor_schema >= 27:
            required_pruning = {
                "obstacle_face_count_before_pruning",
                "obstacle_face_prune_count",
                "face_subset_evaluation_count",
            }
            missing_pruning = required_pruning.difference(corridor_rows[0])
            if missing_pruning:
                raise SystemExit(
                    f"schema v{corridor_schema} corridor CSV lacks "
                    f"{sorted(missing_pruning)}"
                )
        corridor_groups = defaultdict(list)
        for row in corridor_rows:
            corridor_groups[row.get("experiment_tag", "unknown")].append(row)
        for tag in sorted(corridor_groups):
            summarize_corridors(tag, corridor_groups[tag])


if __name__ == "__main__":
    main()
