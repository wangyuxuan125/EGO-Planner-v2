#!/usr/bin/env python3
"""Aggregate TF-SFC schema-v9+ logs without conflating process restarts."""

import argparse
import csv
import math
import statistics
from collections import Counter, defaultdict


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


def summarize(tag, rows):
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
    args = parser.parse_args()

    with open(args.runs_csv, newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("empty run CSV")
    if "planning_event_id" not in rows[0] or "retry_index" not in rows[0]:
        raise SystemExit("schema-v9+ CSV required")

    groups = defaultdict(list)
    for row in rows:
        groups[row.get("experiment_tag", "unknown")].append(row)
    for tag in sorted(groups):
        summarize(tag, groups[tag])


if __name__ == "__main__":
    main()
