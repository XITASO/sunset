#!/usr/bin/env python3
"""
Generate scenario_NNN.yaml files: each scenario contains exactly one event for every
criticality level found in the input YAML events file.

Usage:
    python generate_scenarios.py [input_yaml]

If input_yaml is omitted, defaults to 'events.yaml'.
Requires PyYAML: pip install pyyaml
"""
import sys
import os
import argparse
import itertools
from collections import OrderedDict

try:
    import yaml
except ImportError:
    print("This script requires PyYAML. Install with: pip install pyyaml", file=sys.stderr)
    sys.exit(2)


def load_events(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, list):
        raise ValueError("Input YAML must be a top-level list of event mappings.")
    return data


def group_by_criticality(events):
    # Preserve the order of first occurrence of criticality levels
    groups = OrderedDict()
    for ev in events:
        level = ev.get("criticality_level")
        if level is None:
            # Treat missing as its own group key (could be None)
            level = None
        groups.setdefault(level, []).append(ev)
    return groups


def write_scenarios(combinations, out_dir=".", width=3, start: int=1):
    os.makedirs(out_dir, exist_ok=True)
    total = len(combinations)
    pad = max(width, len(str(total)))
    for idx, combo in enumerate(combinations, start=start):
        filename = os.path.join(out_dir, f"scenario_{idx:0{pad}d}.yaml")
        # Dump the list of event dicts into YAML
        with open(filename, "w", encoding="utf-8") as f:
            # Ensure readable block style and preserve key order (PyYAML >=5.1 supports sort_keys=False)
            yaml.safe_dump(combo, f, sort_keys=False, default_flow_style=False)
    return total


def main():
    parser = argparse.ArgumentParser(description="Generate scenario files from events grouped by criticality level.")
    parser.add_argument("input", nargs="?", default="events.yaml", help="Input YAML file (default: events.yaml)")
    parser.add_argument("--out-dir", default=".", help="Directory to write scenario files (default: current directory)")
    parser.add_argument("--min-pad", type=int, default=3, help="Minimum zero-pad width for filenames (default: 3)")
    args = parser.parse_args()

    try:
        events = load_events(args.input)
    except Exception as e:
        print(f"Failed to load input YAML '{args.input}': {e}", file=sys.stderr)
        sys.exit(2)

    groups = group_by_criticality(events)
    if not groups:
        print("No events found in input.", file=sys.stderr)
        sys.exit(1)

    # Check that each criticality level has at least one event
    empty_levels = [k for k, v in groups.items() if not v]
    if empty_levels:
        print(f"The following criticality levels have no events: {empty_levels}", file=sys.stderr)
        sys.exit(1)

    # Build all combinations picking exactly one event from each criticality level.
    # Preserve the order of criticality levels as keys in groups.
    group_lists = list(groups.values())
    combos = list(itertools.product(*group_lists))
    # Each combo is a tuple of event dicts; convert to list for YAML dumping
    combos_as_lists = [list(c) for c in combos]

    total = 1
    for _ in range(3):
        total += write_scenarios(combos_as_lists, out_dir=args.out_dir, width=args.min_pad, start=total)
    print(f"Wrote {total} scenario files to '{os.path.abspath(args.out_dir)}' (prefix 'scenario_').")


if __name__ == "__main__":
    main()