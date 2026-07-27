#!/usr/bin/env python3
"""Build the HiDrive Table 4 ability table from per-method evaluation logs.

Input manifest format:
{
  "KnowVal [25]": "/path/to/full_eval_result_dir",
  "SimLingo [18]": "/path/to/another_result_dir"
}

Each result directory may either contain route_results/route_*.json or an
ability_scores.json file produced by tools/compute_hidrive_ability_scores.py.
"""

import argparse
import csv
import json
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from compute_hidrive_ability_scores import ABILITY_ORDER, compute_ability_scores, load_records  # noqa: E402

ABILITY_LEVEL = {
    "Emergency avoidance": "Basic",
    "Obstacle detouring": "Basic",
    "Signalized turning": "Basic",
    "Cut-in response": "Basic",
    "Traffic merging": "Basic",
    "Constrained-segment passage": "Basic",
    "Overtaking": "Basic",
    "U-turn execution": "Basic",
    "Narrow-road following": "Basic",
    "Reasonable speed keeping": "Basic",
    "Oncoming encounter etiquette": "Basic",
    "Pedestrian-related ethics": "Hard",
    "Special yielding scenarios": "Hard",
    "Open-world detouring": "Hard",
    "Speed-bump handling": "Hard",
    "Yielding in tight conflicts": "Hard",
    "Defensive distancing from erratic drivers": "Hard",
    "Police-stop compliance": "Hard",
    "Adverse-weather handling": "Hard",
    "Ego-failure mitigation": "Hard",
    "Defensive turning under occlusion": "Hard",
    "Forced lane borrowing": "Thorny",
    "Signal-failure intersection handling": "Thorny",
    "Intrusive cut-in risk mitigation": "Thorny",
    "Accident-scene handling": "Thorny",
    "Wrong-way vehicle avoidance": "Thorny",
    "Red-light emergency yielding": "Thorny",
    "Partial sensor-blindness handling": "Thorny",
    "Value-priority dilemma handling": "Thorny",
    "Defensive distancing from unknown objects": "Thorny",
}


def load_scores(path):
    if os.path.isdir(path):
        precomputed = os.path.join(path, "ability_scores.json")
        if os.path.isfile(precomputed):
            with open(precomputed) as f:
                return json.load(f)
        return compute_ability_scores(load_records(path))

    with open(path) as f:
        data = json.load(f)
    if isinstance(data, dict) and all(ability in data for ability in ABILITY_ORDER if ability in data):
        return data
    raise ValueError("Unsupported score input: {}".format(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", help="JSON mapping method names to result dirs or ability_scores.json files")
    parser.add_argument("-o", "--output", default="table4_reproduced.csv")
    args = parser.parse_args()

    with open(args.manifest) as f:
        manifest = json.load(f)
    if not isinstance(manifest, dict) or not manifest:
        raise SystemExit("manifest must be a non-empty JSON object")

    method_scores = {method: load_scores(path) for method, path in manifest.items()}
    methods = list(manifest.keys())

    with open(args.output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Set", "Ability"] + methods)
        for ability in ABILITY_ORDER:
            row = [ABILITY_LEVEL.get(ability, ""), ability]
            for method in methods:
                value = method_scores[method].get(ability)
                row.append("" if value is None else "{:.2f}".format(float(value)))
            writer.writerow(row)

    print(args.output)


if __name__ == "__main__":
    main()
