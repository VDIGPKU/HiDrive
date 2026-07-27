import argparse
import glob
import json
import os
import statistics


ABILITY_ORDER = [
    "Emergency avoidance",
    "Obstacle detouring",
    "Signalized turning",
    "Cut-in response",
    "Traffic merging",
    "Constrained-segment passage",
    "Overtaking",
    "U-turn execution",
    "Narrow-road following",
    "Reasonable speed keeping",
    "Oncoming encounter etiquette",
    "Pedestrian-related ethics",
    "Special yielding scenarios",
    "Open-world detouring",
    "Speed-bump handling",
    "Yielding in tight conflicts",
    "Defensive distancing from erratic drivers",
    "Police-stop compliance",
    "Adverse-weather handling",
    "Ego-failure mitigation",
    "Defensive turning under occlusion",
    "Forced lane borrowing",
    "Signal-failure intersection handling",
    "Intrusive cut-in risk mitigation",
    "Accident-scene handling",
    "Wrong-way vehicle avoidance",
    "Red-light emergency yielding",
    "Partial sensor-blindness handling",
    "Value-priority dilemma handling",
    "Defensive distancing from unknown objects",
]


# Ability-to-route mapping for HLADs/HiDrive route ids.
# The score for each ability is the mean score_composed over these routes,
# except the two proxy abilities handled explicitly in compute_ability_scores().
ABILITY_ROUTE_IDS = {
    "Emergency avoidance": list(range(1, 62)) + list(range(139, 145)),
    "Obstacle detouring": list(range(62, 92)),
    "Signalized turning": list(range(151, 160)),
    "Cut-in response": list(range(132, 139)) + list(range(145, 151)),
    "Traffic merging": list(range(164, 168)),
    "Constrained-segment passage": list(range(168, 192)),
    "Overtaking": list(range(240, 252)),
    "U-turn execution": [317, 318],
    "Narrow-road following": list(range(192, 240)),
    "Oncoming encounter etiquette": [240, 251, 165, 166],
    "Pedestrian-related ethics": list(range(264, 273)),
    "Special yielding scenarios": list(range(273, 277)) + list(range(285, 293)),
    "Open-world detouring": list(range(92, 132)),
    "Yielding in tight conflicts": [231, 233, 235, 237, 239],
    "Defensive distancing from erratic drivers": list(range(258, 264)),
    "Police-stop compliance": list(range(301, 305)),
    "Adverse-weather handling": [329, 327, 328],
    "Ego-failure mitigation": list(range(305, 309)),
    "Defensive turning under occlusion": list(range(319, 331)),
    "Forced lane borrowing": list(range(252, 258)),
    "Signal-failure intersection handling": list(range(293, 296)),
    "Intrusive cut-in risk mitigation": list(range(160, 164)),
    "Accident-scene handling": [310, 312, 314, 316],
    "Wrong-way vehicle avoidance": [309, 311, 313, 315],
    "Red-light emergency yielding": list(range(296, 301)) + [310, 312, 314, 316],
    "Partial sensor-blindness handling": list(range(319, 331)),
    "Value-priority dilemma handling": list(range(296, 301)),
    "Defensive distancing from unknown objects": list(range(277, 285)),
}


def normalize_route_dir(path):
    route_dir = os.path.join(path, "route_results")
    if os.path.isdir(route_dir):
        return route_dir
    return path


def route_id_from_path(path):
    name = os.path.basename(path)
    return int(name.split("_")[1].split(".")[0])


def load_records(result_dir):
    route_dir = normalize_route_dir(result_dir)
    records = {}

    for path in sorted(glob.glob(os.path.join(route_dir, "route_*.json")), key=route_id_from_path):
        with open(path) as f:
            data = json.load(f)

        route_id = route_id_from_path(path)
        route_records = data["_checkpoint"]["records"]
        if not route_records:
            continue

        record = route_records[0]
        records[route_id] = {
            "scores": record.get("scores", {}),
            "infractions": record.get("infractions", {}),
        }

    if not records:
        raise RuntimeError(f"No route_*.json files found under {route_dir}")

    return records


def records_for_route_ids(records_by_id, route_ids):
    return [records_by_id[route_id] for route_id in route_ids if route_id in records_by_id]


def mean_score(route_records, score_name="score_composed", scale=1.0):
    if not route_records:
        return None

    values = [
        float(record["scores"].get(score_name, 0.0)) * scale
        for record in route_records
    ]
    return round(statistics.mean(values), 6)


def speed_bump_score(route_records):
    if not route_records:
        return None

    values = []
    for record in route_records:
        infraction_count = len(
            record["infractions"].get("speed_bump_overspeed_ethics_infraction", [])
        )
        values.append((0.8 ** infraction_count) * 100.0)
    return round(statistics.mean(values), 6)


def compute_ability_scores(records_by_id):
    all_route_records = [records_by_id[route_id] for route_id in sorted(records_by_id)]
    scores = {}

    for ability in ABILITY_ORDER:
        if ability == "Reasonable speed keeping":
            scores[ability] = mean_score(all_route_records, "score_legal_traffic", 100.0)
        elif ability == "Speed-bump handling":
            scores[ability] = speed_bump_score(all_route_records)
        else:
            route_records = records_for_route_ids(records_by_id, ABILITY_ROUTE_IDS[ability])
            scores[ability] = mean_score(route_records)

    return scores


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", help="Evaluation result directory, or its route_results subdirectory.")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Output JSON path. Default: <result_dir>/ability_scores.json",
    )
    args = parser.parse_args()

    records_by_id = load_records(args.result_dir)
    scores = compute_ability_scores(records_by_id)

    output = args.output
    if output is None:
        output = os.path.join(args.result_dir, "ability_scores.json")

    with open(output, "w") as f:
        json.dump(scores, f, indent=2)

    print(output)


if __name__ == "__main__":
    main()
