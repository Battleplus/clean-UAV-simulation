#!/usr/bin/env python3
"""Create a flyable scenario by fitting opposite-pitch props to motors 4/5/7/8.

The authoritative as-installed thrust-sign evidence remains untouched.
"""

from __future__ import annotations

import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / "config" / "my_drone_v2_cad.json"
OUTPUT = PACKAGE / "config" / "my_drone_v2_cad_flight_pitch_corrected.json"
FLIPPED = {4, 5, 7, 8}


def main() -> None:
    config = json.loads(SOURCE.read_text(encoding="utf-8"))
    config["description"] = (
        "Flight scenario derived from the authoritative CAD config by fitting "
        "opposite-pitch propellers to motors 4, 5, 7 and 8. CAD positions, "
        "shaft lines, tilt, mass and arm dynamics are unchanged."
    )
    config["scenario"] = "opposite_pitch_props_on_4_5_7_8"
    config["authoritative_source"] = "config/my_drone_v2_cad.json"
    for rotor in config["rotors"]:
        motor = int(rotor["motor"])
        if motor in FLIPPED:
            rotor["axis_body"] = [-float(value) for value in rotor["axis_body"]]
            rotor["thrust_sign_status"] = "FLIGHT SCENARIO: opposite-pitch prop makes vertical thrust upward"
            rotor["vertical_thrust_direction"] = "up"
    maximum = float(config["maximum_thrust_n"])
    vertical = maximum * sum(-float(r["axis_body"][2]) for r in config["rotors"])
    mass = float(config["estimated_all_up_mass_kg"])
    config["maximum_vertical_force_n"] = vertical
    config["maximum_supported_mass_kg"] = vertical / 9.80665
    config["estimated_vertical_thrust_to_weight"] = vertical / (mass * 9.80665)
    config["flight_feasibility_nonreversible"] = "FEASIBLE"
    OUTPUT.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(OUTPUT)
    print(f"vertical_force={vertical:.6f} N, T/W={config['estimated_vertical_thrust_to_weight']:.6f}")


if __name__ == "__main__":
    main()
