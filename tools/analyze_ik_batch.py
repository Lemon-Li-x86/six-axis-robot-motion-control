
from pathlib import Path
from collections import Counter
import csv
import struct

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from scipy.spatial.transform import Rotation

DATA_PATH = Path("build/ik_validation/ik_samples.bin")
CSV_PATH = Path("build/ik_validation/ik_results.csv")

SAMPLE_COUNT = 128
RECORD = struct.Struct("<6hiiI48h")

robot = rtb.models.DH.UR5()
robot.links[0].d = 0.089159

robot.base = SE3.Rz(np.pi)
robot.tool = SE3(np.array([
    [0, -1,  0, 0],
    [0,  0, -1, 0],
    [1,  0,  0, 0],
    [0,  0,  0, 1],
], dtype=float))

data = DATA_PATH.read_bytes()

if len(data) != SAMPLE_COUNT * RECORD.size:
    raise ValueError(
        f"Binary size mismatch: {len(data)}"
    )

status_counter = Counter()
solution_counter = Counter()

candidate_position_errors = []
candidate_rotation_errors = []

best_position_errors = []
best_rotation_errors = []

bad_candidates = []
unsolved_cases = []
csv_rows = []

POS_TOL_MM = 0.5
ROT_TOL_DEG = 0.15

for i in range(SAMPLE_COUNT):
    record = RECORD.unpack_from(
        data, i * RECORD.size
    )

    input_raw = np.array(record[:6], dtype=int)
    fk_status = record[6]
    ik_status = record[7]
    solution_count = record[8]

    solution_values = record[9:]

    status_counter[ik_status] += 1
    solution_counter[solution_count] += 1

    q_input = np.deg2rad(
        input_raw.astype(float) * 0.01
    )

    reference = robot.fkine(q_input).A.copy()
    reference[:3, 3] *= 1000.0

    if (
        fk_status != 0
        or ik_status != 0
        or not 1 <= solution_count <= 8
    ):
        unsolved_cases.append((
            i, input_raw.tolist(),
            fk_status, ik_status, solution_count
        ))

        csv_rows.append({
            "case": i,
            "solution": "",
            "fk_status": fk_status,
            "ik_status": ik_status,
            "solution_count": solution_count,
            "position_error_mm": "",
            "rotation_error_deg": "",
            "within_tolerance": False,
        })

        continue

    case_pos = []
    case_rot = []

    for s in range(solution_count):
        start = s * 6

        candidate_raw = np.array(
            solution_values[start:start + 6],
            dtype=int
        )

        q_candidate = np.deg2rad(
            candidate_raw.astype(float) * 0.01
        )

        reconstructed = (
            robot.fkine(q_candidate).A.copy()
        )

        reconstructed[:3, 3] *= 1000.0

        pos_error = np.linalg.norm(
            reconstructed[:3, 3]
            - reference[:3, 3]
        )

        relative_rotation = (
            reference[:3, :3].T
            @ reconstructed[:3, :3]
        )

        rot_error = np.rad2deg(
            Rotation.from_matrix(
                relative_rotation
            ).magnitude()
        )

        case_pos.append(pos_error)
        case_rot.append(rot_error)

        candidate_position_errors.append(pos_error)
        candidate_rotation_errors.append(rot_error)

        valid = (
            pos_error <= POS_TOL_MM
            and rot_error <= ROT_TOL_DEG
        )

        if not valid:
            bad_candidates.append((
                i, s,
                float(pos_error),
                float(rot_error),
                candidate_raw.tolist()
            ))

        csv_rows.append({
            "case": i,
            "solution": s,
            "fk_status": fk_status,
            "ik_status": ik_status,
            "solution_count": solution_count,
            "position_error_mm": float(pos_error),
            "rotation_error_deg": float(rot_error),
            "within_tolerance": valid,
        })

    best_position_errors.append(min(case_pos))
    best_rotation_errors.append(min(case_rot))

CSV_PATH.parent.mkdir(parents=True, exist_ok=True)

with CSV_PATH.open(
    "w", newline="", encoding="utf-8-sig"
) as file:
    writer = csv.DictWriter(
        file, fieldnames=list(csv_rows[0].keys())
    )
    writer.writeheader()
    writer.writerows(csv_rows)

print("\nCortex-M4 IK Diagnostic vs RTB")
print("=" * 48)

print("Total poses:", SAMPLE_COUNT)
print("Solved poses:", SAMPLE_COUNT - len(unsolved_cases))
print("Unsolved poses:", len(unsolved_cases))

print("\nIK status distribution:")
for status, count in sorted(status_counter.items()):
    print(f"  status {status}: {count}")

print("\nSolution-count distribution:")
for count, amount in sorted(solution_counter.items()):
    print(f"  {count} solutions: {amount} poses")

print("\nTotal returned candidates:",
      len(candidate_position_errors))

print("Candidates exceeding tolerance:",
      len(bad_candidates))

if candidate_position_errors:
    p = np.array(candidate_position_errors)
    r = np.array(candidate_rotation_errors)

    print("\nAll candidate position errors (mm):")
    print("  Mean:", f"{np.mean(p):.6f}")
    print("  Max: ", f"{np.max(p):.6f}")
    print("  RMSE:", f"{np.sqrt(np.mean(p**2)):.6f}")

    print("\nAll candidate rotation errors (deg):")
    print("  Mean:", f"{np.mean(r):.6f}")
    print("  Max: ", f"{np.max(r):.6f}")
    print("  RMSE:", f"{np.sqrt(np.mean(r**2)):.6f}")

if best_position_errors:
    print("\nBest-candidate position error:")
    print("  Mean:",
          f"{np.mean(best_position_errors):.6f} mm")

if best_rotation_errors:
    print("Best-candidate rotation error:")
    print("  Mean:",
          f"{np.mean(best_rotation_errors):.6f} deg")

print("\nFirst unsolved cases:")
for case in unsolved_cases[:10]:
    print(" ", case)

print("\nFirst inaccurate candidates:")
for case in bad_candidates[:10]:
    print(" ", case)

print("\nCSV saved:", CSV_PATH)

if not unsolved_cases and not bad_candidates:
    print("\nRESULT: ALL CASES WITHIN TOLERANCE")
else:
    print("\nRESULT: REVIEW REQUIRED")
