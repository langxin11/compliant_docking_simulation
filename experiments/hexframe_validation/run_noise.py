"""HexFrame P2: measurement-noise A/B between the two seating gate variants.

Both gate variants consume the same noisy axial-force measurement (Gaussian,
seeded); they differ only in how the seating decision consumes it:

- raw_strict (historical, formal default): raw measured force thresholds,
  any single out-of-window step resets the 0.5 s dwell.
- filtered_debounce (candidate): decisions on the existing 20 ms low-pass
  channel, blips up to 10 ms do not reset the dwell; audit re-checks the
  trace statistically (>=98% in-window, no invalid run > 10 ms).

A clean filtered_debounce run (sigma=0) first re-anchors the candidate
against the recorded 1 ms baseline. Geometry, gains, phase durations, force
window values and every other gate stay unchanged.
"""
from __future__ import annotations

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import run_grid

from compliant_docking.scene import REPO_ROOT

BASELINE = run_grid.BASELINE
DEFAULT_OUT = REPO_ROOT / "runs/hexframe_noise_20261003"
SIGMAS = (0.02, 0.05)
SEEDS = (1, 2, 3)
GATES = ("raw_strict", "filtered_debounce")
NOMINAL = ("nominal", "pick", (0., 0., 0.))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--jobs", type=int, choices=range(1, 7), default=3)
    args = parser.parse_args()
    out_root = args.out.resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()

    clean_path = out_root/"gd_clean"
    if (clean_path/"validation.json").exists() and (clean_path/"audit.json").exists():
        v = json.loads((clean_path/"validation.json").read_text())
        clean = dict(label="gd_clean", status=v["status"],
                     audit=json.loads((clean_path/"audit.json").read_text())["status"],
                     first_contact_s=v["first_contact_s"], lock_time_s=v["lock_time_s"],
                     peak_axial_force_n=v["peak_axial_force_n"])
        print("CLEAN reused", clean["status"], "audit:", clean["audit"],
              "lock", clean["lock_time_s"], flush=True)
    else:
        clean = run_grid.run_one(("gd_clean",)+NOMINAL[1:], out_root, gate="filtered_debounce", sigma=0.)
        print("CLEAN", clean["status"], "audit:", clean["audit"],
              "lock", clean["lock_time_s"], flush=True)

    jobs = [(f"{'gd' if gate.startswith('filtered') else 'rs'}_s{int(s*1000)}_seed{seed}",
             gate, s, seed)
            for s in SIGMAS for seed in SEEDS for gate in GATES]
    (out_root/"noise_plan.json").write_text(json.dumps(dict(
        jobs=[dict(label=job[0], gate=job[1], sigma_n=job[2], seed=job[3]) for job in jobs],
        baseline=str(BASELINE.relative_to(REPO_ROOT)), measurement="axial force, seeded Gaussian",
        force_window_and_gates_unchanged=True, timestep_s=1e-3), indent=1)+"\n")
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as executor:
        futures = {executor.submit(run_grid.run_one, (label,)+NOMINAL[1:], out_root,
                                   gate=gate, sigma=sigma, seed=seed): label
                   for label, gate, sigma, seed in jobs}
        for future in as_completed(futures):
            record = future.result()
            results.append(record)
            print("NOISE", record["label"], record["status"], "audit:", record.get("audit"),
                  "first_contact", record["first_contact_s"], "lock", record["lock_time_s"], flush=True)
    order = {label: i for i, (label, *_ ) in enumerate(jobs)}
    results.sort(key=lambda r: order[r["label"]])
    baseline = json.loads((BASELINE/"validation.json").read_text())
    by_pair = {}
    for sigma in SIGMAS:
        for seed in SEEDS:
            pair = {}
            for record in results:
                if record["force_noise_sigma_n"] == sigma and record["noise_seed"] == seed:
                    key = "filtered_debounce" if record["seating_gate"] == "filtered_debounce" else "raw_strict"
                    pair[key] = dict(status=record["status"],
                                     first_contact_s=record["first_contact_s"],
                                     lock_time_s=record["lock_time_s"],
                                     peak_axial_force_n=record["peak_axial_force_n"])
            by_pair[f"sigma{int(sigma*1000)}_seed{seed}"] = pair
    summary = dict(clean_filtered_debounce=dict(status=clean["status"], audit=clean["audit"],
                                                lock_time_s=clean["lock_time_s"],
                                                baseline_lock_time_s=baseline["lock_time_s"]),
                   pairs=by_pair, wall_seconds=time.monotonic()-start)
    (out_root/"noise_summary.json").write_text(json.dumps(summary, indent=1)+"\n")
    print("NOISE DONE", json.dumps({k: {g: v[g]["lock_time_s"] for g in v} for k, v in by_pair.items()}),
          flush=True)


if __name__ == "__main__":
    main()
