"""Evidence-only QEMU A/B for JSON-owned FreeRTOS Monitor stack scan policy.

Both binaries must be compiled from exactly the SAME commit, ARM toolchain,
FreeRTOS flags and JSON defaults. Only MONITOR_STACK_OPT differs. Reject
missing/overflow reports. Runtime jitter is NOT hardware WCET.
"""
import argparse
import json
from pathlib import Path


def measure(baseline: dict, optimized: dict, baseline_timing: dict,
            optimized_timing: dict) -> dict:
    for name, report in (("base", baseline), ("optimized", optimized)):
        if report.get("overflow_count") != 0 or not (
                3 <= report.get("observed_tasks", 0) <= 16):
            raise ValueError(name+" CPU trace incomplete or overflowed")
        if report.get("elapsed_seconds", 0) < 1:
            raise ValueError(name+" monitoring observation too short")
    def by_name(r): return {t["name"]: t for t in r["tasks"]}
    b=by_name(baseline); o=by_name(optimized)
    for task in ("Monitor","Control","Trajectory"):
        if task not in b or task not in o:
            raise ValueError("Missing observed FreeRTOS task "+task)
    def by_task(t): return {x["task"]:x for x in t["tasks"]}
    bt=by_task(baseline_timing);ot=by_task(optimized_timing)
    out={"measurement":"same-commit QEMU Stage5 monitor scan optimization A/B",
         "scope":"Stage4 synthetic RTOS task mix; not real-hardware WCET",
         "baseline_elapsed_s":baseline["elapsed_seconds"],
         "optimized_elapsed_s":optimized["elapsed_seconds"],
         "tasks":{}}
    for task in ("Monitor","Control","Trajectory"):
        if task not in bt or task not in ot:
            raise ValueError("Missing GDB timing for "+task)
        out["tasks"][task]={
            "baseline_cpu_share_percent":b[task]["cpu_share_percent"],
            "optimized_cpu_share_percent":o[task]["cpu_share_percent"],
            "cpu_share_delta_percentage_points":round(
                o[task]["cpu_share_percent"]-b[task]["cpu_share_percent"],4),
            "baseline_p99_period_jitter_ms":bt[task]["p99_abs_period_jitter_ms"],
            "optimized_p99_period_jitter_ms":ot[task]["p99_abs_period_jitter_ms"],
            "baseline_p99_execution_ms":bt[task]["p99_processing_ms"],
            "optimized_p99_execution_ms":ot[task]["p99_processing_ms"],
            "baseline_minimum_stack_free_words":
                bt[task]["stack_minimum_free_words_observed"],
            "optimized_minimum_stack_free_words":
                ot[task]["stack_minimum_free_words_observed"],
        }
    out["monitor_cpu_share_measured_lower"]=(
        out["tasks"]["Monitor"]["cpu_share_delta_percentage_points"]<0)
    out["limitations"]=[
        "Short virtual QEMU synthetic low-load windows; not full-PyBullet CPU",
        "An O(stack) scan reduced from every monitor tick to once per JSON cadence",
        "Single A/B run is not statistically robust; do not claim universal speedup",
        "No real-hardware ISR latency or WCET established",
    ]
    return out


def main():
    a=argparse.ArgumentParser()
    for n in ("baseline_cpu","optimized_cpu",
              "baseline_timing","optimized_timing"):
        a.add_argument(n,type=Path)
    a.add_argument("--output",required=True,type=Path)
    args=a.parse_args()
    files=[json.loads(getattr(args,n).read_text(encoding="utf-8"))
           for n in ("baseline_cpu","optimized_cpu",
                     "baseline_timing","optimized_timing")]
    result=measure(*files)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result,indent=2))


if __name__=="__main__":main()
