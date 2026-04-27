#!/usr/bin/env python
"""
collect_results.py

Scan a sweep directory for completed runs, rank by validation loss,
and print a summary table.

Usage:
    python collect_results.py --sweep-dir models/friction/sweeps/sweep_20260426_150000
    python collect_results.py --sweep-dir /orcd/data/.../sweeps/sweep_20260426_150000 --top 20
"""

import argparse
import os
import yaml


def parse_args():
    p = argparse.ArgumentParser(description="Collect sweep results")
    p.add_argument("--sweep-dir", type=str, required=True)
    p.add_argument("--top", type=int, default=10,
                   help="Show top N results (default 10)")
    return p.parse_args()


def main():
    args = parse_args()
    results_dir = os.path.join(args.sweep_dir, "results")

    if not os.path.isdir(results_dir):
        print(f"ERROR: No results directory at {results_dir}")
        return

    results = []
    run_dirs = sorted(os.listdir(results_dir))
    n_missing = 0

    for run_id in run_dirs:
        metrics_path = os.path.join(results_dir, run_id, "metrics.yaml")
        if not os.path.exists(metrics_path):
            n_missing += 1
            continue
        with open(metrics_path, "r") as f:
            m = yaml.safe_load(f)
        m["run_id"] = run_id
        results.append(m)

    if not results:
        print("No completed runs found.")
        return

    results.sort(key=lambda r: r["val_mse"])

    print(f"Completed: {len(results)} / {len(run_dirs)} "
          f"({n_missing} missing)")
    print()

    header = (f"{'Rank':>4}  {'Run':<8}  {'Var':>3}  {'Val MSE':>10}  "
              f"{'Test RMSE':>10}  {'LR':>9}  {'Hidden':>20}  "
              f"{'Act':>10}  {'BS':>5}  {'WD':>9}  {'Drop':>5}  {'Ep':>4}")
    print(header)
    print("-" * len(header))

    for rank, r in enumerate(results[:args.top], 1):
        hidden = str(r.get("hidden_dims", "?"))
        print(f"{rank:>4}  {r['run_id']:<8}  {r['model_variant']:>3}  "
              f"{r['val_mse']:>10.4f}  {r['test_rmse_mN']:>10.2f}  "
              f"{r['learning_rate']:>9.2e}  {hidden:>20}  "
              f"{r['activation']:>10}  {r['batch_size']:>5}  "
              f"{r['weight_decay']:>9.2e}  {r['dropout']:>5.2f}  "
              f"{r['best_epoch']:>4}")

    # Per-variant summary
    print(f"\n{'='*60}")
    print("Best result per model variant:")
    print(f"{'='*60}")
    seen = set()
    for r in results:
        v = r["model_variant"]
        if v in seen:
            continue
        seen.add(v)
        print(f"  Variant {v}: val_mse={r['val_mse']:.4f}, "
              f"test_rmse={r['test_rmse_mN']:.2f} mN, "
              f"run={r['run_id']}, hidden={r.get('hidden_dims')}, "
              f"act={r['activation']}, lr={r['learning_rate']:.2e}")

    # Save summary
    summary = {
        "n_completed": len(results),
        "n_total": len(run_dirs),
        "best_overall": results[0],
        "top_results": results[:args.top],
    }
    summary_path = os.path.join(args.sweep_dir, "sweep_summary.yaml")
    with open(summary_path, "w") as f:
        yaml.dump(summary, f, default_flow_style=False, sort_keys=False)
    print(f"\nSummary saved to {summary_path}")


if __name__ == "__main__":
    main()
