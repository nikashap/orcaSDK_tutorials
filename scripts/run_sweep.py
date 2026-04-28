#!/usr/bin/env python
"""
run_sweep.py

Generate random hyperparameter configurations for a friction model sweep.

Produces:
  - One YAML config per trial in <sweep_dir>/configs/
  - A Slurm array job script at <sweep_dir>/submit_sweep.sh
  - A summary of all generated configs

Usage:
    python run_sweep.py --config sweep_config.yaml [--seed 42]

Then on the cluster:
    cd <sweep_dir> && sbatch submit_sweep.sh
"""

import argparse
import os
import math
import numpy as np
import yaml


def parse_args():
    p = argparse.ArgumentParser(description="Generate sweep configs")
    p.add_argument("--config", type=str, default="sweep_config.yaml")
    p.add_argument("--seed", type=int, default=42)
    return p.parse_args()


def sample_value(spec, rng):
    """Draw one sample from a search space specification."""
    stype = spec["type"]
    if stype == "categorical":
        idx = rng.randint(len(spec["values"]))
        val = spec["values"][idx]
        return val if not isinstance(val, list) else list(val)
    elif stype == "log_uniform":
        log_lo = math.log10(spec["low"])
        log_hi = math.log10(spec["high"])
        return float(10 ** rng.uniform(log_lo, log_hi))
    elif stype == "uniform":
        return float(rng.uniform(spec["low"], spec["high"]))
    else:
        raise ValueError(f"Unknown search space type: {stype}")


def generate_slurm_script(sweep_dir, n_trials, scripts_dir, scratch_dir,
                          dataset_source):
    """Generate a Slurm array job script for MIT Engaging."""
    return f"""#!/bin/bash
#SBATCH --job-name=friction_sweep
#SBATCH --array=1-{n_trials}%50
#SBATCH --output={sweep_dir}/logs/run_%a.out
#SBATCH --error={sweep_dir}/logs/run_%a.err
#SBATCH --time=00:30:00
#SBATCH --mem=4G
#SBATCH -c 2
#SBATCH -p mit_normal

# Environment setup (do NOT use conda init; load module instead)
module load miniforge
source activate orca_test_env

# Copy dataset to scratch (flash) if not already there.
# flock prevents race conditions when multiple array tasks start simultaneously.
SCRATCH_DATASET="{scratch_dir}/friction_dataset.npz"
mkdir -p "{scratch_dir}"
flock -n "{scratch_dir}/.copy.lock" -c '
    if [ ! -f "{scratch_dir}/friction_dataset.npz" ] || \\
       [ "{dataset_source}" -nt "{scratch_dir}/friction_dataset.npz" ]; then
        echo "Copying dataset to scratch..."
        cp "{dataset_source}" "{scratch_dir}/friction_dataset.npz"
    fi
'

# Run the trial matching this array task ID
RUN_ID=$(printf "%03d" $SLURM_ARRAY_TASK_ID)
CONFIG="{sweep_dir}/configs/run_${{RUN_ID}}.yaml"

cd {scripts_dir}
python train_model.py --config "$CONFIG" --engaging
"""


def generate_test_script(sweep_dir, scripts_dir, scratch_dir, dataset_source):
    """Generate a single-job test script for profiling resource usage."""
    return f"""#!/bin/bash
#SBATCH --job-name=friction_test
#SBATCH --output={sweep_dir}/logs/test_run.out
#SBATCH --error={sweep_dir}/logs/test_run.err
#SBATCH --time=00:30:00
#SBATCH --mem=4G
#SBATCH -c 2
#SBATCH -p mit_normal

# Environment setup
module load miniforge
source activate orca_test_env

# Copy dataset to scratch
SCRATCH_DATASET="{scratch_dir}/friction_dataset.npz"
mkdir -p "{scratch_dir}"
if [ ! -f "$SCRATCH_DATASET" ] || [ "{dataset_source}" -nt "$SCRATCH_DATASET" ]; then
    echo "Copying dataset to scratch..."
    cp "{dataset_source}" "$SCRATCH_DATASET"
fi

# Run one config for profiling
CONFIG="{sweep_dir}/configs/run_001.yaml"

cd {scripts_dir}
echo "=== Starting test run ==="
echo "Start time: $(date)"
python train_model.py --config "$CONFIG" --engaging
echo "End time: $(date)"
echo "=== Test run complete ==="
echo ""
echo "After this job finishes, check resource usage with:"
echo "  jobstats $SLURM_JOB_ID"
"""


def main():
    args = parse_args()
    rng = np.random.RandomState(args.seed)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.normpath(os.path.join(script_dir, ".."))

    with open(args.config, "r") as f:
        sweep_cfg = yaml.safe_load(f)

    base_config_path = os.path.join(script_dir, sweep_cfg["base_config"])
    with open(base_config_path, "r") as f:
        base_cfg = yaml.safe_load(f)

    n_trials = sweep_cfg["n_trials"]
    search_space = sweep_cfg["search_space"]

    # Resolve scratch and dataset paths
    scratch_dir = base_cfg.get("scratch_dir", "/home/nikashap/orcd/scratch/friction")
    output_dir_engaging = base_cfg.get("output_dir_engaging",
                                        base_cfg.get("output_dir", "models/friction"))
    dataset_source = os.path.join(output_dir_engaging, "friction_dataset.npz")
    scratch_dataset = os.path.join(scratch_dir, "friction_dataset.npz")

    # Resolve sweep directory
    sweep_dir = sweep_cfg["sweep_dir"]
    if not os.path.isabs(sweep_dir):
        sweep_dir = os.path.join(repo_root, sweep_dir)

    # Create a timestamped sweep subdirectory
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    sweep_run_dir = os.path.join(sweep_dir, f"sweep_{timestamp}")
    configs_dir = os.path.join(sweep_run_dir, "configs")
    logs_dir = os.path.join(sweep_run_dir, "logs")
    os.makedirs(configs_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)

    print(f"Generating {n_trials} configurations in {sweep_run_dir}")
    print(f"Search space:")
    for name, spec in search_space.items():
        print(f"  {name}: {spec}")

    generated = []
    for i in range(1, n_trials + 1):
        trial_cfg = dict(base_cfg)
        overrides = {}
        for name, spec in search_space.items():
            val = sample_value(spec, rng)
            trial_cfg[name] = val
            overrides[name] = val

        # Unique output directory per run; read dataset from scratch
        run_id = f"run_{i:03d}"
        run_output = os.path.join(sweep_run_dir, "results", run_id)
        trial_cfg["output_dir"] = run_output
        trial_cfg["output_dir_engaging"] = run_output
        trial_cfg["dataset_path"] = scratch_dataset

        config_path = os.path.join(configs_dir, f"{run_id}.yaml")
        with open(config_path, "w") as f:
            yaml.dump(trial_cfg, f, default_flow_style=False, sort_keys=False)

        generated.append({"run_id": run_id, **overrides})

    # Save sweep manifest
    manifest_path = os.path.join(sweep_run_dir, "sweep_manifest.yaml")
    manifest = {
        "n_trials": n_trials,
        "seed": args.seed,
        "search_space": search_space,
        "runs": generated,
    }
    with open(manifest_path, "w") as f:
        yaml.dump(manifest, f, default_flow_style=False, sort_keys=False)
    print(f"Manifest saved to {manifest_path}")

    # Generate Slurm scripts
    slurm_script = generate_slurm_script(
        sweep_run_dir, n_trials, script_dir, scratch_dir, dataset_source
    )
    slurm_path = os.path.join(sweep_run_dir, "submit_sweep.sh")
    with open(slurm_path, "w") as f:
        f.write(slurm_script)
    os.chmod(slurm_path, 0o755)
    print(f"Slurm array script saved to {slurm_path}")

    test_script = generate_test_script(
        sweep_run_dir, script_dir, scratch_dir, dataset_source
    )
    test_path = os.path.join(sweep_run_dir, "submit_test.sh")
    with open(test_path, "w") as f:
        f.write(test_script)
    os.chmod(test_path, 0o755)
    print(f"Slurm test script saved to {test_path}")

    print(f"\nDataset will be copied from:")
    print(f"  {dataset_source}")
    print(f"to scratch:")
    print(f"  {scratch_dataset}")

    print(f"\nStep 1 — Profile one run:")
    print(f"  sbatch {test_path}")
    print(f"  # After it finishes, check resource usage:")
    print(f"  jobstats <JOB_ID>")

    print(f"\nStep 2 — Submit full sweep (after adjusting resources):")
    print(f"  sbatch {slurm_path}")

    print(f"\nTo run one config locally (for testing):")
    print(f"  python train_model.py --config {configs_dir}/run_001.yaml")


if __name__ == "__main__":
    main()
