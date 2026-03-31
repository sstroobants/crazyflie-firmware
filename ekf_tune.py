#!/usr/bin/env python3
"""
Hyperparameter tuning for the Crazyflie EKF replay.

Uses Optuna (Bayesian TPE sampler) to search over the tunable EKF parameters
and minimise body-frame velocity RMSE against locSrv ground truth.

Usage:
    python ekf_tune.py                        # default: 200 trials, both CSVs
    python ekf_tune.py --n-trials 500
    python ekf_tune.py --csv flight_sine.csv   # single dataset only
    python ekf_tune.py --show-best             # print best params & re-run with plot

Does NOT modify ekf_replay.py.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import os
import warnings
from pathlib import Path

import numpy as np
import optuna
from optuna.samplers import TPESampler

from ekf_replay import EKFParams, run_ekf

warnings.filterwarnings("ignore", category=FutureWarning)

# ---------------------------------------------------------------------------
# Objective
# ---------------------------------------------------------------------------

CSV_FILES = ["flight_sine.csv", "flight_square.csv"]


def _body_vel_rmse(results) -> dict[str, float]:
    """Compute body-frame velocity RMSE from a run_ekf result DataFrame."""
    def rmse(a, b):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            d = a - b
            return float(np.sqrt(np.nanmean(d * d)))

    return {
        "vx_b": rmse(results["vx_b"].to_numpy(), results["ls_vx_b"].to_numpy()),
        "vy_b": rmse(results["vy_b"].to_numpy(), results["ls_vy_b"].to_numpy()),
        "vz_b": rmse(results["vz_b"].to_numpy(), results["ls_vz_b"].to_numpy()),
    }


def build_params(trial: optuna.Trial) -> EKFParams:
    """Sample EKF parameters from the search space.

    Only searches parameters that meaningfully affect steady-state
    body-frame velocity estimation.  Initial covariances and near-zero
    process noise terms are kept at defaults.
    """

    # --- Process noise (how much EKF trusts IMU integration) ---
    proc_noise_acc_xy = trial.suggest_float("proc_noise_acc_xy", 0.1, 2.5, log=True)     # default 0.5
    proc_noise_acc_z  = trial.suggest_float("proc_noise_acc_z",  0.2, 5.0, log=True)     # default 1.0

    # --- Gyro measurement noise (attitude correction gain) ---
    meas_noise_gyro_rp  = trial.suggest_float("meas_noise_gyro_rp",  0.02, 0.5, log=True)  # default 0.1
    meas_noise_gyro_yaw = trial.suggest_float("meas_noise_gyro_yaw", 0.02, 0.5, log=True)  # default 0.1

    # --- Drag coefficients (body velocity prediction model) ---
    drag_x = trial.suggest_float("drag_x", 1.5, 7.0)   # default 3.8
    drag_y = trial.suggest_float("drag_y", 1.5, 7.0)   # default 3.8
    drag_z = trial.suggest_float("drag_z", 0.05, 1.0)  # default 0.3

    # --- Sensor noise (how much EKF trusts flow & ToF) ---
    flow_std_fixed = trial.suggest_float("flow_std_fixed", 0.5, 8.0, log=True)       # default 2.0
    tof_exp_std_a  = trial.suggest_float("tof_exp_std_a", 0.0005, 0.02, log=True)    # default 0.0025

    return EKFParams(
        proc_noise_acc_xy=proc_noise_acc_xy,
        proc_noise_acc_z=proc_noise_acc_z,
        meas_noise_gyro_rp=meas_noise_gyro_rp,
        meas_noise_gyro_yaw=meas_noise_gyro_yaw,
        drag_x=drag_x,
        drag_y=drag_y,
        drag_z=drag_z,
        flow_std_fixed=flow_std_fixed,
        tof_exp_std_a=tof_exp_std_a,
        cop=np.zeros(3),
        flowdeck_pos=np.zeros(3),
    )

    return EKFParams(
        proc_noise_acc_xy=proc_noise_acc_xy,
        proc_noise_acc_z=proc_noise_acc_z,
        proc_noise_vel=proc_noise_vel,
        proc_noise_pos=proc_noise_pos,
        proc_noise_att=proc_noise_att,
        meas_noise_gyro_rp=meas_noise_gyro_rp,
        meas_noise_gyro_yaw=meas_noise_gyro_yaw,
        drag_x=drag_x,
        drag_y=drag_y,
        drag_z=drag_z,
        flow_std_fixed=flow_std_fixed,
        tof_exp_std_a=tof_exp_std_a,
        std_xy0=std_xy0,
        std_z0=std_z0,
        std_vel0=std_vel0,
        std_att_rp0=std_att_rp0,
        std_att_yaw0=std_att_yaw0,
        cop=np.zeros(3),
        flowdeck_pos=np.zeros(3),
    )


def objective(trial: optuna.Trial, csv_files: list[str]) -> float:
    """Run EKF on each CSV and return mean body-frame velocity RMSE."""
    params = build_params(trial)
    total_rmse = 0.0
    n = 0
    for csv_path in csv_files:
        if not Path(csv_path).exists():
            continue
        try:
            with open(os.devnull, "w") as devnull, contextlib.redirect_stdout(devnull):
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    results = run_ekf(csv_path, params=params)
        except Exception:
            return float("inf")

        if "ls_vx_b" not in results.columns:
            return float("inf")

        metrics = _body_vel_rmse(results)
        total_rmse += metrics["vx_b"] + metrics["vy_b"] + metrics["vz_b"]
        n += 1

    if n == 0:
        return float("inf")

    return total_rmse / n


# ---------------------------------------------------------------------------
# Pretty printing
# ---------------------------------------------------------------------------

def print_best(study: optuna.Study, csv_files: list[str]):
    """Print the best parameters and per-dataset RMSE breakdown."""
    best = study.best_trial
    print("\n" + "=" * 60)
    print(f"  BEST TRIAL #{best.number}  —  objective = {best.value:.4f}")
    print("=" * 60)

    # Print params grouped
    groups = {
        "Process noise": ["proc_noise_acc_xy", "proc_noise_acc_z"],
        "Meas noise":    ["meas_noise_gyro_rp", "meas_noise_gyro_yaw"],
        "Drag":          ["drag_x", "drag_y", "drag_z"],
        "Flow":          ["flow_std_fixed"],
        "ToF":           ["tof_exp_std_a"],
    }
    for group_name, keys in groups.items():
        print(f"\n  {group_name}:")
        for k in keys:
            if k in best.params:
                print(f"    {k:25s} = {best.params[k]:.6g}")

    # Per-dataset breakdown
    params = build_params_from_dict(best.params)
    print("\n  Per-dataset RMSE:")
    for csv_path in csv_files:
        if not Path(csv_path).exists():
            continue
        results = run_ekf(csv_path, params=params)
        m = _body_vel_rmse(results)
        print(f"    {csv_path:30s}  vx_b={m['vx_b']:.4f}  vy_b={m['vy_b']:.4f}  vz_b={m['vz_b']:.4f}")
    print("=" * 60 + "\n")

    # Print as copy-pasteable EKFParams constructor
    print("Copy-paste for ekf_replay.py:\n")
    print("    params = EKFParams(")
    for k, v in best.params.items():
        print(f"        {k}={v:.6g},")
    print("        cop=np.zeros(3),")
    print("        flowdeck_pos=np.zeros(3),")
    print("    )\n")


def build_params_from_dict(d: dict) -> EKFParams:
    """Reconstruct EKFParams from a flat parameter dict (e.g. best_trial.params)."""
    return EKFParams(
        proc_noise_acc_xy=d["proc_noise_acc_xy"],
        proc_noise_acc_z=d["proc_noise_acc_z"],
        meas_noise_gyro_rp=d["meas_noise_gyro_rp"],
        meas_noise_gyro_yaw=d["meas_noise_gyro_yaw"],
        drag_x=d["drag_x"],
        drag_y=d["drag_y"],
        drag_z=d["drag_z"],
        flow_std_fixed=d["flow_std_fixed"],
        tof_exp_std_a=d["tof_exp_std_a"],
        cop=np.zeros(3),
        flowdeck_pos=np.zeros(3),
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Tune EKF replay hyperparameters")
    parser.add_argument("--n-trials", type=int, default=200,
                        help="Number of Optuna trials (default: 200)")
    parser.add_argument("--csv", type=str, nargs="+", default=CSV_FILES,
                        help="CSV file(s) to use for tuning")
    parser.add_argument("--study-name", type=str, default="ekf_tune",
                        help="Optuna study name (for DB storage)")
    parser.add_argument("--db", type=str, default=None,
                        help="Optuna storage URL, e.g. sqlite:///ekf_tune.db")
    parser.add_argument("--show-best", action="store_true",
                        help="Load existing study, print best, and run with plot")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    storage = args.db

    if args.show_best:
        study = optuna.load_study(study_name=args.study_name, storage=storage)
        print_best(study, args.csv)

        # Re-run best on each CSV with plots
        from ekf_replay import plot_results
        best_params = build_params_from_dict(study.best_trial.params)
        for csv_path in args.csv:
            if Path(csv_path).exists():
                print(f"\nRunning best params on {csv_path} ...")
                results = run_ekf(csv_path, params=best_params)
                plot_results(results)
        return

    # Suppress per-trial EKF prints
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    sampler = TPESampler(seed=args.seed, n_startup_trials=10)
    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage,
        direction="minimize",
        sampler=sampler,
        load_if_exists=True,
    )

    # Add current defaults as the first trial so we have a baseline
    default_dict = {
        "proc_noise_acc_xy": 0.5,
        "proc_noise_acc_z":  1.0,
        "meas_noise_gyro_rp":  0.1,
        "meas_noise_gyro_yaw": 0.1,
        "drag_x": 3.8,
        "drag_y": 3.8,
        "drag_z": 0.3,
        "flow_std_fixed": 2.0,
        "tof_exp_std_a": 0.0025,
    }
    # Only enqueue if study is fresh
    if len(study.trials) == 0:
        study.enqueue_trial(default_dict)

    csv_files = [f for f in args.csv if Path(f).exists()]
    if not csv_files:
        print("Error: no CSV files found.")
        return

    print(f"Tuning EKF on: {csv_files}")
    print(f"Running {args.n_trials} trials (TPE sampler, seed={args.seed})")
    print(f"Objective: mean body-frame velocity RMSE (vx_b + vy_b + vz_b)\n")

    # Progress callback
    best_so_far = float("inf")
    def _callback(study, trial):
        nonlocal best_so_far
        val_str = f"{trial.value:.4f}" if trial.value is not None and np.isfinite(trial.value) else "FAILED"
        if trial.value is not None and trial.value < best_so_far:
            best_so_far = trial.value
            print(f"  Trial {trial.number:4d}  obj={val_str}  ★ new best")
        else:
            print(f"  Trial {trial.number:4d}  obj={val_str}")

    study.optimize(
        lambda trial: objective(trial, csv_files),
        n_trials=args.n_trials,
        callbacks=[_callback],
        show_progress_bar=True,
    )

    print_best(study, csv_files)


if __name__ == "__main__":
    main()
