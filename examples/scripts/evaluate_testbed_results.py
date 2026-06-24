#!/usr/bin/env python3
"""Evaluate the shipped testbed extrinsic calibration examples."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GT = REPO_ROOT / "examples/data/gt.yaml"
DEFAULT_RESULTS_DIR = REPO_ROOT / "examples/output/extrinsic_results"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize testbed extrinsic calibration rotation and translation results."
    )
    parser.add_argument(
        "--gt",
        type=Path,
        default=DEFAULT_GT,
        help=f"Ground-truth YAML file (default: {DEFAULT_GT.relative_to(REPO_ROOT)})",
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=DEFAULT_RESULTS_DIR,
        help=(
            "Directory containing case subdirectories with extrinsic_calibration_data.yaml "
            f"(default: {DEFAULT_RESULTS_DIR.relative_to(REPO_ROOT)})"
        ),
    )
    return parser.parse_args()


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def camera_index(name: str) -> int:
    if not name.startswith("cam"):
        raise ValueError(f"Camera name must look like cam0, cam1, ...; got {name}")
    return int(name[3:])


def result_path(results_dir: Path, case_name: str) -> Path:
    return results_dir / case_name / "extrinsic_calibration_data.yaml"


def load_extrinsic_result(results_dir: Path, case_name: str) -> dict:
    path = result_path(results_dir, case_name)
    if not path.exists():
        raise FileNotFoundError(f"Missing result file: {path}")
    return load_yaml(path)


def transform_from_result(result: dict, key: str) -> np.ndarray:
    try:
        transform = np.asarray(result["extrinsic_calibration"][key], dtype=float)
    except KeyError as exc:
        raise KeyError(f"Missing extrinsic_calibration.{key}") from exc

    if transform.shape != (4, 4):
        raise ValueError(f"extrinsic_calibration.{key} must be 4x4, got {transform.shape}")
    return transform


def rotation_z(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def rotation_y(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ]
    )


def rotation_x(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ]
    )


def rpy_to_matrix_deg(rpy_deg: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad(rpy_deg)
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


def matrix_to_rpy_deg(rotation: np.ndarray) -> np.ndarray:
    sy = math.hypot(rotation[0, 0], rotation[1, 0])
    singular = sy < 1e-9

    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return np.rad2deg([roll, pitch, yaw])


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cos_theta = (np.trace(rotation) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cos_theta, -1.0, 1.0))))


def turntable_aligned_offset(result: dict, cam0: int, cam1: int) -> np.ndarray:
    transform0 = transform_from_result(result, f"T_AC{cam0}_ref")
    transform1 = transform_from_result(result, f"T_AC{cam1}_ref")
    t0 = transform0[:3, 3]
    t1 = transform1[:3, 3]

    yaw = math.atan2(t0[1], t0[0])
    align_a_frame = rotation_z(-yaw)
    return align_a_frame @ t1 - align_a_frame @ t0


def turntable_aligned_relative_rotation(result: dict, cam0: int, cam1: int) -> np.ndarray:
    transform0 = transform_from_result(result, f"T_AC{cam0}_ref")
    transform1 = transform_from_result(result, f"T_AC{cam1}_ref")

    t0 = transform0[:3, 3]
    yaw = math.atan2(t0[1], t0[0])
    align_a_frame = rotation_z(-yaw)

    rotation0 = align_a_frame @ transform0[:3, :3]
    rotation1 = align_a_frame @ transform1[:3, :3]
    return rotation1 @ rotation0.T


def translation_gt_mm(gt: dict) -> np.ndarray:
    values = gt["translation_gt"]
    vec = np.array([values["x"], values["y"], values["z"]], dtype=float)
    units = gt.get("translation_units", gt.get("units", "mm")).lower()

    if units in ("m", "meter", "meters"):
        return vec * 1000.0
    if units in ("mm", "millimeter", "millimeters"):
        return vec
    raise ValueError(f"Unsupported translation unit: {units}")


def rotation_gt_deg(gt: dict, case_name: str) -> np.ndarray:
    values = gt["rotation_gt"][case_name]
    vec = np.array([values["roll"], values["pitch"], values["yaw"]], dtype=float)
    units = gt.get("rotation_units", "deg").lower()

    if units in ("rad", "radian", "radians"):
        return np.rad2deg(vec)
    if units in ("deg", "degree", "degrees"):
        return vec
    raise ValueError(f"Unsupported rotation unit: {units}")


def fmt_vec(vec: np.ndarray, unit: str) -> str:
    return f"[{vec[0]: .3f}, {vec[1]: .3f}, {vec[2]: .3f}] {unit}"


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def main() -> None:
    args = parse_args()
    gt = load_yaml(args.gt)

    reference_case = gt["reference_case"]
    target_case = gt.get("target_case", gt.get("displaced_case"))
    if target_case is None:
        raise KeyError("GT YAML must contain target_case or displaced_case")
    cam0, cam1 = [camera_index(name) for name in gt["camera_pair"]]
    cases = [reference_case, target_case]
    results = {case: load_extrinsic_result(args.results_dir, case) for case in cases}

    print("=============== Rotation (turntable-aligned A frame, ZYX RPY) ===============")
    for case in cases:
        result = results[case]
        rotation = turntable_aligned_relative_rotation(result, cam0, cam1)
        estimated_rpy = matrix_to_rpy_deg(rotation)
        gt_rpy = rotation_gt_deg(gt, case)
        rre = rotation_angle_deg(rotation @ rpy_to_matrix_deg(gt_rpy).T)

        print(f"{case}")
        print(f"  estimated RPY: {fmt_vec(estimated_rpy, 'deg')}")
        print(f"  gt RPY:        {fmt_vec(gt_rpy, 'deg')}")
        print(f"  RRE:           {rre: .3f} deg")

    print()
    print("=============== Translation (turntable-aligned A frame) ===============")
    offsets_mm = {}
    for case in cases:
        offsets_mm[case] = 1000.0 * turntable_aligned_offset(results[case], cam0, cam1)

    print(f"reference: {display_path(result_path(args.results_dir, reference_case))}")
    print(f"  t_c{cam0}c{cam1}:      {fmt_vec(offsets_mm[reference_case], 'mm')}")
    print()
    print(f"target: {display_path(result_path(args.results_dir, target_case))}")
    print(f"  t_c{cam0}c{cam1}:      {fmt_vec(offsets_mm[target_case], 'mm')}")

    delta_mm = offsets_mm[target_case] - offsets_mm[reference_case]
    gt_delta_mm = translation_gt_mm(gt)
    error_mm = delta_mm - gt_delta_mm

    print()
    print("comparison: target - reference")
    print(f"  delta:          {fmt_vec(delta_mm, 'mm')}")
    print(f"  expected delta: {fmt_vec(gt_delta_mm, 'mm')}")
    print(f"  error:          {fmt_vec(error_mm, 'mm')}")
    print(f"  error norm:     {np.linalg.norm(error_mm): .3f} mm")


if __name__ == "__main__":
    main()
