"""
Multi extrinsic optimization launch: run optimize_extrinsics_cli on all keyframe result dirs under a root.

All settings from YAML. For each keyframe result dir, writes a temp config with keyframe_result_dir and extrinsic_result_dir set.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
import pathlib
import tempfile
import yaml


def _config_path():
    p = pathlib.Path(__file__).resolve().parent.parent / "config" / "calousel_pipeline.yaml"
    return str(p)


def _load_config(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _save_config(cfg, path):
    with open(path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)


def _opt(s):
    if s is None:
        return None
    t = str(s).strip()
    return t if t else None


def _discover_keyframe_dirs(keyframe_results_root):
    root = pathlib.Path(keyframe_results_root)
    if not root.is_dir():
        return []
    found = []
    for p in root.iterdir():
        if not p.is_dir():
            continue
        meta = p / "metadata.yaml"
        cam0 = p / "cam0"
        if not meta.is_file() or not cam0.is_dir():
            continue
        found.append(p)
    return sorted(found, key=lambda x: x.name)


def _run_extrinsic_optimization(context, *args, **kwargs):
    config_yaml = LaunchConfiguration("config_yaml").perform(context)
    keyframe_results_root_arg = _opt(LaunchConfiguration("keyframe_results_root").perform(context))
    extrinsic_result_dir_arg = _opt(LaunchConfiguration("extrinsic_result_dir").perform(context))

    config_path = (config_yaml or "").strip() or _config_path()
    try:
        cfg = _load_config(config_path)
    except Exception as e:
        raise RuntimeError(f"Failed to load config YAML {config_path}: {e}") from e

    keyframe_results_root = keyframe_results_root_arg or _opt(cfg.get("keyframe_results_root")) or ""
    extrinsic_result_dir = extrinsic_result_dir_arg or _opt(cfg.get("extrinsic_result_dir")) or ""

    if not keyframe_results_root:
        raise RuntimeError("keyframe_results_root must be set in YAML (or via launch arg).")

    out_root = pathlib.Path(extrinsic_result_dir or keyframe_results_root)
    kf_dirs = _discover_keyframe_dirs(keyframe_results_root)

    if not kf_dirs:
        return [ExecuteProcess(cmd=["echo", "No keyframe directories found; nothing to run."], output="screen")]

    optimize_actions = []
    for kf in kf_dirs:
        if "inv" in kf.name:
            continue
        kf_path = str(kf.resolve())
        name = kf.name
        out_path = str((out_root / name).resolve())
        (out_root / name).mkdir(parents=True, exist_ok=True)

        cfg_run = cfg.copy()
        cfg_run["keyframe_result_dir"] = kf_path
        cfg_run["extrinsic_result_dir"] = out_path

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tf:
            _save_config(cfg_run, tf.name)
            config_run_path = tf.name

        cmd = [
            "ros2", "run", "calousel", "optimize_extrinsics_cli",
            "--config", config_run_path,
        ]
        exe = ExecuteProcess(cmd=cmd, output="screen", name=f"optimize_extrinsics_{name}")
        optimize_actions.append(exe)

    run_actions = list(optimize_actions)
    handlers = []
    for i in range(len(run_actions) - 1):
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=run_actions[i],
                    on_exit=[run_actions[i + 1]],
                )
            )
        )

    return [run_actions[0]] + handlers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_yaml",
            default_value=_config_path(),
            description="Path to calousel YAML config.",
        ),
        DeclareLaunchArgument(
            "keyframe_results_root",
            default_value="",
            description="Root with keyframe subdirs. Overrides YAML when set.",
        ),
        DeclareLaunchArgument(
            "extrinsic_result_dir",
            default_value="",
            description="Extrinsic output root. Overrides YAML when set.",
        ),
        OpaqueFunction(function=_run_extrinsic_optimization),
    ])
