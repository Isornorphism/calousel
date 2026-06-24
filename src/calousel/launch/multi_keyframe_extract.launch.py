"""
Multi keyframe extract launch: run keyframe_extractor_cli on all .bag dirs in a dataset.

All settings from YAML config. For each bag, writes a temp config with bag_path and keyframe_result_dir set.
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


def _discover_bags(dataset_dir):
    root = pathlib.Path(dataset_dir)
    if not root.is_dir():
        return []
    bags = [p for p in root.iterdir() if p.is_dir() and p.name.endswith("adjusted2")]
    return sorted(bags, key=lambda p: p.name)


def _run_keyframe_extractor(context, *args, **kwargs):
    config_yaml = LaunchConfiguration("config_yaml").perform(context)
    dataset_dir_arg = _opt(LaunchConfiguration("dataset_dir").perform(context))
    keyframe_result_dir_arg = _opt(LaunchConfiguration("keyframe_result_dir").perform(context))

    config_path = (config_yaml or "").strip() or _config_path()
    try:
        cfg = _load_config(config_path)
    except Exception as e:
        raise RuntimeError(f"Failed to load config YAML {config_path}: {e}") from e

    dataset_dir = dataset_dir_arg or _opt(cfg.get("dataset_dir")) or ""
    keyframe_result_dir = keyframe_result_dir_arg or _opt(cfg.get("keyframe_result_dir")) or ""

    if not dataset_dir or not keyframe_result_dir:
        raise RuntimeError(
            "dataset_dir and keyframe_result_dir must be set in YAML (or via launch args dataset_dir, keyframe_result_dir). "
            f"Got dataset_dir={dataset_dir!r}, keyframe_result_dir={keyframe_result_dir!r}"
        )

    camera = cfg.get("camera")
    if not camera or not isinstance(camera, dict):
        raise RuntimeError("Config must define 'camera' section with cam0, cam1, ...")

    bag_dirs = _discover_bags(dataset_dir)
    if not bag_dirs:
        return [ExecuteProcess(cmd=["echo", "No .bag directories found; nothing to run."], output="screen")]

    out_root = pathlib.Path(keyframe_result_dir)
    extract_actions = []
    for b in bag_dirs:
        bag_path = str(b.resolve())
        bag_name = b.name
        out_name = bag_name.removesuffix(".bag") if bag_name.endswith(".bag") else bag_name
        bag_out = str((out_root / out_name).resolve())
        out_root.mkdir(parents=True, exist_ok=True)
        (out_root / out_name).mkdir(parents=True, exist_ok=True)

        cfg_run = cfg.copy()
        cfg_run["bag_path"] = bag_path
        cfg_run["keyframe_result_dir"] = bag_out

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tf:
            _save_config(cfg_run, tf.name)
            config_run_path = tf.name

        cmd = [
            "ros2", "run", "calousel", "keyframe_extractor_cli",
            "--config", config_run_path,
        ]
        exe = ExecuteProcess(
            cmd=cmd,
            output="screen",
            name=f"keyframe_extract_{bag_name}",
        )
        extract_actions.append(exe)

    run_actions = extract_actions

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
            "dataset_dir",
            default_value="",
            description="Dataset root. Overrides YAML when set.",
        ),
        DeclareLaunchArgument(
            "keyframe_result_dir",
            default_value="",
            description="Keyframe result root. Overrides YAML when set.",
        ),
        OpaqueFunction(function=_run_keyframe_extractor),
    ])
