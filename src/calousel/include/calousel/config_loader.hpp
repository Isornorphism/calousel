#ifndef CALOUSEL__CONFIG_LOADER_HPP_
#define CALOUSEL__CONFIG_LOADER_HPP_

#include <string>
#include <vector>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace calousel {

/** Per-camera extract params for config. Mirrors CamExtractParams in keyframe_extraction_impl. */
struct CamParams {
  int frame_window_size = 5;
  double angle_threshold = 3.0;
  bool rolling_shutter_compensation = false;
  int fps = 30;
  bool debug = false;
};

/** Unified config for keyframe extraction + extrinsic optimization. All settings from YAML. */
struct Config {
  std::string config_yaml_path;

  std::string bag_path;
  std::string keyframe_result_dir;
  std::string extrinsic_result_dir;

  std::vector<std::string> image_topics;
  std::vector<std::string> intrinsic_yamls;

  /** Per-camera params from camera.camN section. */
  std::vector<CamParams> cam_params;

  bool use_weight = false;
  bool fix_reference_camera_z_to_zero = false;
  bool optimize_reprojection_error = false;

  std::optional<int> board_n_x;
  std::optional<int> board_n_y;
  std::optional<double> board_radius;
  std::optional<double> board_distance;
  std::optional<bool> board_asymmetric;
};

inline bool load_config_from_yaml(const std::string& path, Config& out) {
  try {
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node n = root;
    if (root["calousel_pipeline"]) n = root["calousel_pipeline"];
    if (n["ros__parameters"]) n = n["ros__parameters"];

    if (n["bag_path"]) out.bag_path = n["bag_path"].as<std::string>();
    if (n["keyframe_result_dir"]) out.keyframe_result_dir = n["keyframe_result_dir"].as<std::string>();
    if (n["extrinsic_result_dir"]) out.extrinsic_result_dir = n["extrinsic_result_dir"].as<std::string>();

    // New format: camera.cam0, cam1, cam2... and board
    YAML::Node camera = n["camera"];
    YAML::Node board = n["board"];
    if (camera && camera.IsMap()) {
      out.image_topics.clear();
      out.intrinsic_yamls.clear();
      for (int i = 0; i < 32; i++) {
        std::string key = "cam" + std::to_string(i);
        if (!camera[key]) break;
        YAML::Node cam_node = camera[key];
        if (cam_node["topic"]) {
          out.image_topics.push_back(cam_node["topic"].as<std::string>());
        }
        if (cam_node["intrinsic_yaml"]) {
          out.intrinsic_yamls.push_back(cam_node["intrinsic_yaml"].as<std::string>());
        }
        CamParams cp;
        if (cam_node["frame_window_size"]) cp.frame_window_size = cam_node["frame_window_size"].as<int>();
        if (cam_node["angle_threshold"]) cp.angle_threshold = cam_node["angle_threshold"].as<double>();
        if (cam_node["rolling_shutter_compensation"]) cp.rolling_shutter_compensation = cam_node["rolling_shutter_compensation"].as<bool>();
        if (cam_node["fps"]) cp.fps = cam_node["fps"].as<int>();
        if (cam_node["debug"]) cp.debug = cam_node["debug"].as<bool>();
        out.cam_params.push_back(cp);
      }
    }
    if (board && board.IsMap()) {
      if (board["n_x"]) out.board_n_x = board["n_x"].as<int>();
      if (board["n_y"]) out.board_n_y = board["n_y"].as<int>();
      if (board["radius"]) out.board_radius = board["radius"].as<double>();
      if (board["distance"]) out.board_distance = board["distance"].as<double>();
      if (board["asymmetric"]) out.board_asymmetric = board["asymmetric"].as<bool>();
      else if (board["assymmetric"]) out.board_asymmetric = board["assymmetric"].as<bool>();
    }

    if (n["use_weight"]) out.use_weight = n["use_weight"].as<bool>();
    if (n["fix_reference_camera_z_to_zero"]) {
      out.fix_reference_camera_z_to_zero = n["fix_reference_camera_z_to_zero"].as<bool>();
    }
    if (n["optimize_reprojection_error"]) out.optimize_reprojection_error = n["optimize_reprojection_error"].as<bool>();

    out.config_yaml_path = path;
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace calousel

#endif
