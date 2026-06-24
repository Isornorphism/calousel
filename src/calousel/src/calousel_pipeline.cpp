/**
 * Unified calibration pipeline: keyframe extraction then extrinsic optimization.
 * All settings from YAML config via --config.
 */

#include <iostream>
#include <string>

#include <calousel/config_loader.hpp>
#include <calousel/keyframe_extraction_impl.hpp>
#include <calousel/logging.hpp>
#include <calousel/extrinsic_optimization_impl.hpp>

namespace {

void apply_config(const calousel::Config& c,
                  calousel::ExtractArgs& ext,
                  calousel::ExtrinsicOptimizationArgs& opt) {
  ext.bag_path = c.bag_path;
  ext.keyframe_result_dir = c.keyframe_result_dir;
  ext.image_topics = c.image_topics;
  ext.intrinsic_yamls = c.intrinsic_yamls;

  ext.cam_params.clear();
  for (const auto& p : c.cam_params) {
    calousel::CamExtractParams ep;
    ep.frame_window_size = p.frame_window_size;
    ep.angle_threshold = p.angle_threshold;
    ep.rolling_shutter_compensation = p.rolling_shutter_compensation;
    ep.fps = p.fps;
    ep.debug = p.debug;
    ext.cam_params.push_back(ep);
  }

  ext.board_n_x = c.board_n_x;
  ext.board_n_y = c.board_n_y;
  ext.board_radius = c.board_radius;
  ext.board_distance = c.board_distance;
  ext.board_asymmetric = c.board_asymmetric;

  opt.keyframe_result_dir = c.keyframe_result_dir;
  opt.extrinsic_result_dir = c.extrinsic_result_dir.empty() ? opt.keyframe_result_dir : c.extrinsic_result_dir;
  opt.use_weight = c.use_weight;
  opt.fix_reference_camera_z_to_zero = c.fix_reference_camera_z_to_zero;
  opt.optimize_reprojection_error = c.optimize_reprojection_error;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || std::string(argv[1]) != "--config") {
    std::cerr << "Usage: calousel_pipeline --config <yaml_path>\n"
              << "  All settings are read from the YAML file.\n";
    return 2;
  }

  const std::string config_path = argv[2];
  calousel::Config config;
  if (!calousel::load_config_from_yaml(config_path, config)) {
    std::cerr << "Failed to load config: " << config_path << "\n";
    return 2;
  }

  calousel::ExtractArgs extract_args;
  calousel::ExtrinsicOptimizationArgs optimize_args;
  apply_config(config, extract_args, optimize_args);

  if (extract_args.bag_path.empty() || extract_args.keyframe_result_dir.empty() ||
      extract_args.intrinsic_yamls.empty() || extract_args.image_topics.empty()) {
    std::cerr << "Config must define bag_path, keyframe_result_dir, and camera section.\n";
    return 2;
  }
  if (extract_args.intrinsic_yamls.size() != extract_args.image_topics.size()) {
    std::cerr << "Number of cameras must match number of image_topics.\n";
    return 2;
  }

  const auto log = calousel::stderr_logger();

  if (!calousel::run_keyframe_extraction(extract_args, log)) {
    return 1;
  }

  optimize_args.keyframe_result_dir = extract_args.keyframe_result_dir;
  if (optimize_args.extrinsic_result_dir.empty()) {
    optimize_args.extrinsic_result_dir = optimize_args.keyframe_result_dir;
  }

  if (!calousel::run_extrinsic_optimization(optimize_args, log)) {
    return 1;
  }
  return 0;
}
