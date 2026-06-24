#include <iostream>
#include <string>

#include <calousel/config_loader.hpp>
#include <calousel/keyframe_extraction_impl.hpp>
#include <calousel/logging.hpp>

namespace {

void apply_config_to_args(const calousel::Config& c, calousel::ExtractArgs& out) {
  out.bag_path = c.bag_path;
  out.keyframe_result_dir = c.keyframe_result_dir;
  out.image_topics = c.image_topics;
  out.intrinsic_yamls = c.intrinsic_yamls;

  out.cam_params.clear();
  for (const auto& p : c.cam_params) {
    calousel::CamExtractParams ep;
    ep.frame_window_size = p.frame_window_size;
    ep.angle_threshold = p.angle_threshold;
    ep.rolling_shutter_compensation = p.rolling_shutter_compensation;
    ep.fps = p.fps;
    ep.debug = p.debug;
    out.cam_params.push_back(ep);
  }

  out.board_n_x = c.board_n_x;
  out.board_n_y = c.board_n_y;
  out.board_radius = c.board_radius;
  out.board_distance = c.board_distance;
  out.board_asymmetric = c.board_asymmetric;
}

}  // namespace

int main(int argc, char** argv) {
  const auto logger = calousel::stderr_logger();

  if (argc < 3 || std::string(argv[1]) != "--config") {
    std::cerr << "Usage: keyframe_extractor_cli --config <yaml_path>\n"
              << "  All settings are read from the YAML file (bag_path, keyframe_result_dir, camera, board).\n";
    return 2;
  }

  const std::string config_path = argv[2];
  calousel::Config config;
  if (!calousel::load_config_from_yaml(config_path, config)) {
    std::cerr << "Failed to load config: " << config_path << "\n";
    return 2;
  }

  calousel::ExtractArgs args;
  apply_config_to_args(config, args);

  if (args.bag_path.empty() || args.keyframe_result_dir.empty() || args.intrinsic_yamls.empty() || args.image_topics.empty()) {
    std::cerr << "Config must define bag_path, keyframe_result_dir, and camera section (cam0, cam1, ...) with "
              << "topic and intrinsic_yaml.\n";
    return 2;
  }
  if (args.intrinsic_yamls.size() != args.image_topics.size()) {
    std::cerr << "Number of cameras must match number of image_topics.\n";
    return 2;
  }

  if (!calousel::run_keyframe_extraction(args, logger)) {
    return 1;
  }
  return 0;
}
