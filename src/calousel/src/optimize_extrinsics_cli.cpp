#include <iostream>
#include <string>

#include <calousel/config_loader.hpp>
#include <calousel/logging.hpp>
#include <calousel/extrinsic_optimization_impl.hpp>

namespace {

void apply_config(const calousel::Config& c, calousel::ExtrinsicOptimizationArgs& a) {
  a.keyframe_result_dir = c.keyframe_result_dir;
  a.extrinsic_result_dir = c.extrinsic_result_dir.empty() ? a.keyframe_result_dir : c.extrinsic_result_dir;
  a.use_weight = c.use_weight;
  a.fix_reference_camera_z_to_zero = c.fix_reference_camera_z_to_zero;
  a.optimize_reprojection_error = c.optimize_reprojection_error;
  a.board_n_x = c.board_n_x;
  a.board_n_y = c.board_n_y;
  a.board_radius = c.board_radius;
  a.board_distance = c.board_distance;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || std::string(argv[1]) != "--config") {
    std::cerr << "Usage: optimize_extrinsics_cli --config <yaml_path>\n"
              << "  All settings from YAML (keyframe_result_dir, extrinsic_result_dir, optimization options).\n";
    return 2;
  }

  const std::string config_path = argv[2];
  calousel::Config config;
  if (!calousel::load_config_from_yaml(config_path, config)) {
    std::cerr << "Failed to load config: " << config_path << "\n";
    return 2;
  }

  calousel::ExtrinsicOptimizationArgs args;
  apply_config(config, args);

  if (args.keyframe_result_dir.empty()) {
    std::cerr << "Config must define keyframe_result_dir.\n";
    return 2;
  }
  if (args.extrinsic_result_dir.empty()) {
    args.extrinsic_result_dir = args.keyframe_result_dir;
  }

  const auto log = calousel::stderr_logger();
  if (!calousel::run_extrinsic_optimization(args, log)) {
    return 1;
  }
  return 0;
}
