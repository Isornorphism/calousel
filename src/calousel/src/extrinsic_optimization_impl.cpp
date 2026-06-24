#include <filesystem>
#include <vector>

#include <calousel/keyframe_storage.hpp>
#include <calousel/optimize.hpp>
#include <calousel/extrinsic_optimization_impl.hpp>
#include <calousel/utils.hpp>

namespace fs = std::filesystem;

namespace calousel {

bool run_extrinsic_optimization(const ExtrinsicOptimizationArgs& args, const Logger& log) {
  std::string extrinsic_result_dir = args.extrinsic_result_dir.empty() ? args.keyframe_result_dir : args.extrinsic_result_dir;

  std::vector<CamConfig> cam_configs;
  BoardConfig board_config;
  if (!calousel::keyframe_storage::read_metadata_yaml(args.keyframe_result_dir, board_config, cam_configs)) {
    if (log.error) log.error("Failed to read metadata.yaml from " + args.keyframe_result_dir);
    return false;
  }
  if (cam_configs.empty()) {
    if (log.error) log.error("No camera configs in metadata (image_topics / intrinsics).");
    return false;
  }
  const int cam_num = static_cast<int>(cam_configs.size());

  std::vector<ThreadSafeQueue<KeyFrame>> queues(static_cast<size_t>(cam_num));
  std::vector<std::vector<double>> angular_vel_vecs(static_cast<size_t>(cam_num));

  for (int cam = 0; cam < cam_num; cam++) {
    fs::path cam_dir = fs::path(args.keyframe_result_dir) / ("cam" + std::to_string(cam));
    std::deque<KeyFrame> kfs;
    std::vector<double> angular_vel_vec;
    if (!calousel::keyframe_storage::read_keyframes_yaml(cam_dir, kfs, angular_vel_vec)) {
      if (log.error) log.error("Failed to read keyframe_data.yaml from " + cam_dir.string());
      return false;
    }
    for (const auto& kf : kfs) queues[static_cast<size_t>(cam)].push(kf);
    angular_vel_vecs[static_cast<size_t>(cam)] = std::move(angular_vel_vec);
    if (log.info) {
      log.info("Loaded cam" + std::to_string(cam) + " keyframes=" + std::to_string(kfs.size()));
    }
  }

  rp_calibration calib(queues, board_config, cam_configs, cam_num, args.use_weight, args.fix_reference_camera_z_to_zero);
  calib.set_discocal_target();
  calib.set_keyframe_ref(-1); // for debug
  calib.set_init_angular_vel(angular_vel_vecs);
  calib.set_init_T_ba();
  for (int cam = 0; cam < cam_num; cam++) {
    calib.set_init_T_ac(cam);
  }
  calib.move_initial_value_to_optimize_params();

  if (!args.optimize_reprojection_error) {
    (void)calib.optimize();
  } else {
    (void)calib.optimize_reprojection_error();
  }
  calib.calculate_rep_error();

  fs::create_directories(extrinsic_result_dir);
  return calib.save_extrinsic_calibration_result(extrinsic_result_dir, log);
}

}  // namespace calousel
