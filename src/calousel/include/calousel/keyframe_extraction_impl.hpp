#ifndef CALOUSEL__KEYFRAME_EXTRACTION_IMPL_HPP_
#define CALOUSEL__KEYFRAME_EXTRACTION_IMPL_HPP_

#include <optional>
#include <string>
#include <vector>
#include <calousel/logging.hpp>

namespace calousel {

/** Per-camera extract params. When cam_params is populated, use cam_params[cam_id]. */
struct CamExtractParams {
  int frame_window_size = 5;
  double angle_threshold = 3.0;
  bool rolling_shutter_compensation = false;
  int fps = 30;
  bool debug = false;
};

struct ExtractArgs {
  std::string bag_path;
  std::string keyframe_result_dir;
  std::vector<std::string> image_topics;
  std::vector<std::string> intrinsic_yamls;

  /** Per-camera params from camera.camN. */
  std::vector<CamExtractParams> cam_params;

  std::optional<int> board_n_x;
  std::optional<int> board_n_y;
  std::optional<double> board_radius;
  std::optional<double> board_distance;
  std::optional<bool> board_asymmetric;
};

/** Run keyframe extraction (bag -> keyframe_result_dir with metadata + cam0/cam1 keyframes). Returns false on failure. */
bool run_keyframe_extraction(const ExtractArgs& args, const Logger& logger);

}  // namespace calousel

#endif
