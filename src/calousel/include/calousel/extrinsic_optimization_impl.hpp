#ifndef CALOUSEL__EXTRINSIC_OPTIMIZATION_IMPL_HPP_
#define CALOUSEL__EXTRINSIC_OPTIMIZATION_IMPL_HPP_

#include <optional>
#include <string>
#include <calousel/logging.hpp>

namespace calousel {

struct ExtrinsicOptimizationArgs {
  std::string keyframe_result_dir;
  std::string extrinsic_result_dir;
  bool use_weight = false;
  bool fix_reference_camera_z_to_zero = true;
  bool optimize_reprojection_error = false;

  std::optional<int> board_n_x;
  std::optional<int> board_n_y;
  std::optional<double> board_radius;
  std::optional<double> board_distance;
};

/** Run extrinsic optimization (keyframe_result_dir -> extrinsic_result_dir). Returns false on failure. */
bool run_extrinsic_optimization(const ExtrinsicOptimizationArgs& args, const Logger& logger);

}  // namespace calousel

#endif
