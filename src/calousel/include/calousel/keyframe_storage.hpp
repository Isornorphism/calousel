#ifndef CALOUSEL__KEYFRAME_STORAGE_HPP_
#define CALOUSEL__KEYFRAME_STORAGE_HPP_

#include <cstdint>
#include <deque>
#include <filesystem>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <calousel/utils.hpp>

namespace calousel::keyframe_storage {


// Writes keyframe_result_dir/metadata.yaml
bool write_metadata_yaml(const std::filesystem::path& keyframe_result_dir, const BoardConfig& board_config, const std::vector<CamConfig>& cam_configs);
bool read_metadata_yaml(const std::filesystem::path& keyframe_result_dir, BoardConfig& board_config, std::vector<CamConfig>& cam_configs);

// Reads keyframe_result_dir/cam{i}/keyframe_data.yaml
bool read_keyframes_yaml(
    const std::filesystem::path& cam_dir,
    std::deque<KeyFrame>& keyframes_out,
    std::vector<double>& angular_vel_vec_out);

}  // namespace calousel::keyframe_storage

#endif
