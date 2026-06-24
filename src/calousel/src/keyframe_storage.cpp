#include <calousel/keyframe_storage.hpp>

#include <fstream>
#include <utility>

namespace calousel::keyframe_storage {

static void ensure_dir(const std::filesystem::path& p) {
  std::filesystem::create_directories(p);
}

bool write_metadata_yaml(const std::filesystem::path& keyframe_result_dir, const BoardConfig& board_config, const std::vector<CamConfig>& cam_configs) {
  try {
    ensure_dir(keyframe_result_dir);

    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "bag_path" << YAML::Value << cam_configs[0].bag_path;

    out << YAML::Key << "image_topics" << YAML::Value << YAML::BeginSeq;
    for (const auto& cam_config : cam_configs) out << cam_config.topic;
    out << YAML::EndSeq;

    out << YAML::Key << "intrinsics" << YAML::Value << YAML::BeginSeq;
    for (const auto& cam_config : cam_configs) {
      const Params& p = cam_config.intrinsic;
      out << YAML::BeginMap;
      out << YAML::Key << "cam_id" << YAML::Value << cam_config.cam_id;
      out << YAML::Key << "fx" << YAML::Value << p.fx;
      out << YAML::Key << "fy" << YAML::Value << p.fy;
      out << YAML::Key << "cx" << YAML::Value << p.cx;
      out << YAML::Key << "cy" << YAML::Value << p.cy;
      out << YAML::Key << "skew" << YAML::Value << p.skew;
      out << YAML::Key << "d1" << YAML::Value << p.d[0];
      out << YAML::Key << "d2" << YAML::Value << p.d[1];
      out << YAML::Key << "d3" << YAML::Value << p.d[2];
      out << YAML::Key << "d4" << YAML::Value << p.d[3];
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "discocal_target" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "n_x" << YAML::Value << board_config.n_x;
    out << YAML::Key << "n_y" << YAML::Value << board_config.n_y;
    out << YAML::Key << "n_d" << YAML::Value << board_config.n_d;
    out << YAML::Key << "radius" << YAML::Value << board_config.r;
    out << YAML::Key << "distance" << YAML::Value << board_config.distance;
    out << YAML::Key << "asymmetric" << YAML::Value << board_config.asymmetric;
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout((keyframe_result_dir / "metadata.yaml").string());
    fout << out.c_str();
    return true;
  } catch (...) {
    return false;
  }
}

bool read_metadata_yaml(const std::filesystem::path& keyframe_result_dir, BoardConfig& board_config, std::vector<CamConfig>& cam_configs) {
  try {
    const auto path = keyframe_result_dir / "metadata.yaml";
    if (!std::filesystem::exists(path)) return false;

    std::ifstream fin(path.string());
    YAML::Node doc = YAML::Load(fin);

    cam_configs.clear();

    if (doc["image_topics"]) {
      auto image_topics = doc["image_topics"].as<std::vector<std::string>>();
      std::string bag_path = doc["bag_path"] ? doc["bag_path"].as<std::string>() : "";
      for (size_t c = 0; c < image_topics.size(); c++) {
        CamConfig cfg;
        cfg.cam_id = static_cast<int>(c);
        cfg.topic = image_topics[c];
        cfg.bag_path = bag_path;
        cam_configs.push_back(cfg);
      }
    }

    if (doc["discocal_target"]) {
      const auto& dt = doc["discocal_target"];
      if (dt["n_x"]) board_config.n_x = dt["n_x"].as<int>();
      if (dt["n_y"]) board_config.n_y = dt["n_y"].as<int>();
      if (dt["n_d"]) board_config.n_d = dt["n_d"].as<int>();
      if (dt["radius"]) board_config.r = dt["radius"].as<double>();
      if (dt["distance"]) board_config.distance = dt["distance"].as<double>();
      if (dt["asymmetric"]) board_config.asymmetric = dt["asymmetric"].as<bool>();
      else if (dt["assymmetric"]) board_config.asymmetric = dt["assymmetric"].as<bool>();
    }

    if (doc["intrinsics"] && doc["intrinsics"].IsSequence()) {
      const auto& seq = doc["intrinsics"];
      size_t idx = 0;
      for (size_t i = 0; i < seq.size() && idx < cam_configs.size(); i++) {
        if (seq[i].IsMap() && seq[i]["fx"]) {
          cam_configs[idx].intrinsic = Params(seq[i]);
          idx++;
        }
      }
    }

    return true;
  } catch (...) {
    return false;
  }
}

bool read_keyframes_yaml(
    const std::filesystem::path& cam_dir,
    std::deque<KeyFrame>& keyframes_out,
    std::vector<double>& angular_vel_vec_out) {
  const auto path = cam_dir / "keyframe_data.yaml";
  if (!std::filesystem::exists(path)) return false;

  YAML::Node config;
  try {
    config = YAML::LoadFile(path.string());
  } catch (...) {
    return false;
  }

  if (!config["keyframes"].IsSequence()) return false;

  keyframes_out.clear();
  angular_vel_vec_out.clear();

  for (const auto& node_item : config["keyframes"]) {
    if (!node_item["data"] || !node_item["data"].IsMap()) continue;
    try {
      KeyFrame kf;
      node_item["data"] >> kf;
      keyframes_out.push_back(std::move(kf));
    } catch (...) {
      continue;
    }
  }

  if (config["angular_vel_initial_estimate"]) {
    const auto& av_node = config["angular_vel_initial_estimate"]["angular_vel_vec"];
    if (av_node.IsSequence()) {
      for (const auto& n : av_node) {
        angular_vel_vec_out.push_back(n.as<double>());
      }
    }
  }

  return !keyframes_out.empty();
}

}  // namespace calousel::keyframe_storage
