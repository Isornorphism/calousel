#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <iostream>
#include <sophus/se3.hpp>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <CLieAlgebra.h>
#include <utils.h>
#include <calousel/CLieAlgebra.hpp>

namespace calousel {
using TimestampNs = int64_t;
inline double to_seconds(TimestampNs ns) { return static_cast<double>(ns) * 1e-9; }
}


template <typename T>
class ThreadSafeQueue {
public:
    void push(const T& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push_back(value);
        lock.unlock();
        cond_var_.notify_one();
    }


    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return !queue_.empty(); });
        T value = queue_.front();
        queue_.pop_front();
        return value;
    }

    bool try_pop_for(T& value, long long timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (cond_var_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return !queue_.empty(); })) {
            value = queue_.front();
            queue_.pop_front();
            return true;
        }
        return false;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    void clear() {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.clear();
    }

    // size_t size() const {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     return queue_.size();
    // }
    int size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<int>(queue_.size());
    }

    T back() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return !queue_.empty(); });
        return queue_.back();
    }

    T front() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return !queue_.empty(); });
        return queue_.front();
    }

    T operator[](size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (index >= queue_.size()) {
            throw std::out_of_range("ThreadSafeDeque::operator[]: index out of bounds");
        }
        return queue_[index];
    }

    std::deque<T> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_;
    }

private:
    std::deque<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_var_;
};


struct CameraFrame {
    cv::Mat image;
    calousel::TimestampNs timestamp_ns;
    int detection_cycle;

    CameraFrame() : timestamp_ns(0) {}
    CameraFrame(const cv::Mat& image_, calousel::TimestampNs timestamp_ns_, int detection_cycle_)
        : image(image_.clone()), timestamp_ns(timestamp_ns_), detection_cycle(detection_cycle_)
    {}
};

struct CalibrationFrame {
    cv::Mat image;
    calousel::TimestampNs timestamp_ns;
    int detection_cycle;
    se3 initial_E;
    double rep_error, cal_qual;
    Eigen::Matrix<double, 6, 6> se3_cov;
    std::vector<Shape> target;
    std::vector<std::pair<double, double>> pixel_vels;
    bool is_pixel_vel_calculated = false;

    CalibrationFrame() : timestamp_ns(0), is_pixel_vel_calculated(false) {}
    CalibrationFrame(const cv::Mat& image_, calousel::TimestampNs timestamp_ns_, int detection_cycle_, const se3& initial_E_, double rep_error_, double cal_qual_, const Eigen::Matrix<double, 6, 6>& se3_cov_, const std::vector<Shape>& target_, const std::vector<std::pair<double, double>>& pixel_vels_, bool is_pixel_vel_calculated_)
        : image(image_.clone()), timestamp_ns(timestamp_ns_), detection_cycle(detection_cycle_), initial_E(initial_E_), rep_error(rep_error_), cal_qual(cal_qual_), se3_cov(se3_cov_), target(target_), pixel_vels(pixel_vels_), is_pixel_vel_calculated(is_pixel_vel_calculated_)
    {}
    CalibrationFrame(const CameraFrame& frame_, const se3& initial_E_, double rep_error_, double cal_qual_, const Eigen::Matrix<double, 6, 6>& se3_cov_, const std::vector<Shape>& target_, const std::vector<std::pair<double, double>>& pixel_vels_, bool is_pixel_vel_calculated_)
        : image(frame_.image.clone()), timestamp_ns(frame_.timestamp_ns), detection_cycle(frame_.detection_cycle), initial_E(initial_E_), rep_error(rep_error_), cal_qual(cal_qual_), se3_cov(se3_cov_), target(target_), pixel_vels(pixel_vels_), is_pixel_vel_calculated(is_pixel_vel_calculated_)
    {}
};

struct KeyFrame {
    // cv::Mat image;
    calousel::TimestampNs timestamp_ns;
    int detection_cycle;
    se3 E;
    double rep_error, cal_qual;
    Eigen::Matrix<double, 6, 6> se3_cov;
    std::vector<Shape> target;
    
    KeyFrame() : timestamp_ns(0) {}
    KeyFrame(const CalibrationFrame& cal_frame_)
        : timestamp_ns(cal_frame_.timestamp_ns), detection_cycle(cal_frame_.detection_cycle), E(cal_frame_.initial_E), rep_error(cal_frame_.rep_error), cal_qual(cal_frame_.cal_qual), se3_cov(cal_frame_.se3_cov), target(cal_frame_.target)
    {}
    KeyFrame(calousel::TimestampNs timestamp_ns_, int detection_cycle_, const se3& E_, double rep_error_, double cal_qual_, const Eigen::Matrix<double, 6, 6>& se3_cov_, const std::vector<Shape>& target_)
        : timestamp_ns(timestamp_ns_), detection_cycle(detection_cycle_), E(E_), rep_error(rep_error_), cal_qual(cal_qual_), se3_cov(se3_cov_), target(target_)
    {}
};

struct Circle3D {
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
    double radius;
};

struct BoardConfig {
    int n_x = 0, n_y = 0;
    int n_d = 0;
    double r = 0.0, distance = 0.0;
    bool asymmetric = false;
};

struct CamConfig {
    int cam_id = 0;
    std::string bag_path;
    std::string topic;
    Params intrinsic;
};

inline CamConfig load_cam_config(const std::string& yaml_path, int cam_id, const std::string& bag_path, const std::string& topic) {
    CamConfig cfg;
    cfg.cam_id = cam_id;
    cfg.bag_path = bag_path;
    cfg.topic = topic;
  
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node intr = root["intrinsic_calibration"];
  
    cfg.intrinsic = Params(
        intr["fx"].as<double>(),
        intr["fy"].as<double>(),
        intr["cx"].as<double>(),
        intr["cy"].as<double>(),
        intr["skew"].as<double>(),
        intr["d1"].as<double>(),
        intr["d2"].as<double>(),
        intr["d3"].as<double>(),
        intr["d4"].as<double>());
  
    // optional uncertainty
    if (intr["sfx"]) {
      std::array<double, 9> params_cov = {
          intr["sfx"].as<double>(),
          intr["sfy"].as<double>(),
          intr["scx"].as<double>(),
          intr["scy"].as<double>(),
          intr["sskew"].as<double>(),
          intr["sd1"].as<double>(),
          intr["sd2"].as<double>(),
          intr["sd3"].as<double>(),
          intr["sd4"].as<double>()};
      cfg.intrinsic.update_unc(params_cov);
    }
  
    return cfg;
  }
  
inline BoardConfig load_board_config(const std::string& yaml_path) {
    BoardConfig cfg;
    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node camera = root["camera"];
    cfg.n_d = camera["n_d"].as<int>();
    cfg.n_x = camera["n_x"].as<int>();
    cfg.n_y = camera["n_y"].as<int>();
    cfg.r = camera["radius"].as<double>();
    cfg.distance = camera["distance"].as<double>();
    if (camera["asymmetric"]) cfg.asymmetric = camera["asymmetric"].as<bool>();
    else if (camera["assymmetric"]) cfg.asymmetric = camera["assymmetric"].as<bool>();
    return cfg;
}

// Message structure for queue
struct ImageMessage {
    cv::Mat image;
    int64_t timestamp_ns;
    int cam_id;
  };

namespace YAML {

template<>
struct convert<Eigen::Matrix<double, 6, 6>> {
    static Node encode(const Eigen::Matrix<double, 6, 6>& rhs) {
        Node node;
        for (int i = 0; i < rhs.rows(); ++i) {
            Node row_node;
            row_node.SetStyle(YAML::EmitterStyle::Flow);
            for (int j = 0; j < rhs.cols(); ++j) {
                std::stringstream ss;
                ss << std::setprecision(6) << rhs(i, j);
                row_node.push_back(ss.str());
            }
            node.push_back(row_node);
        }
        return node;
    }

    static bool decode(const Node& node, Eigen::Matrix<double, 6, 6>& rhs) {
        if (!node.IsSequence() || node.size() != rhs.rows()) {
            return false;
        }

        for (int i = 0; i < rhs.rows(); ++i) {
            const Node& row_node = node[i];
            if (!row_node.IsSequence() || row_node.size() != rhs.cols()) {
                return false;
            }

            for (int j = 0; j < rhs.cols(); ++j) {
                try {
                    rhs(i, j) = row_node[j].as<double>();
                } catch (const YAML::BadConversion& e) {
                    return false;
                }
            }
        }
        return true;
    }
};


template<>
struct convert<Eigen::Matrix4d> {
    static Node encode(const Eigen::Matrix4d& rhs) {
        Node node;
        for (int i = 0; i < rhs.rows(); ++i) {
            Node row_node;
            row_node.SetStyle(YAML::EmitterStyle::Flow);
            for (int j = 0; j < rhs.cols(); ++j) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(6) << rhs(i, j);
                row_node.push_back(ss.str());
            }
            node.push_back(row_node);
        }
        return node;
    }

    static bool decode(const Node& node, Eigen::Matrix4d& rhs) {
        if (!node.IsSequence() || node.size() != rhs.rows()) {
            return false;
        }

        for (int i = 0; i < rhs.rows(); ++i) {
            const Node& row_node = node[i];
            if (!row_node.IsSequence() || row_node.size() != rhs.cols()) {
                return false;
            }

            for (int j = 0; j < rhs.cols(); ++j) {
                try {
                    rhs(i, j) = row_node[j].as<double>();
                } catch (const YAML::BadConversion& e) {
                    return false;
                }
            }
        }
        return true;
    }
};

template<>
struct convert<Eigen::Vector3d> {
    static Node encode(const Eigen::Vector3d& rhs) {
        Node node;
        node.SetStyle(YAML::EmitterStyle::Flow);
        for (int i = 0; i < 3; ++i) {
            std::stringstream ss;
            ss << std::setprecision(6) << rhs(i);
            node.push_back(ss.str());
        }
        return node;
    }
};

template<>
struct convert<std::vector<Shape>> {
    static Node encode(const std::vector<Shape>& shapes) {
        Node node(YAML::NodeType::Sequence);
        
        for (const auto& shape : shapes) {
            Node shape_node;
            shape_node["x"] = shape.x;
            shape_node["y"] = shape.y;
            shape_node["m00"] = shape.m00;
            shape_node["m10"] = shape.m10;
            shape_node["m01"] = shape.m01;
            shape_node["m20"] = shape.m20;
            shape_node["m11"] = shape.m11;
            shape_node["m02"] = shape.m02;
            shape_node["Kxx"] = shape.Kxx;
            shape_node["Kxy"] = shape.Kxy;
            shape_node["Kyy"] = shape.Kyy;
            shape_node["n"] = shape.n;

            node.push_back(shape_node);
        }
        return node;
    }

    
    static bool decode(const Node& node, std::vector<Shape>& shapes) {
        if (!node.IsSequence()) {
            return false;
        }
        shapes.clear();
        
        for (const auto& shape_node : node) {
            shapes.emplace_back(
                shape_node["n"].as<int32_t>(),
                shape_node["m00"].as<double>(),
                shape_node["m10"].as<double>(),
                shape_node["m01"].as<double>(),
                shape_node["m20"].as<double>(),
                shape_node["m11"].as<double>(),
                shape_node["m02"].as<double>(),
                shape_node["Kxx"].as<double>(),
                shape_node["Kxy"].as<double>(),
                shape_node["Kyy"].as<double>()
            );
        }
        return true;
    }
};


inline YAML::Emitter& operator<<(YAML::Emitter& out, const KeyFrame& kf) {
    out << YAML::BeginMap;
    out << YAML::Key << "timestamp_sec" << YAML::Value << calousel::to_seconds(kf.timestamp_ns);
    out << YAML::Key << "timestamp_nanosec" << YAML::Value << static_cast<uint64_t>(kf.timestamp_ns);
    out << YAML::Key << "detection_cycle" << YAML::Value << kf.detection_cycle;
    out << YAML::Key << "se3" << YAML::Value << kf.E.to_string(); // to_string() 사용
    out << YAML::Key << "rep_error" << YAML::Value << kf.rep_error;
    out << YAML::Key << "cal_qual" << YAML::Value << kf.cal_qual;
    out << YAML::Key << "se3_cov" << YAML::Value << YAML::convert<Eigen::Matrix<double, 6, 6>>::encode(kf.se3_cov);
    out << YAML::Key << "target" <<  YAML::Value << YAML::convert<std::vector<Shape>>::encode(kf.target);
    out << YAML::EndMap;
    return out;
}

inline const YAML::Node& operator>>(const YAML::Node& node, KeyFrame& kf) {
    if (!node.IsMap()) {
        throw YAML::Exception(YAML::Mark(), "Cannot convert non-map YAML Node to KeyFrame");
    }

    try {
        if (node["timestamp_nanosec"]) {
            kf.timestamp_ns = static_cast<calousel::TimestampNs>(node["timestamp_nanosec"].as<uint64_t>());
        } else if (node["timestamp_ns"]) {
            kf.timestamp_ns = static_cast<calousel::TimestampNs>(node["timestamp_ns"].as<uint64_t>());
        } else if (node["timestamp_sec"]) {
            const double sec = node["timestamp_sec"].as<double>();
            kf.timestamp_ns = static_cast<calousel::TimestampNs>(sec * 1e9);
        } else {
            kf.timestamp_ns = 0;
        }
        kf.detection_cycle = node["detection_cycle"].as<int>();
    } catch (const YAML::BadConversion& e) {
        std::string msg = "Failed to parse timestamp or detection_cycle: ";
        msg += e.what();
        throw YAML::Exception(YAML::Mark(), msg);
    }
    
    try {
        kf.E = kf.E.from_string(node["se3"].as<std::string>());
    } catch (const std::exception& e) {
        std::string msg = "Error parsing se3 from string: ";
        msg += e.what();
        throw YAML::Exception(YAML::Mark(), msg);
    } catch (const YAML::BadConversion& e) {
        std::string msg = "Failed to convert 'se3' node: ";
        msg += e.what();
        throw YAML::Exception(YAML::Mark(), msg);
    }


    try {
        kf.rep_error = node["rep_error"].as<double>();
        kf.cal_qual = node["cal_qual"].as<double>();
    } catch (const YAML::BadConversion& e) {
        std::string msg = "Failed to parse rep_error or cal_qual: ";
        msg += e.what();
        throw YAML::Exception(YAML::Mark(), msg);
    }

    try {
        kf.se3_cov = node["se3_cov"].as<Eigen::Matrix<double, 6, 6>>();
    } catch (const YAML::BadConversion& e) {
        std::string msg = "Failed to convert 'se3_cov' node: ";
        msg += e.what();
        throw YAML::Exception(YAML::Mark(), msg);
    }

    try {
        kf.target = node["target"].as<std::vector<Shape>>();
    } catch (const YAML::BadConversion& e) {
        std::string msg = "Failed to convert 'target' node: ";
        msg += e.what();
        throw YAML::Exception(YAML::Mark(), msg);
    }
    
    return node;
}
}


#endif