#ifndef CALOUSEL__KEY_FRAME_ACQUISITION_HPP_
#define CALOUSEL__KEY_FRAME_ACQUISITION_HPP_

#include <memory>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <CCalibrator.h>
#include <CTargetDetector.h>
#include <CCircleGridFinder.hpp>
#include <calousel/utils.hpp>
#include <calousel/logging.hpp>

namespace calousel
{
class key_frame_acquisition
{
public:
    key_frame_acquisition(const CamConfig& cam_config_, const BoardConfig& board_config_, string result_path_,
                          int frame_window_size_, double angle_threshold_, bool rolling_shutter_compensation_, int fps_,
                          ThreadSafeQueue<KeyFrame>& key_frame_queue_);
    ~key_frame_acquisition(){};

    bool isBoardInFOV(const cv::Mat& image, double downscale_factor = 2.0, bool visualize = false);
    
    void select_key_frames(const CameraFrame& current_frame);
    void select_key_frame_in_window();

    bool is_frame_window_empty() { return frame_window.empty(); };
    bool is_angular_vel_vec_empty() { return angular_vel_vec.empty(); };
    void push_back_angular_vel() { angular_vel_vec.push_back(this->angular_vel); };
    void calculate_Es();
    void compensate_rolling_shutter();
    
    bool save_key_frame_result(const std::string& result_path, const calousel::Logger& logger);

private:
    int find_closest_pose_in_window(const std::vector<se3>& result_Es, 
                                    const std::deque<CameraFrame>& window,
                                    const se3& ref_E);

    void first_calculate_angular_vel(const std::vector<se3>& Es, const std::deque<CameraFrame>& frame_window);
    void update_angular_vel_from_ref(const CameraFrame& current_frame, const se3& current_E);
    void update_angular_vel_per_revolution(int closest_idx, const std::deque<CameraFrame>& window);
    void update_revolution_ref(const CameraFrame& current_frame, const se3& current_E);


    std::deque<CameraFrame> frame_window;
    std::deque<CalibrationFrame> pre_cal_frame_queue;
    ThreadSafeQueue<KeyFrame>& key_frame_queue;
    int frame_window_size;
    double angle_threshold;
    BoardConfig board_config;
    CamConfig cam_config;

    int detection_cycle = 0;
    int processing_num = 0;
    bool new_cycle_detected = false;

    // Revolution time measurement members
    double angular_vel = 0.0;
    bool angular_vel_calculated = false;

    calousel::TimestampNs revolution_ref_ts_ns_ = 0;
    se3 revolution_ref_E_;
    int revolution_ref_cycle_ = -1;
    std::vector<double> angular_vel_vec;
    

    cv::SimpleBlobDetector::Params blob_params;
    
    TargetDetector discocal_detector;
    Calibrator discocal_calibrator;

    bool rolling_shutter_compensation = false;
    int fps = 30;
};

}

namespace YAML {
YAML::Emitter& operator<<(YAML::Emitter& out, const KeyFrame& kf);
inline const YAML::Node& operator>>(const YAML::Node& node, KeyFrame& kf);
}
#endif
