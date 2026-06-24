#include <calousel/key_frame_acquisition.hpp>
#include <calousel/CLieAlgebra.hpp>
#include <sophus/se3.hpp>
#include <algorithm>
#include <cmath>
#include <string>

namespace calousel
{
key_frame_acquisition::key_frame_acquisition(
    const CamConfig& cam_config_, const BoardConfig& board_config_, string result_path_,
    int frame_window_size_, double angle_threshold_, bool rolling_shutter_compensation_, int fps_,
    ThreadSafeQueue<KeyFrame>& key_frame_queue_
)
    : frame_window_size(frame_window_size_), angle_threshold(angle_threshold_),
      rolling_shutter_compensation(rolling_shutter_compensation_), fps(fps_),
      cam_config(cam_config_), board_config(board_config_),
      discocal_detector(board_config_.n_x, board_config_.n_y, board_config_.asymmetric),
      discocal_calibrator(board_config_.n_x, board_config_.n_y, board_config_.asymmetric, board_config_.n_d, board_config_.r, board_config_.distance, frame_window_size_, result_path_),
      key_frame_queue(key_frame_queue_)
{
    blob_params.minArea = 100;
    blob_params.maxArea = 1000;
    blob_params.filterByCircularity = true;
    blob_params.minCircularity = 0.8;
    blob_params.filterByConvexity = true;
    blob_params.minConvexity = 0.8;
    blob_params.filterByColor = true;
    blob_params.blobColor = 0;
}

bool key_frame_acquisition::isBoardInFOV(const cv::Mat& image, double downscale_factor, bool visualize) {
    cv::Mat gray_image;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    }
    else {
        gray_image = image;
    }

    cv::Mat processed_image;
    if (downscale_factor > 1.0) {
        cv::resize(gray_image, processed_image, cv::Size(), 1.0 / downscale_factor, 1.0 / downscale_factor, cv::INTER_AREA);
    }

    cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(blob_params);
    const cv::Size patternSize = board_config.asymmetric
        ? cv::Size(board_config.n_y, board_config.n_x)
        : cv::Size(board_config.n_x, board_config.n_y);

    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point2f> found_corners;

    blobDetector->detect(processed_image, keypoints);

    std::vector<cv::Point2f> keypoints_pt;
    keypoints_pt.reserve(keypoints.size());
    for (const auto& kp : keypoints) {
        keypoints_pt.push_back(kp.pt);
    }

    CircleGridFinder gridfinder(board_config.asymmetric);
    gridfinder.findGrid(keypoints_pt, patternSize, found_corners);

    if(visualize) {
        cv::Mat grid_attempt_display = processed_image.clone();
        cv::cvtColor(grid_attempt_display, grid_attempt_display, cv::COLOR_GRAY2BGR);

        if (!found_corners.empty()) {
            for (const auto& pt : found_corners) {
                cv::circle(grid_attempt_display, pt, 5, cv::Scalar(0, 255, 0), -1); // 녹색 원
            }
        }

        cv::imshow("FindCirclesGrid Attempt cam" + std::to_string(cam_config.cam_id), grid_attempt_display);
        cv::waitKey(1);
    }

    return found_corners.size() == static_cast<size_t>(board_config.n_x * board_config.n_y) ? true : false;
}


void key_frame_acquisition::first_calculate_angular_vel(const std::vector<se3>& Es, const std::deque<CameraFrame>& frame_window) {
    if(Es.size() <= 1) {
        std::cout << "Size of frame window must be greater than 2 for calculating angular velocity." << std::endl;
        return;
    }
    std::vector<double> angular_vel_tmp_vec;
    for(int i=0; i<Es.size()-1; i++) {
        const double time_diff_sec =
            calousel::to_seconds(frame_window[i+1].timestamp_ns - frame_window[i].timestamp_ns);
        if (time_diff_sec <= 0.0) {
            continue;
        }
        double angular_vel_est = LieAlgebra_rp::angle_diff(Es[i], Es[i+1]) / time_diff_sec * 180./M_PI;
        angular_vel_tmp_vec.push_back(angular_vel_est);
    }
    if (angular_vel_tmp_vec.empty()) {
        return;
    }
    angular_vel = std::accumulate(angular_vel_tmp_vec.begin(), angular_vel_tmp_vec.end(), 0.0) / angular_vel_tmp_vec.size();
    angular_vel_calculated = true;
    printf("[Cam %d] Set temp angular vel : %f\n", cam_config.cam_id, angular_vel);
}

void key_frame_acquisition::update_angular_vel_from_ref(const CameraFrame& current_frame, const se3& current_E) {
    const double time_diff_sec =
        calousel::to_seconds(current_frame.timestamp_ns - revolution_ref_ts_ns_);
    if(time_diff_sec <= 0.0) {
        return;
    }

    angular_vel = LieAlgebra_rp::angle_diff(revolution_ref_E_, current_E) / time_diff_sec * 180./M_PI;
    angular_vel_calculated = true;

    printf("[Cam %d] Update temp angular vel : %f\n", cam_config.cam_id, angular_vel);
}

int key_frame_acquisition::find_closest_pose_in_window(const std::vector<se3>& result_Es,
                                                       const std::deque<CameraFrame>& window,
                                                       const se3& ref_E) {
    if (result_Es.size() != window.size() || result_Es.empty()) {
        return -1;
    }
    
    // Convert ref_E to SE3
    Twist ref_twist = LieAlgebra_rp::to_twist(ref_E);
    Eigen::Matrix4d ref_T_CB_matrix = LieAlgebra_rp::to_SE3(ref_twist);
    Sophus::SE3d ref_T_CB(ref_T_CB_matrix);
    
    int best_idx = -1;
    double min_twist_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < result_Es.size(); i++) {
        // Convert current E to SE3
        Twist curr_twist = LieAlgebra_rp::to_twist(result_Es[i]);
        Eigen::Matrix4d curr_T_CB_matrix = LieAlgebra_rp::to_SE3(curr_twist);
        Sophus::SE3d curr_T_CB(curr_T_CB_matrix);
        
        // Compute relative transformation: T_rel = ref_T_CB^-1 * curr_T_CB
        Sophus::SE3d T_rel = ref_T_CB.inverse() * curr_T_CB;
        
        // Get twist vector (6D) and compute its norm
        Eigen::Matrix<double, 6, 1> twist_vec = T_rel.log();
        double distance = twist_vec.norm();
        
        if (distance < min_twist_distance) {
            min_twist_distance = distance;
            best_idx = static_cast<int>(i);
        }
    }
    
    if (best_idx >= 0) {
        printf("[Cam %d] Closest match: idx=%d, twist_norm=%.6f\n",
               cam_config.cam_id, best_idx, min_twist_distance);
    }
    return best_idx;
}

void key_frame_acquisition::update_angular_vel_per_revolution(int closest_idx, const std::deque<CameraFrame>& window) {
    if (closest_idx >= 0) {
        calousel::TimestampNs revolution_match_ts_ns_ = window[closest_idx].timestamp_ns;
        double revolution_time_sec_ = calousel::to_seconds(revolution_match_ts_ns_ - revolution_ref_ts_ns_);
        int revolution_delta = window[closest_idx].detection_cycle - revolution_ref_cycle_;

        if (revolution_time_sec_ <= 0.0 || revolution_delta <= 0) {
            printf("[Cam %d] Invalid revolution time found\n", cam_config.cam_id);
            angular_vel_vec.push_back(0.0);
            return;
        }

        angular_vel = 360.0 * static_cast<double>(revolution_delta) / revolution_time_sec_;
        angular_vel_vec.push_back(angular_vel);

        printf("[Cam %d] ======== REVOLUTION TIME MEASURED ========\n", cam_config.cam_id);
        printf("[Cam %d] Reference: cycle=%d, ts=%.3f\n",
               cam_config.cam_id, revolution_ref_cycle_, calousel::to_seconds(revolution_ref_ts_ns_));
        printf("[Cam %d] Match: cycle=%d, ts=%.3f\n",
               cam_config.cam_id, window[closest_idx].detection_cycle,
               calousel::to_seconds(revolution_match_ts_ns_));
        printf("[Cam %d] revolution angular vel: %.6f deg/s\n", cam_config.cam_id, angular_vel);
        printf("[Cam %d] ===========================================\n", cam_config.cam_id);

    }
    else {
        printf("[Cam %d] No revolution time found\n", cam_config.cam_id);
        angular_vel_vec.push_back(0.0);
    }
}

void key_frame_acquisition::update_revolution_ref(const CameraFrame& current_frame, const se3& current_E) {
    revolution_ref_ts_ns_ = current_frame.timestamp_ns;
    revolution_ref_E_ = current_E;
    revolution_ref_cycle_ = current_frame.detection_cycle;
}

void key_frame_acquisition::select_key_frame_in_window() {
    discocal_calibrator.clear();
    if(frame_window.size() == 0) return;

    discocal_calibrator.set_max_scene(frame_window.size());

    cv::Mat gray_img;
    std::vector<std::pair<bool, std::vector<Shape>>> result_vec;
    std::vector<int> success_idx_vec;
    for(int idx=0; idx<frame_window.size(); idx++) {
        gray_img = TargetDetector::preprocessing(frame_window[idx].image, "");
        if(gray_img.rows == 0) {
            throw exception();
        }
        result_vec.push_back( discocal_detector.detect(gray_img, "circle", false));
        if(result_vec.back().first) {
            discocal_calibrator.inputTarget(result_vec.back().second);
            success_idx_vec.push_back(idx);
        }
    }

    if (success_idx_vec.empty()) {
        printf("[Cam %d] No target detected\n", cam_config.cam_id);
        frame_window.clear();
        return;
    }

    std::deque<CameraFrame> success_frame_window;
    for (int success_idx : success_idx_vec) {
        success_frame_window.push_back(frame_window[success_idx]);
    }

    // Calculate pixel velocity for rolling shutter effect calibration.
    std::vector<std::pair<double, double>> pixel_vel_vec;
    bool is_pixel_vel_calculated = false;
    if (rolling_shutter_compensation && success_idx_vec.size() >= 2) {
        std::vector<std::pair<double, double>> first_result_centers, last_result_centers;
        int first_idx = success_idx_vec.front();
        int last_idx = success_idx_vec.back();
        const auto& first_target = result_vec[first_idx].second;
        const auto& last_target = result_vec[last_idx].second;
        double time_diff_sec = calousel::to_seconds(frame_window[last_idx].timestamp_ns - frame_window[first_idx].timestamp_ns);
        if(first_target.size() == last_target.size() && time_diff_sec > 0.0) {
            for(size_t i=0; i<first_target.size(); i++) {
                first_result_centers.push_back(std::make_pair(first_target[i].x, first_target[i].y));
                last_result_centers.push_back(std::make_pair(last_target[i].x, last_target[i].y));
            }
            for(size_t i=0; i<first_result_centers.size(); i++) {
                std::pair<double, double> pixel_vel = std::make_pair(
                    (last_result_centers[i].first - first_result_centers[i].first) / time_diff_sec,
                    (last_result_centers[i].second - first_result_centers[i].second) / time_diff_sec);
                pixel_vel_vec.push_back(pixel_vel);
            }
            is_pixel_vel_calculated = true;
        }
    }

    discocal_calibrator.set_inital_Es(cam_config.intrinsic);
    discocal_calibrator.update_Es(cam_config.intrinsic, 0, false);
    
    std::vector<se3> result_Es = discocal_calibrator.get_extrinsic();
    if (result_Es.empty()) {
        printf("[Cam %d] No pose estimated\n", cam_config.cam_id);
        frame_window.clear();
        return;
    }
    this->processing_num += 1;
    int result_idx;

    std::vector<double> rep_error = discocal_calibrator.cal_reprojection_error_each_image(cam_config.intrinsic, 0);
    std::vector<double> cal_qual = discocal_calibrator.cal_calibration_quality_each_image(cam_config.intrinsic, 0);
    std::vector<Eigen::Matrix<double, 6, 6>> se3_cov = discocal_calibrator.compute_se3_cov(cam_config.intrinsic, 0);
    if (rep_error.empty() || cal_qual.empty() || se3_cov.empty()) {
        printf("[Cam %d] No keyframe quality metrics computed\n", cam_config.cam_id);
        frame_window.clear();
        return;
    }
    auto min_it = std::min_element(rep_error.begin(), rep_error.end());
    result_idx = std::distance(rep_error.begin(), min_it);

    const bool first_window_in_cycle = new_cycle_detected;
    if (new_cycle_detected) {
        int revolution_idx = find_closest_pose_in_window(result_Es, success_frame_window, revolution_ref_E_);
        update_angular_vel_per_revolution(revolution_idx, success_frame_window);
        new_cycle_detected = false;
    }

    int frame_idx = success_idx_vec[result_idx];
    printf("[Cam %d / Cycle %d / Cnt %d] ", cam_config.cam_id, success_frame_window[result_idx].detection_cycle, this->processing_num);
    std::cout << "Total processed frame : " << success_frame_window.size() << " / ";
    std::cout << frame_idx << "th frame is selected" << std::endl;

    if(!angular_vel_calculated) {
        first_calculate_angular_vel(result_Es, success_frame_window);
        if(angular_vel_calculated) {
            update_revolution_ref(success_frame_window[result_idx], result_Es[result_idx]);
        }
    }
    else if(detection_cycle == 0 && !first_window_in_cycle) {
        update_angular_vel_from_ref(success_frame_window[result_idx], result_Es[result_idx]);
    }

    pre_cal_frame_queue.push_back(CalibrationFrame(success_frame_window[result_idx],
        result_Es[result_idx],
        rep_error[result_idx],
        cal_qual[result_idx],
        se3_cov[result_idx],
        result_vec[frame_idx].second,
        pixel_vel_vec,
        is_pixel_vel_calculated));

    frame_window.clear();
}


void key_frame_acquisition::select_key_frames(const CameraFrame& current_frame) {
    if(pre_cal_frame_queue.size() == 0) {
        if(frame_window.size() != frame_window_size)
            frame_window.push_back(current_frame);

        if(frame_window.size() == frame_window_size) {
            select_key_frame_in_window();
        }
    }
    else {
        const double time_diff_sec =
            calousel::to_seconds(current_frame.timestamp_ns - pre_cal_frame_queue.back().timestamp_ns);
        if(time_diff_sec * angular_vel >= angle_threshold) {
            if(frame_window.size() != frame_window_size) {
                // New cycle detected
                if(detection_cycle != current_frame.detection_cycle) {
                    if(frame_window.size() != 0) {
                        select_key_frame_in_window();
                    }
                    new_cycle_detected = true;
                    detection_cycle += 1;

                }
                frame_window.push_back(current_frame);
            }

            if(frame_window.size() == frame_window_size) {
                select_key_frame_in_window();
            }
        }
    }
}

void key_frame_acquisition::calculate_Es() {
    discocal_calibrator.clear();

    if(pre_cal_frame_queue.size()==0) return;

    discocal_calibrator.set_max_scene(pre_cal_frame_queue.size());
    printf("[Cam %d] ", cam_config.cam_id);
    std::cout << "Total calibration frame : " << pre_cal_frame_queue.size() << std::endl;

    for(auto cf : pre_cal_frame_queue) {
        discocal_calibrator.inputTarget(cf.target);
        discocal_calibrator.set_inital_Es(cf.initial_E);
    }

    Params final_params;
    std::vector<int> sample;
    for(int i=0; i<pre_cal_frame_queue.size(); i++) {
        sample.push_back(i);
    }
    final_params = discocal_calibrator.batch_optimize(sample, cam_config.intrinsic, 0, false, true, false);

    std::vector<se3> result_Es = discocal_calibrator.get_extrinsic();
    std::vector<double> rep_error = discocal_calibrator.cal_reprojection_error_each_image(final_params, 0);
    std::vector<double> cal_qual = discocal_calibrator.cal_calibration_quality_each_image(final_params, 0);
    std::vector<Eigen::Matrix<double, 6, 6>> se3_cov = discocal_calibrator.compute_se3_cov(final_params, 0);

    for(int i=0; i<result_Es.size(); i++)
        key_frame_queue.push(KeyFrame(pre_cal_frame_queue[i].timestamp_ns,
                                      pre_cal_frame_queue[i].detection_cycle,
                                      result_Es[i],
                                      rep_error[i],
                                      cal_qual[i],
                                      se3_cov[i],
                                      pre_cal_frame_queue[i].target));
}

void key_frame_acquisition::compensate_rolling_shutter() {
    discocal_calibrator.clear();

    if(pre_cal_frame_queue.size()==0) return;

    discocal_calibrator.set_max_scene(pre_cal_frame_queue.size());
    printf("[Cam %d] ", cam_config.cam_id);
    std::cout << "Total calibration frame : " << pre_cal_frame_queue.size() << std::endl;

    std::vector<std::vector<pair<double, double>>> pixel_vels_vec;

    for(int i=0; i<pre_cal_frame_queue.size(); i++) {
        auto cf = pre_cal_frame_queue[i];
        if(cf.is_pixel_vel_calculated) {
            pixel_vels_vec.push_back(cf.pixel_vels);
        }
        else {
            if(i != pre_cal_frame_queue.size()-1) {
                auto next_cf = pre_cal_frame_queue[i+1];
                double time_diff_sec = calousel::to_seconds(next_cf.timestamp_ns - cf.timestamp_ns);
                std::vector<std::pair<double, double>> pixel_vels;
                for(int j=0; j<cf.target.size(); j++) {
                    std::pair<double, double> pixel_vel = std::make_pair(
                        (next_cf.target[j].x - cf.target[j].x) / time_diff_sec,
                        (next_cf.target[j].y - cf.target[j].y) / time_diff_sec);
                    pixel_vels.push_back(pixel_vel);
                }
                pixel_vels_vec.push_back(pixel_vels);
            }
            else {
                auto prev_cf = pre_cal_frame_queue[i-1];
                double time_diff_sec = calousel::to_seconds(cf.timestamp_ns - prev_cf.timestamp_ns);
                std::vector<std::pair<double, double>> pixel_vels;
                for(int j=0; j<cf.target.size(); j++) {
                    std::pair<double, double> pixel_vel = std::make_pair(
                        (cf.target[j].x - prev_cf.target[j].x) / time_diff_sec,
                        (cf.target[j].y - prev_cf.target[j].y) / time_diff_sec);
                    pixel_vels.push_back(pixel_vel);
                }
                pixel_vels_vec.push_back(pixel_vels);
            }
        }
        discocal_calibrator.inputTarget(cf.target);
        discocal_calibrator.set_inital_Es(cf.initial_E);
    }

    // printf("[Cam %d] ================================ Pixel velocity per target ================================\n", cam_config.cam_id);
    // for(int i=0; i<pixel_vels_vec.size(); i++) {
    //     printf("[Cam %d] Frame %d: \n", cam_config.cam_id, i);
    //     for(int j=0; j<pixel_vels_vec[i].size(); j++) {
    //         printf("[Cam %d] Target %d: (%.6f, %.6f)\n", cam_config.cam_id, j, pixel_vels_vec[i][j].first, pixel_vels_vec[i][j].second);
    //     }
    //     printf("\n");
    // }

    int width = pre_cal_frame_queue[0].image.cols;
    int height = pre_cal_frame_queue[0].image.rows;
    discocal_calibrator.set_image_size(width, height);

    double t_readout[1];

    t_readout[0] = 1 / static_cast<double>(fps);

    discocal_calibrator.update_Es_rolling_shutter(
        cam_config.intrinsic, 0, pixel_vels_vec, t_readout, false);

    printf("[Cam %d] Rolling shutter calibration completed, t_readout: %.6f\n", cam_config.cam_id, t_readout[0]);


    for(int i=0; i<pre_cal_frame_queue.size(); i++) {
        for(int j=0; j<pre_cal_frame_queue[i].target.size(); j++) {
            double t_line = t_readout[0] * pre_cal_frame_queue[i].target[j].y / height;
            pre_cal_frame_queue[i].target[j].x -= t_line * pixel_vels_vec[i][j].first;
            pre_cal_frame_queue[i].target[j].y -= t_line * pixel_vels_vec[i][j].second;
        }
    }

    std::vector<se3> result_Es = discocal_calibrator.get_extrinsic();
    std::vector<double> rep_error = discocal_calibrator.cal_reprojection_error_rolling_shutter_each_image(cam_config.intrinsic, 0, pixel_vels_vec, t_readout);
    std::vector<double> cal_qual = discocal_calibrator.cal_calibration_quality_rolling_shutter_each_image(cam_config.intrinsic, 0, pixel_vels_vec, t_readout);
    std::vector<Eigen::Matrix<double, 6, 6>> se3_cov = discocal_calibrator.compute_se3_cov_rolling_shutter(cam_config.intrinsic, 0, pixel_vels_vec, t_readout);

    for(int i=0; i<result_Es.size(); i++) 
        key_frame_queue.push(KeyFrame(pre_cal_frame_queue[i].timestamp_ns,
                                      pre_cal_frame_queue[i].detection_cycle,
                                      result_Es[i],
                                      rep_error[i],
                                      cal_qual[i],
                                      se3_cov[i],
                                      pre_cal_frame_queue[i].target));
}



bool key_frame_acquisition::save_key_frame_result(const std::string& result_path, const calousel::Logger& logger) {
    namespace fs = std::filesystem;

    try {
        fs::path base_dir(result_path);
        if (!fs::exists(base_dir)) {
            fs::create_directories(base_dir);
            if (logger.info) logger.info(std::string("Created result directory: ") + base_dir.string());
        }
    }
    catch (const fs::filesystem_error& e) {
        if (logger.error) logger.error(std::string("Failed to create result directory: ") + result_path + " : " + e.what());
        return false;
    }

    fs::path yaml_file_path = fs::path(result_path) / "keyframe_data.yaml";
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "keyframes";
    out << YAML::Value << YAML::BeginSeq;


    int num_keyframes = key_frame_queue.size(); 

    if (logger.info) logger.info("Processing " + std::to_string(num_keyframes) + " keyframes from the queue...");


    out << YAML::BeginMap;
    out << YAML::Key << "num_keyframes" << YAML::Value << num_keyframes;
    out << YAML::EndMap;

    int saved_count = 0;
    KeyFrame kf;
    for(int i=0; i<num_keyframes; i++) {
        try {
            kf = key_frame_queue[i];
        }
        catch (const std::out_of_range& e) {
            if (logger.error) logger.error("Error accessing keyframe at index " + std::to_string(i) + ": " + e.what());
            continue;
        }
        out << YAML::BeginMap << YAML::Key << "data" << YAML::Value << kf << YAML::EndMap;

        saved_count++;
    }

    out << YAML::EndSeq;

    out << YAML::Key << "angular_vel_initial_estimate" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "angular_vel_vec" << YAML::Value << YAML::BeginSeq;
    for (double v : this->angular_vel_vec) out << v;
    out << YAML::EndSeq;
    out << YAML::Key << "angular_vel_num" << YAML::Value << this->angular_vel_vec.size();
    out << YAML::EndMap;
    
    out << YAML::EndMap;

    try {
        std::ofstream fout(yaml_file_path.string());
        fout << out.c_str();
        fout.close();
        if (logger.info) logger.info("Successfully saved " + std::to_string(saved_count) + " keyframes data to YAML: " + yaml_file_path.string());
        return true;
    }
    catch (const std::exception& e) {
        if (logger.error) logger.error(std::string("Failed to save YAML file ") + yaml_file_path.string() + ": " + e.what());
        return false;
    }
}

}