#include <atomic>
#include <filesystem>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <calousel/key_frame_acquisition.hpp>
#include <calousel/keyframe_extraction_impl.hpp>
#include <calousel/keyframe_storage.hpp>
#include <calousel/logging.hpp>
#include <calousel/utils.hpp>

namespace fs = std::filesystem;

namespace {

static calousel::CamExtractParams get_cam_params(const calousel::ExtractArgs& args, int cam_id) {
  if (static_cast<size_t>(cam_id) < args.cam_params.size()) {
    return args.cam_params[static_cast<size_t>(cam_id)];
  }
  return calousel::CamExtractParams{};
}

static void process_camera_from_queue(
    const CamConfig& cam_config,
    const BoardConfig& board_config,
    const calousel::ExtractArgs& args,
    ThreadSafeQueue<ImageMessage>& image_queue,
    std::atomic<bool>& bag_reading_done,
    const fs::path& cam_out_dir,
    const calousel::Logger& logger) {
  if (logger.info) {
    logger.info("[cam" + std::to_string(cam_config.cam_id) + "] Starting processing from queue");
  }

  const calousel::CamExtractParams cp = get_cam_params(args, cam_config.cam_id);

  ThreadSafeQueue<KeyFrame> kf_queue;
  calousel::key_frame_acquisition kfa(
      cam_config, board_config,
      cam_out_dir.string(),
      cp.frame_window_size, cp.angle_threshold, cp.rolling_shutter_compensation, cp.fps, kf_queue);

  int detection_cycle = 0;
  bool during_detection = false;
  int msg_count = 0;

  while (true) {
    ImageMessage img_msg;
    if (image_queue.try_pop_for(img_msg, 100)) {
      msg_count++;
      const bool in_fov = kfa.isBoardInFOV(img_msg.image, 4.0, cp.debug);

      if (in_fov) {
        if (!during_detection) {
          during_detection = true;
        }
        kfa.select_key_frames(CameraFrame(img_msg.image, img_msg.timestamp_ns, detection_cycle));
      } else {
        if (during_detection) {
          during_detection = false;
          detection_cycle += 1;
        }
      }
    } else {
      if (bag_reading_done && image_queue.empty()) break;
    }
  }

  if (!kfa.is_frame_window_empty()) kfa.select_key_frame_in_window();

  if (logger.info) {
    logger.info("[cam" + std::to_string(cam_config.cam_id) + "] Processed " + std::to_string(msg_count) + " messages");
  }

  if (cp.rolling_shutter_compensation) {
    kfa.compensate_rolling_shutter();
  }
  else {
    kfa.calculate_Es();
  }
  if (kfa.is_angular_vel_vec_empty()) {
    if (logger.info) logger.info("[cam" + std::to_string(cam_config.cam_id) + "] No angular velocity vector found. Substitute to temp angular vel.");
    kfa.push_back_angular_vel();
  }

  if (!kfa.save_key_frame_result(cam_out_dir.string(), logger)) {
    if (logger.error) logger.error("[cam" + std::to_string(cam_config.cam_id) + "] Failed to write keyframe_data.yaml");
  } else if (logger.info) {
    logger.info("[cam" + std::to_string(cam_config.cam_id) + "] Wrote " + std::to_string(kf_queue.size()) + " keyframes");
  }
}

}  // namespace

namespace calousel {

bool run_keyframe_extraction(const ExtractArgs& args, const Logger& logger) {
  const int cam_num = static_cast<int>(args.image_topics.size());
  BoardConfig board_config = load_board_config(args.intrinsic_yamls[0]);

  if (args.board_n_x) board_config.n_x = *args.board_n_x;
  if (args.board_n_y) board_config.n_y = *args.board_n_y;
  if (args.board_radius) board_config.r = *args.board_radius;
  if (args.board_distance) board_config.distance = *args.board_distance;
  if (args.board_asymmetric) board_config.asymmetric = *args.board_asymmetric;

  std::vector<CamConfig> cam_configs;
  cam_configs.reserve(static_cast<size_t>(cam_num));
  for (int i = 0; i < cam_num; i++) {
    cam_configs.push_back(load_cam_config(args.intrinsic_yamls[static_cast<size_t>(i)], i, args.bag_path, args.image_topics[static_cast<size_t>(i)]));
  }

  fs::path out_dir(args.keyframe_result_dir);
  fs::create_directories(out_dir);
  if (!calousel::keyframe_storage::write_metadata_yaml(out_dir, board_config, cam_configs)) {
    if (logger.warn) logger.warn("Failed to write metadata.yaml");
  }

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = args.bag_path;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);
    if (logger.info) logger.info("Opened bag file: " + args.bag_path);
  } catch (const std::exception& e) {
    if (logger.error) logger.error("Failed to open bag: " + std::string(e.what()));
    return false;
  }

  std::vector<ThreadSafeQueue<ImageMessage>> image_queues(static_cast<size_t>(cam_num));
  std::atomic<bool> bag_reading_done(false);
  std::vector<std::thread> threads;
  threads.reserve(static_cast<size_t>(cam_num));

  for (int i = 0; i < cam_num; i++) {
    fs::path cam_out = out_dir / ("cam" + std::to_string(i));
    fs::create_directories(cam_out);
    threads.emplace_back(process_camera_from_queue,
                         cam_configs[static_cast<size_t>(i)], board_config, std::cref(args),
                         std::ref(image_queues[static_cast<size_t>(i)]),
                         std::ref(bag_reading_done), cam_out, logger);
  }

  rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
  std::vector<int> msg_counts(static_cast<size_t>(cam_num), 0);

  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    int cam_idx = -1;
    for (int i = 0; i < cam_num; i++) {
      if (bag_message->topic_name == args.image_topics[static_cast<size_t>(i)]) {
        cam_idx = i;
        break;
      }
    }
    if (cam_idx < 0) continue;

    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
    try {
      serialization.deserialize_message(&extracted_serialized_msg, image_msg.get());
    } catch (const std::exception& e) {
      if (logger.warn) logger.warn("Failed to deserialize " + bag_message->topic_name + ": " + std::string(e.what()));
      continue;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      if (logger.warn) logger.warn("cv_bridge exception " + bag_message->topic_name + ": " + std::string(e.what()));
      continue;
    }

    cv::Mat gray_image;
    if (cv_ptr->image.channels() == 3) {
      cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    } else {
      gray_image = cv_ptr->image.clone();
    }

    const int64_t ts_ns = static_cast<int64_t>(image_msg->header.stamp.sec) * 1000000000LL +
                          static_cast<int64_t>(image_msg->header.stamp.nanosec);
    ImageMessage img_msg;
    img_msg.image = gray_image;
    img_msg.timestamp_ns = ts_ns;
    img_msg.cam_id = cam_idx;
    image_queues[static_cast<size_t>(cam_idx)].push(img_msg);
    msg_counts[static_cast<size_t>(cam_idx)]++;
  }

  bag_reading_done = true;
  if (logger.info) {
    for (int i = 0; i < cam_num; i++) {
      logger.info("Distributed " + std::to_string(msg_counts[static_cast<size_t>(i)]) + " messages to cam" + std::to_string(i));
    }
  }
  for (auto& t : threads) t.join();
  return true;
}

}  // namespace calousel
