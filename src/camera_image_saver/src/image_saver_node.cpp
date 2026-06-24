#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <filesystem>

class ImageSaverNode : public rclcpp::Node {
public:
    ImageSaverNode() : Node("image_saver_node") {
        this->declare_parameter<std::string>("save_directory", "captured_images");
        this->declare_parameter<std::string>("topic_name", "/cam_0/image_raw");
        save_directory_ = this->get_parameter("save_directory").as_string();
        topic_name_ = this->get_parameter("topic_name").as_string();

        if (!std::filesystem::exists(save_directory_)) {
            if (std::filesystem::create_directories(save_directory_)) {
                RCLCPP_INFO(this->get_logger(), "Created save directory: %s", save_directory_.c_str());
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to create save directory: %s", save_directory_.c_str());
            }
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Save directory already exists: %s", save_directory_.c_str());
        }

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name_,
            rclcpp::QoS(10).best_effort(),
            std::bind(&ImageSaverNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ImageSaverNode has been started. Subscribing to /camera/image_raw.");
        RCLCPP_INFO(this->get_logger(), "Press 's' to save a frame. Press 'q' to quit.");

        // OpenCV 창 이름 설정
        cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow("Camera Feed", cv_ptr->image);

        int key = cv::waitKey(1);

        if (key == 's' || key == 'S') {
            auto now = std::chrono::high_resolution_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            std::string filename = save_directory_ + "/frame_" + std::to_string(timestamp) + ".png";

            if (cv::imwrite(filename, cv_ptr->image)) {
                RCLCPP_INFO(this->get_logger(), "Saved frame to: %s", filename.c_str());
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image to: %s", filename.c_str());
            }
        }
        else if (key == 'q' || key == 'Q') {
            RCLCPP_INFO(this->get_logger(), "Quitting ImageSaverNode.");
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    std::string save_directory_, topic_name_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}