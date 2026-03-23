#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <map>
#include <mutex>
#include <thread>

class CameraViewer : public rclcpp::Node {
private:
    image_transport::ImageTransport it_;
    std::map<std::string, cv::Mat> images_;
    std::mutex image_mutex_;
    std::vector<image_transport::Subscriber> subscribers_;
    int window_width_, window_height_;
    bool running_;
    std::thread display_thread_;

public:
    CameraViewer() : Node("camera_viewer"), it_(shared_from_this()) {
        this->declare_parameter<int>("window_width", 1600);
        this->declare_parameter<int>("window_height", 900);

        window_width_ = this->get_parameter("window_width").as_int();
        window_height_ = this->get_parameter("window_height").as_int();
        running_ = true;

        // Câmeras a monitorar
        std::vector<std::string> camera_topics = {
            "/uav1/bluefox_down/image_raw",
            "/uav1/bluefox_front/image_raw",
            "/uav1/bluefox_reverse/image_raw",
            "/uav1/realsense_front/color/image_raw",
            "/uav1/oak_d_front/rgb/image_raw"
        };

        for (const auto& topic : camera_topics) {
            auto sub = it_.subscribe(topic, 1,
                [this, topic](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    this->imageCallback(msg, topic);
                });
            subscribers_.push_back(sub);
            RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", topic.c_str());
        }

        cv::namedWindow("Drone Cameras", cv::WINDOW_NORMAL);
        cv::resizeWindow("Drone Cameras", window_width_, window_height_);

        display_thread_ = std::thread([this]() { this->displayLoop(); });
    }

    ~CameraViewer() {
        running_ = false;
        if (display_thread_.joinable()) {
            display_thread_.join();
        }
        cv::destroyAllWindows();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                      const std::string& topic) {
        try {
            cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                images_[topic] = std::move(cv_image);
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void displayLoop() {
        while (running_ && rclcpp::ok()) {
            std::map<std::string, cv::Mat> images_copy;
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                if (images_.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                    continue;
                }
                images_copy = images_;
            }
            cv::Mat canvas = createCanvas(images_copy);
            cv::imshow("Drone Cameras", canvas);
            int key = cv::waitKey(30) & 0xFF;
            if (key == 27) {  // ESC
                running_ = false;
                rclcpp::shutdown();
            }
        }
    }

    cv::Mat createCanvas(const std::map<std::string, cv::Mat>& images) {
        int num_images = static_cast<int>(images.size());
        int cols = (num_images <= 2) ? num_images : 2;
        int rows = (num_images + cols - 1) / cols;

        int img_width = window_width_ / cols;
        int img_height = window_height_ / rows;

        cv::Mat canvas(window_height_, window_width_, CV_8UC3, cv::Scalar(0, 0, 0));

        int idx = 0;
        for (const auto& [topic, image] : images) {
            if (image.empty()) {
                idx++;
                continue;
            }

            int row = idx / cols;
            int col = idx % cols;

            cv::Mat resized;
            cv::resize(image, resized, cv::Size(img_width, img_height));

            int y_start = row * img_height;
            int x_start = col * img_width;

            resized.copyTo(canvas(cv::Rect(x_start, y_start, img_width, img_height)));

            // Extract camera name from topic (e.g., /uav1/bluefox_down/image_raw -> bluefox_down)
            size_t last_slash = topic.find_last_of('/');
            size_t second_last_slash = topic.find_last_of('/', last_slash - 1);
            std::string camera_name = (second_last_slash != std::string::npos)
                ? topic.substr(second_last_slash + 1, last_slash - second_last_slash - 1)
                : topic;

            cv::putText(canvas, camera_name, cv::Point(x_start + 10, y_start + 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            idx++;
        }

        // Separator lines between cameras
        cv::Scalar line_color = cv::Scalar(200, 200, 200);  // Light gray
        int line_thickness = 2;

        // Vertical lines (between columns)
        for (int col = 1; col < cols; col++) {
            int x = col * img_width;
            cv::line(canvas, cv::Point(x, 0), cv::Point(x, window_height_),
                     line_color, line_thickness);
        }

        // Horizontal lines (between rows)
        for (int row = 1; row < rows; row++) {
            int y = row * img_height;
            cv::line(canvas, cv::Point(0, y), cv::Point(window_width_, y),
                     line_color, line_thickness);
        }

        return canvas;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraViewer>());
    rclcpp::shutdown();
    return 0;
}
