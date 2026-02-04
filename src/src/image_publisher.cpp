#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() : Node("drone_image_publisher"), frame_count_(0)
    {
        // Publisher oluştur
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("raw_image", 10);
        
        // Kamera aç (0 = default webcam)
        cap_.open("https:www.learningcontainer.com/wp-content/uploads/2020/05/sample-mp4-file.mp4");
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Kamera açılamadı!");
            rclcpp::shutdown();
            return;
        }
        
        // Kamera ayarları
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap_.set(cv::CAP_PROP_FPS, 30);
        
        RCLCPP_INFO(this->get_logger(), "✅ Drone Node başlatıldı - Kamera aktif");
        
        // Timer (30 FPS için ~33ms)
        timer_ = this->create_wall_timer(
            33ms,
            std::bind(&ImagePublisher::capture_and_publish, this)
        );
        
        // FPS hesaplama için
        last_time_ = this->now();
    }

private:
    void capture_and_publish()
    {
        cv::Mat frame, processed_frame;
        
        // Kameradan oku
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Boş frame!");
            return;
        }
        
        // --- IMAGE PREPROCESSING ---
        // Gaussian Blur (noise reduction)
        cv::GaussianBlur(frame, processed_frame, cv::Size(5, 5), 0);
        
        // JPEG compression için encode
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
        cv::imencode(".jpg", processed_frame, buffer, params);
        
        // Decode (simülasyon için - gerçekte compressed gönderilir)
        processed_frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
        
        // ROS2 mesajına dönüştür
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", processed_frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        
        // Yayınla
        publisher_->publish(*msg);
        
        // FPS hesapla
        frame_count_++;
        if (frame_count_ % 30 == 0) {
            auto current_time = this->now();
            double elapsed = (current_time - last_time_).seconds();
            double fps = 30.0 / elapsed;
            RCLCPP_INFO(this->get_logger(), "📊 FPS: %.1f | Frames: %ld", fps, frame_count_);
            last_time_ = current_time;
        }
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t frame_count_;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
