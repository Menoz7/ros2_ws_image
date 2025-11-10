#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber() : Node("image_subscriber"), count_(0), total_latency_ms_(0.0)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image", 10, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Image Subscriber avviato");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Calcola la latenza
    auto now = this->now();
    auto publish_time = rclcpp::Time(msg->header.stamp);
    auto latency = (now - publish_time).seconds() * 1000.0; // Converti in millisecondi
    
    // Aggiorna statistiche
    count_++;
    total_latency_ms_ += latency;
    double avg_latency = total_latency_ms_ / count_;
    
    // Aggiorna min e max
    if (count_ == 1) {
      min_latency_ms_ = latency;
      max_latency_ms_ = latency;
    } else {
      if (latency < min_latency_ms_) min_latency_ms_ = latency;
      if (latency > max_latency_ms_) max_latency_ms_ = latency;
    }
    
    // Log ogni 30 frame
    if (count_ % 30 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Frame #%ld | Latenza: %.2f ms | Media: %.2f ms | Min: %.2f ms | Max: %.2f ms",
                  count_, latency, avg_latency, min_latency_ms_, max_latency_ms_);
    }
    
    // Opzionale: visualizza l'immagine con OpenCV
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      
      // Aggiungi informazioni sulla latenza all'immagine
      std::string latency_text = "Latenza: " + std::string(latency < 10 ? " " : "") + 
                                 std::to_string(latency).substr(0, 5) + " ms";
      cv::putText(cv_ptr->image, latency_text, cv::Point(50, 50), 
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
      
      std::string avg_text = "Media: " + std::to_string(avg_latency).substr(0, 5) + " ms";
      cv::putText(cv_ptr->image, avg_text, cv::Point(50, 100), 
                  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
      
      // Mostra l'immagine
      cv::imshow("Image Subscriber", cv_ptr->image);
      cv::waitKey(1);
      
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  size_t count_;
  double total_latency_ms_;
  double min_latency_ms_;
  double max_latency_ms_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return 0;
}