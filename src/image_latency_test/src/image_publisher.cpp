#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher(const std::string& image_path) : Node("image_publisher"), count_(0)
  {
    // Carica l'immagine dal file
    image_ = cv::imread(image_path, cv::IMREAD_COLOR);
    
    if (image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Impossibile caricare l'immagine: %s", image_path.c_str());
      throw std::runtime_error("Errore nel caricamento dell'immagine");
    }
    
    RCLCPP_INFO(this->get_logger(), "Immagine caricata: %s (dimensioni: %dx%d)", 
                image_path.c_str(), image_.cols, image_.rows);
    
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
    
    // Dichiara parametro per la frequenza di pubblicazione (default 30 Hz)
    this->declare_parameter("publish_rate", 30.0);
    double rate = this->get_parameter("publish_rate").as_double();
    
    int period_ms = static_cast<int>(1000.0 / rate);
    
    RCLCPP_INFO(this->get_logger(), "Frequenza di pubblicazione: %.1f Hz", rate);
    
    // Timer per pubblicare immagini
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms), 
      std::bind(&ImagePublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Image Publisher avviato");
  }

private:
  void timer_callback()
  {
    // Crea una copia dell'immagine per aggiungere informazioni
    cv::Mat image_to_send = image_.clone();
    
    // Aggiungi il numero del frame sull'immagine
    std::string text = "Frame: " + std::to_string(count_);
    cv::putText(image_to_send, text, cv::Point(20, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    
    // Converti l'immagine OpenCV in messaggio ROS2
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_to_send).toImageMsg();
    
    // Imposta il timestamp corrente
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_frame";
    
    // Pubblica l'immagine
    publisher_->publish(*msg);
    
    if (count_ % 30 == 0) {
      RCLCPP_INFO(this->get_logger(), "Pubblicata immagine #%ld", count_);
    }
    
    count_++;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat image_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Verifica che sia stato fornito il percorso dell'immagine
  if (argc < 2) {
    std::cerr << "Uso: ros2 run image_latency_test image_publisher <percorso_immagine>" << std::endl;
    std::cerr << "Esempio: ros2 run image_latency_test image_publisher /path/to/image.jpg" << std::endl;
    return 1;
  }
  
  std::string image_path = argv[1];
  
  try {
    auto node = std::make_shared<ImagePublisher>(image_path);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cerr << "Errore: " << e.what() << std::endl;
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}