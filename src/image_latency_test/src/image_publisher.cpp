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
    cv::Mat original_image = cv::imread(image_path, cv::IMREAD_COLOR);
    
    if (original_image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Impossibile caricare l'immagine: %s", image_path.c_str());
      throw std::runtime_error("Errore nel caricamento dell'immagine");
    }
    
    RCLCPP_INFO(this->get_logger(), "Immagine originale caricata: %s (dimensioni: %dx%d)", 
                image_path.c_str(), original_image.cols, original_image.rows);
    
    // Parametri per il ridimensionamento
    this->declare_parameter("width", -1);
    this->declare_parameter("height", -1);
    this->declare_parameter("scale", 1.0);
    
    int target_width = this->get_parameter("width").as_int();
    int target_height = this->get_parameter("height").as_int();
    double scale = this->get_parameter("scale").as_double();
    
    // Ridimensiona l'immagine se necessario
    if (scale != 1.0) {
      // Ridimensionamento tramite scala percentuale
      int new_width = static_cast<int>(original_image.cols * scale);
      int new_height = static_cast<int>(original_image.rows * scale);
      cv::resize(original_image, image_, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
      RCLCPP_INFO(this->get_logger(), "Immagine ridimensionata con scala %.2f: %dx%d -> %dx%d", 
                  scale, original_image.cols, original_image.rows, image_.cols, image_.rows);
    } else if (target_width > 0 && target_height > 0) {
      // Ridimensionamento con larghezza e altezza specifiche
      cv::resize(original_image, image_, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
      RCLCPP_INFO(this->get_logger(), "Immagine ridimensionata: %dx%d -> %dx%d", 
                  original_image.cols, original_image.rows, image_.cols, image_.rows);
    } else if (target_width > 0) {
      // Ridimensionamento mantenendo aspect ratio (solo larghezza specificata)
      double aspect_ratio = static_cast<double>(original_image.rows) / original_image.cols;
      target_height = static_cast<int>(target_width * aspect_ratio);
      cv::resize(original_image, image_, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
      RCLCPP_INFO(this->get_logger(), "Immagine ridimensionata (mantenendo aspect ratio): %dx%d -> %dx%d", 
                  original_image.cols, original_image.rows, image_.cols, image_.rows);
    } else if (target_height > 0) {
      // Ridimensionamento mantenendo aspect ratio (solo altezza specificata)
      double aspect_ratio = static_cast<double>(original_image.cols) / original_image.rows;
      target_width = static_cast<int>(target_height * aspect_ratio);
      cv::resize(original_image, image_, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
      RCLCPP_INFO(this->get_logger(), "Immagine ridimensionata (mantenendo aspect ratio): %dx%d -> %dx%d", 
                  original_image.cols, original_image.rows, image_.cols, image_.rows);
    } else {
      // Nessun ridimensionamento
      image_ = original_image;
      RCLCPP_INFO(this->get_logger(), "Immagine utilizzata senza ridimensionamento: %dx%d", 
                  image_.cols, image_.rows);
    }
    
    // Calcola e mostra la dimensione approssimativa del messaggio
    size_t image_size_bytes = image_.total() * image_.elemSize();
    double image_size_mb = image_size_bytes / (1024.0 * 1024.0);
    RCLCPP_INFO(this->get_logger(), "Dimensione approssimativa del messaggio: %.2f MB", image_size_mb);
    
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