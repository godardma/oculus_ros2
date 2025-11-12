/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, ENSTA-Bretagne
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <oculus_interfaces/msg/ping.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


sensor_msgs::msg::CompressedImage compressImageMsg(const sensor_msgs::msg::Image & image_msg)
{
    // Convert ROS Image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg.encoding);
    } catch (cv_bridge::Exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }

    // Encode image using OpenCV (e.g., JPEG)
    std::vector<uchar> compressed_data;
    // std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 95};
    // std::vector<int> compression_params = {cv::IMWRITE_JPEG2000_COMPRESSION_X250, 250};
    std::vector<int> compression_params = {cv::IMWRITE_JPEG2000_COMPRESSION_X1000, 100};

    std::string format = "jp2";  // You can make this configurable
    if (!cv::imencode("." + format, cv_ptr->image, compressed_data, compression_params)) {
        throw std::runtime_error("Failed to compress image using OpenCV");
    }

    // Create CompressedImage message
    sensor_msgs::msg::CompressedImage compressed_msg;
    compressed_msg.header = image_msg.header;
    compressed_msg.format = format;
    compressed_msg.data = std::move(compressed_data);

    return compressed_msg;
}

class ScientificViewer : public rclcpp::Node
{
  public:
    ScientificViewer()
    : Node("scientific_viewer")
    {
      this->declare_parameter<bool>("remove_cag", false);
      this->declare_parameter<bool>("apply_tvg", false);
      this->declare_parameter<double>("gain", 3000.);

      this->remove_cag_ = this->get_parameter("remove_cag").as_bool();
      this->apply_tvg_ = this->get_parameter("apply_tvg").as_bool();
      this->gain_applied_ = this->get_parameter("gain").as_double();

      RCLCPP_INFO(this->get_logger(), "removing CAG: %s", remove_cag_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "applying TVG: %s", apply_tvg_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "gain: %.1f", gain_applied_);

      if (apply_tvg_ && !remove_cag_)
        RCLCPP_INFO(this->get_logger(), "Warning, if the CAG is not removed the application of the TVG is not necessary");


      ping_subscriber_ = this->create_subscription<oculus_interfaces::msg::Ping>("ping", 1, std::bind(&ScientificViewer::ping_callback, this, std::placeholders::_1));

      rtheta_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rtheta_image", 1);
      rtheta_compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("compressed_rtheta_image", 1);

    }

  private:
    void ping_callback(const oculus_interfaces::msg::Ping & msg)
    {
      auto rtheta_image = sensor_msgs::msg::Image();
      rtheta_image.header = msg.header;
      rtheta_image.height = msg.n_ranges;
      rtheta_image.width = msg.n_beams;
      rtheta_image.encoding = sensor_msgs::image_encodings::MONO16;
      rtheta_image.step = 2*rtheta_image.width;
      std::vector<uint8_t> datas;
      auto ping_data = msg.ping_data;

      const int SIZE_OF_GAIN_ = 4;

      int height = msg.n_ranges;
      int width = msg.n_beams;

      const int step = 2*width + SIZE_OF_GAIN_;

      const int offset = 2048;

      float r_max= msg.range;

      for (int i = 0; i < height; ++i)
      {
        uint8_t byte0 =  *(ping_data.begin() + offset + i * step);
        uint8_t byte1 =  *(ping_data.begin() + offset + i * step +1);
        uint8_t byte2 =  *(ping_data.begin() + offset + i * step +2);
        uint8_t byte3 =  *(ping_data.begin() + offset + i * step +3);
        uint32_t gain = (static_cast<uint32_t>(byte0)) |
                  (static_cast<uint32_t>(byte1) << 8) |
                  (static_cast<uint32_t>(byte2) << 16) |
                  (static_cast<uint32_t>(byte3) << 24);

        // 3000. is a chosen gain
        // set gain_i to a constant to keep the CAG
        float gain_i = this->gain_applied_;

        if (this->remove_cag_)
          gain_i/=std::sqrt(static_cast<float>(gain));

        // Compensation of the energy diffusion depending on the range
        float tvg_factor = 40.0*log(1+r_max*((float) i/(float) height));

        if (this->apply_tvg_)
          gain_i *= tvg_factor/90.;        

        for (int j = SIZE_OF_GAIN_; j < step; j++)
        {
          auto data = *(ping_data.begin() + offset + i * step + j);
          float new_data = data;
          
          new_data = new_data * gain_i ; // no CAG

          if (new_data>255)
            new_data=255.;
          datas.push_back(new_data);  // no CAG
        }
      }
      rtheta_image.data = datas;
      rtheta_publisher_->publish(rtheta_image);
      
      auto compressed_image = compressImageMsg(rtheta_image);
      rtheta_compressed_publisher_->publish(compressed_image);
    }

    bool remove_cag_;
    bool apply_tvg_;
    double gain_applied_;

    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr ping_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rtheta_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rtheta_compressed_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScientificViewer>());
  rclcpp::shutdown();
  return 0;
}
