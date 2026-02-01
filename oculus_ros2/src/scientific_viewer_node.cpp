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

#include <oculus_ros2/sonar_viewer.hpp>

float TL_M1200d(float range, float alpha = 0.3)
{
  // the first term stand for the geometrical divergece, and the second term stands for the absorption in the water
  // theorical transmission loss should be 40log10(range) + 2.0*alpha*range;
  // here we use 10log10(range) as it was found to fit best. This difference could be explained by the amplification etc done by the sonar prior to the gain application
  return 10.0*log10(range) + 2.0*alpha*range;
}


sensor_msgs::msg::CompressedImage compressImageMsg(const sensor_msgs::msg::Image & image_msg)
{
    // Convert ROS Image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg.encoding);
    } catch (cv_bridge::Exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }

    // Encode image using OpenCV
    std::vector<uchar> compressed_data;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG2000_COMPRESSION_X1000, 100};

    std::string format = "jp2";
    if (!cv::imencode("." + format, cv_ptr->image, compressed_data, compression_params)) {
        throw std::runtime_error("Failed to compress image using OpenCV");
    }

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
    : Node("scientific_viewer"),
      sonar_viewer_(static_cast<rclcpp::Node*>(this)),
      frame_id_(this->declare_parameter<std::string>("frame_id", "sonar_frame"))
    {
      this->declare_parameter<bool>("remove_agc", false);
      this->declare_parameter<bool>("apply_tvg", false);
      this->declare_parameter<double>("gain", 3000.);

      this->remove_agc_ = this->get_parameter("remove_agc").as_bool();
      this->apply_tvg_ = this->get_parameter("apply_tvg").as_bool();
      this->gain_applied_ = this->get_parameter("gain").as_double();

      RCLCPP_INFO(this->get_logger(), "removing AGC: %s", remove_agc_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "applying TVG: %s", apply_tvg_ ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "gain: %.1f", gain_applied_);

      if (apply_tvg_ && !remove_agc_)
        RCLCPP_INFO(this->get_logger(), "Warning, if the AGC is not removed the application of the TVG is not necessary");


      ping_subscriber_ = this->create_subscription<oculus_interfaces::msg::Ping>("ping", 1, std::bind(&ScientificViewer::ping_callback, this, std::placeholders::_1));

      rtheta_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("rtheta_image", 1);
      rtheta_compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("compressed_rtheta_image", 1);

    }

  private:
    void ping_callback(const oculus_interfaces::msg::Ping & msg)
    {
      oculus_interfaces::msg::Ping ping_8_bit_msg = msg;
      ping_8_bit_msg.step = msg.n_beams ;
      ping_8_bit_msg.sample_size = 1; // 8 bit
      auto rtheta_image = sensor_msgs::msg::Image();
      rtheta_image.header = msg.header;
      rtheta_image.height = msg.n_ranges;
      rtheta_image.width = msg.n_beams;
      rtheta_image.encoding = sensor_msgs::image_encodings::MONO16;
      rtheta_image.step = 2*rtheta_image.width;
      auto ping_data = msg.ping_data;

      const int SIZE_OF_GAIN_ = 4;

      int height = msg.n_ranges;
      int width = msg.n_beams;

      const int step = 2*width + SIZE_OF_GAIN_;

      const int offset = 2048;

      float r_max= msg.range;

      std::vector<uint8_t> datas;
      std::vector<uint8_t> datas_8_bit;

      for (int i = 0; i < height; ++i)
      {
        // for each line (= 1 range)
        std::vector<uint8_t> data_line; // 1 line of data
        
        // we first read the gain

        uint8_t byte0 =  *(ping_data.begin() + offset + i * step);
        uint8_t byte1 =  *(ping_data.begin() + offset + i * step +1);
        uint8_t byte2 =  *(ping_data.begin() + offset + i * step +2);
        uint8_t byte3 =  *(ping_data.begin() + offset + i * step +3);
        uint32_t gain = (static_cast<uint32_t>(byte0)) |
                  (static_cast<uint32_t>(byte1) << 8) |
                  (static_cast<uint32_t>(byte2) << 16) |
                  (static_cast<uint32_t>(byte3) << 24);

        // We read the data of the line
        for (int j = SIZE_OF_GAIN_; j < step; j++)
        {
          auto data = *(ping_data.begin() + offset + i * step + j);
          float new_data = data;

          data_line.push_back(new_data);
        }

        // Base gain
        float gain_i = this->gain_applied_;

        // Eventually remove the gain applied by the sonar
        if (this->remove_agc_)
          gain_i/=std::sqrt(static_cast<float>(gain));

        // Compensation of transmission loss

        float r_min = 0.1; // see documentation of the sonar
        float r = r_min + ((r_max-r_min)*((float) i/(float) height));

        float alpha = 0.3; // absorption coefficient, 300dB/km at 1 Mhz

        float TL = (r<2.5) ? TL_M1200d(r, alpha) : TL_M1200d(2.5, alpha);
        float tvg_factor = pow(10.,(TL/10.));

        if (this->apply_tvg_)
          gain_i *= tvg_factor;  

        // data_line is uint8 array, we first convert it to uint16 array
        size_t n_pixels = msg.n_beams;
        std::vector<uint16_t> data_16(n_pixels);

        for (size_t index = 0; index < n_pixels; index++)
          data_16[index] =
              static_cast<uint16_t>(data_line[2*index]) |
              (static_cast<uint16_t>(data_line[2*index + 1]) << 8); // MONO16 = little-endian

        // Apply the gains on the line (!16-bit!)
        for (size_t index = 0; index < n_pixels; index++)
          {
            float new_data = static_cast<float>(data_16[index]) * gain_i;
            data_16[index] = new_data<65535 ? static_cast<uint16_t>(new_data) : 65535;
            datas_8_bit.push_back(static_cast<uint8_t>(data_16[index]/256)); // for the 8-bit version
          }

        // convert back to uint8

        std::vector<uint8_t> data_8(2 * n_pixels);

        for (size_t index = 0; index < n_pixels; index++)
        {
          data_8[2*index]     = data_16[index] & 0xFF;        // LSB
          data_8[2*index + 1] = (data_16[index] >> 8) & 0xFF; // MSB
        }
        datas.insert(datas.end(), data_8.begin(), data_8.end());
      }

      rtheta_image.data = datas;
      rtheta_publisher_->publish(rtheta_image);
      
      auto compressed_image = compressImageMsg(rtheta_image);
      rtheta_compressed_publisher_->publish(compressed_image);

      ping_8_bit_msg.ping_data = datas;
      sonar_viewer_.publishFan(msg.n_beams, msg.n_ranges, 0, datas, msg.bearings, msg.master_mode, msg.header);
    }

    bool remove_agc_;
    bool apply_tvg_;
    double gain_applied_;

    rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr ping_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rtheta_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rtheta_compressed_publisher_;

    SonarViewer sonar_viewer_;
    const std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScientificViewer>());
  rclcpp::shutdown();
  return 0;
}
