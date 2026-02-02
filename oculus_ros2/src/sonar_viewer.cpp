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

#include <oculus_ros2/sonar_viewer.hpp>

std::vector<float> bearings_in_rad(const std::vector<int16_t>& bearings) {
  std::vector<float> bearings_in_rad;
  bearings_in_rad.reserve(bearings.size());
  for (const auto& bearing : bearings) 
    bearings_in_rad.push_back(static_cast<float>(bearing) * M_PI / 18000.0f);
  return bearings_in_rad;
}

SonarViewer::SonarViewer(rclcpp::Node* node) : node_(node) {
  image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("image_cartesian", 1);
}

SonarViewer::~SonarViewer() {}

void SonarViewer::publishFan(const int& width,
    const int& height,
    const int& offset,
    const std::vector<uint8_t>& ping_data,
    const std::vector<int16_t>& bearings,
    const int& master_mode,
    const std_msgs::msg::Header& header) const {
  const int step = 2*width;
  const float theta_shift = 270;
  const int mat_encoding = CV_16U;
  const char* ros_image_encoding = sensor_msgs::image_encodings::MONO16;
  const auto bearings_rad = bearings_in_rad(bearings);

  const double bearing = bearings_rad.back();
  const float bearing_ratio = 2 * bearing / width;
  const int negative_height = static_cast<int>(std::floor(height * std::sin(-bearing)));
  const int positive_height = static_cast<int>(std::ceil(height * std::sin(bearing)));
  const int image_width = positive_height - negative_height;
  const int origin_width = abs(negative_height);  // x coordinate of the origin
  const cv::Size image_size(image_width, height);
  cv::Mat map(image_size, CV_32FC2);
  cv::parallel_for_(cv::Range(0, map.total()), [&](const cv::Range& range) {
    for (auto i = range.start; i < range.end; i++) {
      int y = i / map.cols;
      int x = i % map.cols;

      // Calculate range and bearing of this pixel from origin
      const float dx = x - origin_width;
      const float dy = map.rows - y;

      const float range = sqrt(dx * dx + dy * dy);
      const float bearing_x_y = atan2(dx, dy);


      float xp = range;
      
      auto it = std::lower_bound(bearings_rad.begin(), bearings_rad.end(), bearing_x_y);
      float col;      
      if (it == bearings_rad.begin())
        if (bearing_x_y < bearings_rad.front())
          col = -1.0f;
        else
          col = 0.0f;
      else if (it == bearings_rad.end())
        col = -1.0f;
      else {
        int k = int(it - bearings_rad.begin()) - 1;
        float b0 = bearings_rad[k];
        float b1 = bearings_rad[k + 1];

        float alpha = (bearing_x_y - b0) / (b1 - b0);
        col = k + alpha;
      }

      float yp = col;

      map.at<cv::Vec2f>(cv::Point(x, y)) = cv::Vec2f(xp, yp);
    }
  });

  cv::Mat source_map_1, source_map_2;
  cv::convertMaps(map, cv::Mat(), source_map_1, source_map_2, CV_16SC2);

  cv::Mat sonar_mat_data(height, step, mat_encoding);  // Note that the width is 'step' to include gain data
  for (int i = 0; i < height; ++i)
    std::copy(ping_data.begin() + offset + i * step, ping_data.begin() + offset + (i + 1) * step, sonar_mat_data.ptr<uint8_t>(i));

  cv::Mat out = cv::Mat::ones(cv::Size(image_width, height), CV_MAKETYPE(mat_encoding, 1)) * std::numeric_limits<uint8_t>::max();
  cv::remap(sonar_mat_data.t(), out, source_map_1, source_map_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT,
      cv::Scalar(std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint16_t>::max(), std::numeric_limits<uint16_t>::max()));

  // Publish sonar conic image
  sensor_msgs::msg::Image msg;
  cv_bridge::CvImage(header, ros_image_encoding, out).toImageMsg(msg);
  image_publisher_->publish(msg);
}
