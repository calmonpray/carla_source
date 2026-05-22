// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaLivoxLidarPublisher.h"

#include "carla/sensor/data/LivoxLidarData.h"

namespace carla {
namespace ros2 {

const size_t CarlaLivoxLidarPublisher::GetPointSize() {
  return sizeof(sensor::data::LivoxLidarDetection);
}

std::vector<sensor_msgs::msg::PointField> CarlaLivoxLidarPublisher::GetFields() {

    sensor_msgs::msg::PointField descriptor1;
    descriptor1.name("x");
    descriptor1.offset(0);
    descriptor1.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor1.count(1);

    sensor_msgs::msg::PointField descriptor2;
    descriptor2.name("y");
    descriptor2.offset(4);
    descriptor2.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor2.count(1);

    sensor_msgs::msg::PointField descriptor3;
    descriptor3.name("z");
    descriptor3.offset(8);
    descriptor3.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor3.count(1);

    sensor_msgs::msg::PointField descriptor4;
    descriptor4.name("intensity");
    descriptor4.offset(12);
    descriptor4.datatype(sensor_msgs::msg::PointField__FLOAT32);
    descriptor4.count(1);

    sensor_msgs::msg::PointField descriptor5;
    descriptor5.name("tag");
    descriptor5.offset(16);
    descriptor5.datatype(sensor_msgs::msg::PointField__UINT8);
    descriptor5.count(1);

    sensor_msgs::msg::PointField descriptor6;
    descriptor6.name("line");
    descriptor6.offset(17);
    descriptor6.datatype(sensor_msgs::msg::PointField__UINT8);
    descriptor6.count(1);

    sensor_msgs::msg::PointField descriptor7;
    descriptor7.name("timestamp");
    descriptor7.offset(18);
    descriptor7.datatype(sensor_msgs::msg::PointField__FLOAT64);
    descriptor7.count(1);

    return {descriptor1, descriptor2, descriptor3, descriptor4, descriptor5, descriptor6, descriptor7};
}

std::vector<uint8_t> CarlaLivoxLidarPublisher::ComputePointCloud(uint32_t height, uint32_t width, uint8_t *data) {

  sensor::data::LivoxLidarDetection* detections = reinterpret_cast<sensor::data::LivoxLidarDetection*>(data);

  const size_t total_points = height * width;
  for (size_t i = 0; i < total_points; ++i) {
    detections[i].y *= -1.0f;
  }

  const size_t total_bytes = total_points * sizeof(sensor::data::LivoxLidarDetection);
  std::vector<uint8_t> vector_data(reinterpret_cast<uint8_t*>(detections),
                                   reinterpret_cast<uint8_t*>(detections) + total_bytes);
  return vector_data;
}

}  // namespace ros2
}  // namespace carla