// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Debug.h"
#include "carla/sensor/data/SemanticLidarData.h"

#include <cstdint>
#include <cstring>
#include <numeric>
#include <ostream>
#include <vector>

namespace carla {

namespace ros2 {
  class ROS2;
}

namespace sensor {

namespace s11n {
  class LivoxLidarSerializer;
  class LivoxLidarHeaderView;
}

namespace data {

#pragma pack(push, 1)
  struct LivoxLidarDetection
  {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t tag;
    uint8_t line;
    double timestamp;

    LivoxLidarDetection()
      : x(0.0f), y(0.0f), z(0.0f), intensity(0.0f), tag(0u), line(0u), timestamp(0.0) {}

    LivoxLidarDetection(
        float InX,
        float InY,
        float InZ,
        float InIntensity,
        uint8_t InTag,
        uint8_t InLine,
        double InTimestamp)
      : x(InX), y(InY), z(InZ), intensity(InIntensity), tag(InTag), line(InLine), timestamp(InTimestamp) {}

    void WritePlyHeaderInfo(std::ostream &out) const
    {
      out << "property float32 x\n"
          "property float32 y\n"
          "property float32 z\n"
          "property float32 intensity\n"
          "property uint8 tag\n"
          "property uint8 line\n"
          "property float64 timestamp";
    }

    void WriteDetection(std::ostream &out) const
    {
      out << x << ' ' << y << ' ' << z << ' ' << intensity << ' '
          << static_cast<uint32_t>(tag) << ' '
          << static_cast<uint32_t>(line) << ' '
          << timestamp;
    }
  };
#pragma pack(pop)

  static_assert(sizeof(LivoxLidarDetection) == 26u, "LivoxLidarDetection must match PointXYZRTLT packed layout");

  class LivoxLidarData : public SemanticLidarData
  {
  public:

    explicit LivoxLidarData(uint32_t ChannelCount = 1u)
      : SemanticLidarData(ChannelCount) {}

    LivoxLidarData &operator=(LivoxLidarData &&) = default;

    ~LivoxLidarData() = default;

    virtual void ResetMemory(std::vector<uint32_t> points_per_channel)
    {
      DEBUG_ASSERT(GetChannelCount() >= points_per_channel.size());

      std::memset(
          _header.data() + Index::SIZE,
          0,
          sizeof(uint32_t) * GetChannelCount());

      uint32_t total_points = static_cast<uint32_t>(
          std::accumulate(points_per_channel.begin(), points_per_channel.end(), 0u));

      _points.clear();
      _points.reserve(total_points);
    }

    void WritePointSync(const LivoxLidarDetection &detection)
    {
      _points.emplace_back(detection);
    }

    virtual void WritePointSync(SemanticLidarDetection &detection)
    {
      (void)detection;
      DEBUG_ASSERT(false);
    }

    const std::vector<LivoxLidarDetection> &GetDetections() const
    {
      return _points;
    }

  private:

    std::vector<LivoxLidarDetection> _points;

    friend class s11n::LivoxLidarSerializer;
    friend class s11n::LivoxLidarHeaderView;
    friend class carla::ros2::ROS2;
  };

} // namespace data
} // namespace sensor
} // namespace carla