// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Debug.h"
#include "carla/sensor/data/Array.h"
#include "carla/sensor/data/LivoxLidarData.h"
#include "carla/sensor/s11n/LivoxLidarSerializer.h"

namespace carla {
namespace sensor {
namespace data {

  class LivoxLidarMeasurement : public Array<data::LivoxLidarDetection>
  {
    static_assert(sizeof(data::LivoxLidarDetection) == 26u, "Invalid Livox point size");

    using Super = Array<data::LivoxLidarDetection>;

  protected:

    using Serializer = s11n::LivoxLidarSerializer;

    friend Serializer;

    explicit LivoxLidarMeasurement(RawData DESERIALIZE_DECL_DATA(data))
      : Super(DESERIALIZE_MOVE_DATA(data), [](const RawData &d) {
          return Serializer::GetHeaderOffset(d);
        }) {}

  private:

    auto GetHeader() const
    {
      return Serializer::DeserializeHeader(Super::GetRawData());
    }

  public:

    auto GetHorizontalAngle() const
    {
      return GetHeader().GetHorizontalAngle();
    }

    auto GetChannelCount() const
    {
      return GetHeader().GetChannelCount();
    }

    auto GetPointCount(size_t channel) const
    {
      return GetHeader().GetPointCount(channel);
    }
  };

} // namespace data
} // namespace sensor
} // namespace carla