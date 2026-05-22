// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/sensor/s11n/LivoxLidarSerializer.h"

#include "carla/sensor/SensorData.h"
#include "carla/sensor/data/LivoxLidarMeasurement.h"

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> LivoxLidarSerializer::Deserialize(RawData &&data)
  {
    return SharedPtr<SensorData>(
        new sensor::data::LivoxLidarMeasurement(std::move(data)));
  }

} // namespace s11n
} // namespace sensor
} // namespace carla