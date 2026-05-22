// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Util/RandomEngine.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/LivoxLidarData.h>
#include <compiler/enable-ue4-macros.h>

#include <vector>

#include "LivoxAviaLidar.generated.h"

UCLASS()
class CARLA_API ALivoxAviaLidar : public ASensor
{
  GENERATED_BODY()

  using FLivoxLidarData = carla::sensor::data::LivoxLidarData;
  using FLivoxLidarDetection = carla::sensor::data::LivoxLidarDetection;

public:

  ALivoxAviaLidar(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Set(const FActorDescription &ActorDescription) override;

  void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

private:

  struct FLivoxPatternSample
  {
    double Time = 0.0;
    float AzimuthDeg = 0.0f;
    float ElevationDeg = 0.0f;
    uint8_t Line = 0u;
  };

  struct FLivoxAviaConfig
  {
    float Range = 45000.0f;
    uint32 PointsPerSecond = 240000u;
    float AtmosphereAttenuationRate = 0.004f;
    int RandomSeed = 0;

    float DropOffGeneralRate = 0.0f;
    float DropOffIntensityLimit = 0.0f;
    float DropOffAtZeroIntensity = 0.0f;
    float NoiseStdDev = 0.0f;

    float DefaultIntensity = 100.0f;
    float PatternDuration = 4.0f;
    bool LoopPattern = true;
    bool RelativeTimestamp = false;

    int MaxPointsPerTick = 20000;

    FString CsvPath;
  };

  void LoadPatternCsv();
  bool ParseCsvLine(const FString &Line, FLivoxPatternSample &OutSample) const;

  void SimulateLivoxAvia(float DeltaTime);

  bool ShootLivoxSample(
      const FLivoxPatternSample &Sample,
      double PointTimestamp,
      FLivoxLidarDetection &OutDetection);

  float ComputeIntensity(const FVector &SensorPoint) const;
  bool PostprocessDetection(FLivoxLidarDetection &Detection) const;

private:

  FLivoxAviaConfig Config;
  FLivoxLidarData LivoxLidarData;
  std::vector<FLivoxPatternSample> Pattern;
  uint32 PatternIndex = 0u;
  std::vector<uint32_t> PointsPerChannel;

  bool DropOffGenActive = false;
  float DropOffAlpha = 0.0f;
  float DropOffBeta = 1.0f;

  UPROPERTY()
  URandomEngine *RandomEngine = nullptr;
};