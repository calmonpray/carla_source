// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <cmath>
#include <limits>

#include "Carla.h"
#include "Carla/Sensor/LivoxAviaLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/ros2/ROS2.h"
#include <compiler/enable-ue4-macros.h>

#include "Engine/CollisionProfile.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

FActorDefinition ALivoxAviaLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("livox_avia"));
}

ALivoxAviaLidar::ALivoxAviaLidar(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Config.RandomSeed);
}

void ALivoxAviaLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);

  constexpr float TO_CENTIMETERS = 100.0f;

  const auto &Vars = ActorDescription.Variations;

  Config.Range =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          TEXT("range"), Vars, 450.0f) * TO_CENTIMETERS;

  Config.PointsPerSecond = static_cast<uint32>(FMath::Max(
      0,
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(
          TEXT("points_per_second"), Vars, 240000)));

  Config.AtmosphereAttenuationRate =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          TEXT("atmosphere_attenuation_rate"), Vars, 0.004f);

  Config.RandomSeed =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(
          TEXT("noise_seed"), Vars, 0);

  Config.DropOffGeneralRate =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          TEXT("dropoff_general_rate"), Vars, 0.0f);

  Config.DropOffIntensityLimit =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          TEXT("dropoff_intensity_limit"), Vars, 0.0f);

  Config.DropOffAtZeroIntensity =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          TEXT("dropoff_zero_intensity"), Vars, 0.0f);

  Config.NoiseStdDev =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          TEXT("noise_stddev"), Vars, 0.0f);

  Config.DefaultIntensity =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          TEXT("default_intensity"), Vars, 100.0f);

  Config.PatternDuration =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
          TEXT("pattern_duration"), Vars, 4.0f);

  Config.LoopPattern =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToBool(
          TEXT("loop_pattern"), Vars, true);

  Config.RelativeTimestamp =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToBool(
          TEXT("relative_timestamp"), Vars, false);

  Config.MaxPointsPerTick = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(
      TEXT("max_points_per_tick"), Vars, 20000);

  Config.CsvPath =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
          TEXT("csv_path"), Vars, TEXT(""));

  if (Config.CsvPath.IsEmpty())
  {
    Config.CsvPath = FPaths::Combine(FPaths::ProjectDir(), TEXT("LivoxCsv/avia.csv"));
  }

  SetSeed(Config.RandomSeed);

  LivoxLidarData = FLivoxLidarData(1u);
  PointsPerChannel.resize(1u);

  DropOffBeta = 1.0f - Config.DropOffAtZeroIntensity;
  if (Config.DropOffIntensityLimit > std::numeric_limits<float>::epsilon())
  {
    DropOffAlpha = Config.DropOffAtZeroIntensity / Config.DropOffIntensityLimit;
  }
  else
  {
    DropOffAlpha = 0.0f;
  }
  DropOffGenActive = Config.DropOffGeneralRate > std::numeric_limits<float>::epsilon();

  LoadPatternCsv();
}

void ALivoxAviaLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ALivoxAviaLidar::PostPhysTick);

  static_cast<void>(World);
  static_cast<void>(TickType);

  SimulateLivoxAvia(DeltaTime);

  auto DataStream = GetDataStream(*this);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Livox Avia Stream");
    DataStream.SerializeAndSend(*this, LivoxLidarData, DataStream.PopBufferFromPool());
  }

#if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send Livox Avia");
    AActor *ParentActor = GetAttachParentActor();
    auto Transform = (ParentActor)
        ? GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform())
        : GetActorTransform();
    ROS2->ProcessDataFromLivoxLidar(DataStream.GetSensorType(), Transform, LivoxLidarData, this);
  }
#endif
}

void ALivoxAviaLidar::LoadPatternCsv()
{
  Pattern.clear();
  PatternIndex = 0u;

  TArray<FString> Lines;
  if (!FFileHelper::LoadFileToStringArray(Lines, *Config.CsvPath))
  {
    UE_LOG(LogCarla, Error, TEXT("LivoxAviaLidar: failed to read csv_path=%s"), *Config.CsvPath);
    return;
  }

  Pattern.reserve(static_cast<size_t>(Lines.Num()));

  for (const FString &Line : Lines)
  {
    FString Trimmed = Line;
    Trimmed.TrimStartAndEndInline();

    if (Trimmed.IsEmpty())
    {
      continue;
    }

    if (Trimmed.StartsWith(TEXT("time")) || Trimmed.StartsWith(TEXT("#")))
    {
      continue;
    }

    FLivoxPatternSample Sample;
    if (ParseCsvLine(Trimmed, Sample))
    {
      Pattern.emplace_back(Sample);
    }
  }

  if (Pattern.empty())
  {
    UE_LOG(LogCarla, Error, TEXT("LivoxAviaLidar: CSV loaded but no valid samples: %s"), *Config.CsvPath);
  }
  else
  {
    UE_LOG(LogCarla, Log, TEXT("LivoxAviaLidar: loaded %d samples from %s"),
        static_cast<int32>(Pattern.size()), *Config.CsvPath);
  }
}

bool ALivoxAviaLidar::ParseCsvLine(const FString &Line, FLivoxPatternSample &OutSample) const
{
  TArray<FString> Parts;
  Line.ParseIntoArray(Parts, TEXT(","), true);

  if (Parts.Num() < 4)
  {
    return false;
  }

  for (FString &Part : Parts)
  {
    Part.TrimStartAndEndInline();
  }

  const double Time = FCString::Atod(*Parts[0]);
  const float Azimuth = FCString::Atof(*Parts[1]);

  const float Zenith = FCString::Atof(*Parts[2]);
  const float Elevation = Zenith - 90.0f;

  const int32 LineIndex = FCString::Atoi(*Parts[3]);

  OutSample.Time = Time;
  OutSample.AzimuthDeg = Azimuth;
  OutSample.ElevationDeg = Elevation;
  OutSample.Line = static_cast<uint8_t>(FMath::Clamp(LineIndex, 0, 255));

  return true;
}

void ALivoxAviaLidar::SimulateLivoxAvia(float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ALivoxAviaLidar::SimulateLivoxAvia);

  std::vector<FLivoxLidarDetection> Detections;

  if (Pattern.empty())
  {
    PointsPerChannel[0] = 0u;
    LivoxLidarData.ResetMemory(PointsPerChannel);
    LivoxLidarData.SetHorizontalAngle(0.0f);
    LivoxLidarData.WriteChannelCount(PointsPerChannel);
    return;
  }

  uint32 PointsThisTick = FMath::RoundHalfFromZero(static_cast<float>(Config.PointsPerSecond) * DeltaTime);
  PointsThisTick = static_cast<uint32>(FMath::Clamp(
      static_cast<int32>(PointsThisTick),
      0,
      FMath::Max(0, Config.MaxPointsPerTick)));

  Detections.reserve(PointsThisTick);

  const double WorldTime = GetWorld() ? static_cast<double>(GetWorld()->GetTimeSeconds()) : 0.0;

  for (uint32 i = 0u; i < PointsThisTick; ++i)
  {
    if (PatternIndex >= Pattern.size())
    {
      if (Config.LoopPattern)
      {
        PatternIndex = 0u;
      }
      else
      {
        break;
      }
    }

    const auto &Sample = Pattern[PatternIndex++];

    const double PointTimestamp = Config.RelativeTimestamp
        ? Sample.Time
        : WorldTime + (static_cast<double>(i) / static_cast<double>(FMath::Max(1u, PointsThisTick))) * DeltaTime;

    FLivoxLidarDetection Detection;
    if (ShootLivoxSample(Sample, PointTimestamp, Detection))
    {
      if (PostprocessDetection(Detection))
      {
        Detections.emplace_back(Detection);
      }
    }
  }

  PointsPerChannel[0] = static_cast<uint32_t>(Detections.size());
  LivoxLidarData.ResetMemory(PointsPerChannel);

  for (const auto &Detection : Detections)
  {
    LivoxLidarData.WritePointSync(Detection);
  }

  LivoxLidarData.SetHorizontalAngle(0.0f);
  LivoxLidarData.WriteChannelCount(PointsPerChannel);
}

bool ALivoxAviaLidar::ShootLivoxSample(
    const FLivoxPatternSample &Sample,
    double PointTimestamp,
    FLivoxLidarDetection &OutDetection)
{
  if (GetWorld() == nullptr)
  {
    return false;
  }

  FHitResult HitInfo(ForceInit);

  FCollisionQueryParams TraceParams(FName(TEXT("LivoxAvia_Trace")), true, this);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;

  const FTransform ActorTransf = GetTransform();
  const FVector LidarBodyLoc = ActorTransf.GetLocation();
  const FRotator LidarBodyRot = ActorTransf.Rotator();

  const FRotator LaserRot(Sample.ElevationDeg, Sample.AzimuthDeg, 0.0f);
  const FRotator ResultRot = UKismetMathLibrary::ComposeRotators(LaserRot, LidarBodyRot);

  const FVector EndTrace = Config.Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  if (!GetWorld()->LineTraceSingleByChannel(
          HitInfo,
          LidarBodyLoc,
          EndTrace,
          ECC_GameTraceChannel2,
          TraceParams,
          FCollisionResponseParams::DefaultResponseParam))
  {
    return false;
  }

  const FVector SensorPoint = ActorTransf.Inverse().TransformPosition(HitInfo.ImpactPoint);

  OutDetection.x = SensorPoint.X;
  OutDetection.y = SensorPoint.Y;
  OutDetection.z = SensorPoint.Z;
  OutDetection.intensity = ComputeIntensity(SensorPoint);
  OutDetection.tag = 0u;
  OutDetection.line = Sample.Line;
  OutDetection.timestamp = PointTimestamp;

  return true;
}

float ALivoxAviaLidar::ComputeIntensity(const FVector &SensorPoint) const
{
  const float DistanceCm = SensorPoint.Size();
  const float DistanceM = DistanceCm / 100.0f;
  const float AbsAtm = std::exp(-Config.AtmosphereAttenuationRate * DistanceM);

  return FMath::Clamp(Config.DefaultIntensity * AbsAtm, 0.0f, 255.0f);
}

bool ALivoxAviaLidar::PostprocessDetection(FLivoxLidarDetection &Detection) const
{
  if (DropOffGenActive && RandomEngine->GetUniformFloat() < Config.DropOffGeneralRate)
  {
    return false;
  }

  if (Config.NoiseStdDev > std::numeric_limits<float>::epsilon())
  {
    FVector P(Detection.x, Detection.y, Detection.z);
    const FVector ForwardVector = P.GetSafeNormal();
    const FVector Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Config.NoiseStdDev);
    P += Noise;
    Detection.x = P.X;
    Detection.y = P.Y;
    Detection.z = P.Z;
  }

  if (Config.DropOffIntensityLimit <= std::numeric_limits<float>::epsilon())
  {
    return true;
  }

  if (Detection.intensity > Config.DropOffIntensityLimit)
  {
    return true;
  }

  return RandomEngine->GetUniformFloat() < DropOffAlpha * Detection.intensity + DropOffBeta;
}