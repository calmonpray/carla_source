# CarlaAir v0.1.7 添加 Livox Avia LiDAR 传感器：Codex 直接改动说明

目标环境：

- CarlaAir v0.1.7
- CARLA 0.9.16 代码结构
- Ubuntu 22.04
- ROS 2 Humble
- 新传感器类名：`LivoxAviaLidar`
- 新 blueprint id：`sensor.lidar.livox_avia`
- 真实扫描模式：Livox Avia 非重复扫描角度序列，由 CSV 驱动
- Unreal 内部测距方式：对每个 Avia CSV 角度样本执行 UE line trace 求交。注意：这是仿真测距实现，不代表 Avia 是传统旋转 ray-cast LiDAR；类名和 blueprint 不使用 `RayCast`。

---

## 0. 文件名修正

不要使用：

```text
RayCastLivoxLidar.h
RayCastLivoxLidar.cpp
```

改成：

```text
LivoxAviaLidar.h
LivoxAviaLidar.cpp
```

最终新增文件：

```text
# 必须新增：UE sensor actor
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.h
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.cpp

# 必须新增：LibCarla 数据结构与序列化
LibCarla/source/carla/sensor/data/LivoxLidarData.h
LibCarla/source/carla/sensor/data/LivoxLidarMeasurement.h
LibCarla/source/carla/sensor/s11n/LivoxLidarSerializer.h
LibCarla/source/carla/sensor/s11n/LivoxLidarSerializer.cpp

# ROS2 直接发布时新增
LibCarla/source/carla/ros2/publishers/CarlaLivoxLidarPublisher.h
LibCarla/source/carla/ros2/publishers/CarlaLivoxLidarPublisher.cpp

# 可选示例
PythonAPI/examples/livox_avia_spawn.py
```

必须修改文件：

```text
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/ActorBlueprintFunctionLibrary.cpp
LibCarla/source/carla/sensor/SensorRegistry.h
PythonAPI/carla/source/libcarla/SensorData.cpp
PythonAPI/carla/source/carla/libcarla.pyi

# ROS2 直接发布时修改
LibCarla/source/carla/ros2/ROS2.h
LibCarla/source/carla/ros2/ROS2.cpp
```

---

## 1. 新增：`Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.h`

新建文件，内容如下：

```cpp
#pragma once

#include "Carla/Sensor/Sensor.h"
#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Util/RandomEngine.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/LivoxLidarData.h>
#include <compiler/enable-ue4-macros.h>

#include <vector>
#include <string>

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
    // CARLA / UE distance follows CARLA lidar convention: range attribute is meters,
    // stored internally as centimeters after Set().
    float Range = 45000.0f;                 // cm, default 450 m
    uint32 PointsPerSecond = 240000u;       // Avia typical order of magnitude
    float AtmosphereAttenuationRate = 0.004f;
    int RandomSeed = 0;

    float DropOffGeneralRate = 0.0f;
    float DropOffIntensityLimit = 0.0f;
    float DropOffAtZeroIntensity = 0.0f;
    float NoiseStdDev = 0.0f;

    float DefaultIntensity = 100.0f;        // 0..255 pseudo reflectivity
    float PatternDuration = 4.0f;           // Livox official CSV often covers 4 s
    bool LoopPattern = true;
    bool RelativeTimestamp = false;         // false: publish sim-time-like point timestamps

    int MaxPointsPerTick = 20000;

    FString CsvPath;
  };

private:

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
```

---

## 2. 新增：`Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.cpp`

新建文件，内容如下：

```cpp
#include "Carla.h"
#include "Carla/Sensor/LivoxAviaLidar.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "HAL/PlatformFilemanager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace crp = carla::rpc;

FActorDefinition ALivoxAviaLidar::GetSensorDefinition()
{
  // Produces blueprint id: sensor.lidar.livox_avia
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

  Config.PointsPerSecond =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(
          TEXT("points_per_second"), Vars, 240000);

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

  Config.MaxPointsPerTick =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(
          TEXT("max_points_per_tick"), Vars, 20000);

  Config.CsvPath =
      UActorBlueprintFunctionLibrary::RetrieveActorAttributeToString(
          TEXT("csv_path"), Vars, TEXT(""));

  if (Config.CsvPath.IsEmpty())
  {
    // Default location relative to packaged/project dir.
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

  SimulateLivoxAvia(DeltaTime);

  auto DataStream = GetDataStream(*this);
  auto SensorTransform = DataStream.GetSensorTransform();

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

  for (const FString &Line : Lines)
  {
    FString Trimmed = Line;
    Trimmed.TrimStartAndEndInline();

    if (Trimmed.IsEmpty())
    {
      continue;
    }

    // Skip header if present.
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

  const double Time = FCString::Atod(*Parts[0]);
  const float Azimuth = FCString::Atof(*Parts[1]);

  // CSV convention expected:
  // column 2 is zenith angle in degrees, so elevation = zenith - 90.
  // If your CSV already stores elevation, change this line accordingly.
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
    LivoxLidarData.WriteChannelCount(PointsPerChannel);
    return;
  }

  uint32 PointsThisTick = FMath::RoundHalfFromZero(Config.PointsPerSecond * DeltaTime);
  PointsThisTick = static_cast<uint32>(FMath::Clamp(
      static_cast<int32>(PointsThisTick),
      0,
      Config.MaxPointsPerTick));

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
        : WorldTime + (static_cast<double>(i) / FMath::Max(1u, PointsThisTick)) * DeltaTime;

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

  for (auto &Detection : Detections)
  {
    LivoxLidarData.WritePointSync(Detection);
  }

  LivoxLidarData.WriteChannelCount(PointsPerChannel);
}

bool ALivoxAviaLidar::ShootLivoxSample(
    const FLivoxPatternSample &Sample,
    double PointTimestamp,
    FLivoxLidarDetection &OutDetection)
{
  FHitResult HitInfo(ForceInit);

  FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("LivoxAvia_Trace")), true, this);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;

  const FTransform ActorTransf = GetTransform();
  const FVector LidarBodyLoc = ActorTransf.GetLocation();
  const FRotator LidarBodyRot = ActorTransf.Rotator();

  // Unreal FRotator(Pitch, Yaw, Roll):
  // Pitch = elevation angle, Yaw = azimuth angle.
  const FRotator LaserRot(Sample.ElevationDeg, Sample.AzimuthDeg, 0.0f);
  const FRotator ResultRot = UKismetMathLibrary::ComposeRotators(LaserRot, LidarBodyRot);

  const FVector EndTrace =
      Config.Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  GetWorld()->ParallelLineTraceSingleByChannel(
      HitInfo,
      LidarBodyLoc,
      EndTrace,
      ECC_GameTraceChannel2,
      TraceParams,
      FCollisionResponseParams::DefaultResponseParam);

  if (!HitInfo.bBlockingHit)
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
  // CARLA existing lidar code uses attenuation exp(-rate * distance).
  // Here we scale it to a Livox-like reflectivity range 0..255.
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
```

---

## 3. 新增：`LibCarla/source/carla/sensor/data/LivoxLidarData.h`

```cpp
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
      : x(InX),
        y(InY),
        z(InZ),
        intensity(InIntensity),
        tag(InTag),
        line(InLine),
        timestamp(InTimestamp) {}
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
```

---

## 4. 新增：`LibCarla/source/carla/sensor/data/LivoxLidarMeasurement.h`

```cpp
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
```

---

## 5. 新增：`LibCarla/source/carla/sensor/s11n/LivoxLidarSerializer.h`

```cpp
#pragma once

#include "carla/Buffer.h"
#include "carla/Memory.h"
#include "carla/sensor/RawData.h"
#include "carla/sensor/data/LivoxLidarData.h"

#include <boost/asio/buffer.hpp>
#include <array>

namespace carla {
namespace sensor {

  class SensorData;

namespace s11n {

  class LivoxLidarHeaderView
  {
    using Index = data::LivoxLidarData::Index;

  public:

    float GetHorizontalAngle() const
    {
      return reinterpret_cast<const float &>(_begin[Index::HorizontalAngle]);
    }

    uint32_t GetChannelCount() const
    {
      return _begin[Index::ChannelCount];
    }

    uint32_t GetPointCount(size_t channel) const
    {
      DEBUG_ASSERT(channel < GetChannelCount());
      return _begin[Index::SIZE + channel];
    }

  private:

    friend class LivoxLidarSerializer;

    explicit LivoxLidarHeaderView(const uint32_t *begin)
      : _begin(begin)
    {
      DEBUG_ASSERT(_begin != nullptr);
    }

    const uint32_t *_begin;
  };

  class LivoxLidarSerializer
  {
  public:

    static LivoxLidarHeaderView DeserializeHeader(const RawData &data)
    {
      return LivoxLidarHeaderView{reinterpret_cast<const uint32_t *>(data.begin())};
    }

    static size_t GetHeaderOffset(const RawData &data)
    {
      auto View = DeserializeHeader(data);
      return sizeof(uint32_t) * (View.GetChannelCount() + data::LivoxLidarData::Index::SIZE);
    }

    template <typename Sensor>
    static Buffer Serialize(
        const Sensor &sensor,
        const data::LivoxLidarData &data,
        Buffer &&output);

    static SharedPtr<SensorData> Deserialize(RawData &&data);
  };

  template <typename Sensor>
  inline Buffer LivoxLidarSerializer::Serialize(
      const Sensor &,
      const data::LivoxLidarData &data,
      Buffer &&output)
  {
    std::array<boost::asio::const_buffer, 2u> seq = {
        boost::asio::buffer(data._header),
        boost::asio::buffer(data._points)
    };
    output.copy_from(seq);
    return std::move(output);
  }

} // namespace s11n
} // namespace sensor
} // namespace carla
```

---

## 6. 新增：`LibCarla/source/carla/sensor/s11n/LivoxLidarSerializer.cpp`

```cpp
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
```

---

## 7. 修改：`Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/ActorBlueprintFunctionLibrary.cpp`

找到：

```cpp
void UActorBlueprintFunctionLibrary::MakeLidarDefinition(
    const FString &Id,
    bool &Success,
    FActorDefinition &Definition)
```

在已有 `ray_cast` / `hss_lidar` / `ray_cast_semantic` 分支附近，增加 `livox_avia` 分支。

### 7.1 在公共 LiDAR variation 定义后增加 Livox 专用 variations

在已有 `StdDevLidar` 后面加入：

```cpp
  // Livox Avia specific parameters.
  FActorVariation CsvPath;
  CsvPath.Id = TEXT("csv_path");
  CsvPath.Type = EActorAttributeType::String;
  CsvPath.RecommendedValues = { TEXT("") };
  CsvPath.bRestrictToRecommended = false;

  FActorVariation PatternDuration;
  PatternDuration.Id = TEXT("pattern_duration");
  PatternDuration.Type = EActorAttributeType::Float;
  PatternDuration.RecommendedValues = { TEXT("4.0") };
  PatternDuration.bRestrictToRecommended = false;

  FActorVariation LoopPattern;
  LoopPattern.Id = TEXT("loop_pattern");
  LoopPattern.Type = EActorAttributeType::Bool;
  LoopPattern.RecommendedValues = { TEXT("true") };
  LoopPattern.bRestrictToRecommended = false;

  FActorVariation RelativeTimestamp;
  RelativeTimestamp.Id = TEXT("relative_timestamp");
  RelativeTimestamp.Type = EActorAttributeType::Bool;
  RelativeTimestamp.RecommendedValues = { TEXT("false") };
  RelativeTimestamp.bRestrictToRecommended = false;

  FActorVariation DefaultIntensity;
  DefaultIntensity.Id = TEXT("default_intensity");
  DefaultIntensity.Type = EActorAttributeType::Float;
  DefaultIntensity.RecommendedValues = { TEXT("100.0") };
  DefaultIntensity.bRestrictToRecommended = false;

  FActorVariation MaxPointsPerTick;
  MaxPointsPerTick.Id = TEXT("max_points_per_tick");
  MaxPointsPerTick.Type = EActorAttributeType::Int;
  MaxPointsPerTick.RecommendedValues = { TEXT("20000") };
  MaxPointsPerTick.bRestrictToRecommended = false;
```

### 7.2 增加 `livox_avia` 分支

在 `else if (Id == "ray_cast_semantic")` 前后均可，加入：

```cpp
  else if (Id == "livox_avia") {
    Channels.RecommendedValues = { TEXT("1") };
    Range.RecommendedValues = { TEXT("450.0") };
    PointsPerSecond.RecommendedValues = { TEXT("240000") };
    Frequency.RecommendedValues = { TEXT("10.0") };
    UpperFOV.RecommendedValues = { TEXT("38.4") };
    LowerFOV.RecommendedValues = { TEXT("-38.4") };
    HorizontalFOV.RecommendedValues = { TEXT("70.4") };

    DropOffGenRate.RecommendedValues = { TEXT("0.0") };
    DropOffIntensityLimit.RecommendedValues = { TEXT("0.0") };
    DropOffAtZeroIntensity.RecommendedValues = { TEXT("0.0") };

    Definition.Variations.Append({
      Channels,
      Range,
      PointsPerSecond,
      AtmospAttenRate,
      NoiseSeed,
      DropOffGenRate,
      DropOffIntensityLimit,
      DropOffAtZeroIntensity,
      StdDevLidar,
      CsvPath,
      PatternDuration,
      LoopPattern,
      RelativeTimestamp,
      DefaultIntensity,
      MaxPointsPerTick
    });
  }
```

说明：

- `rotation_frequency` 对 Avia 不是真实机械旋转频率，所以不放入 Livox branch 的 variations。
- `upper_fov/lower_fov/horizontal_fov` 可作为文档属性，但真正扫描角度由 CSV 决定。若需要显示在 blueprint，可加入 variations；最小实现不依赖它们。

---

## 8. 修改：`LibCarla/source/carla/sensor/SensorRegistry.h`

### 8.1 serializer include 区增加

在其他 serializer include 后加入：

```cpp
#include "carla/sensor/s11n/LivoxLidarSerializer.h"
```

### 8.2 forward declaration 区增加

```cpp
class ALivoxAviaLidar;
```

### 8.3 `SensorRegistry = CompositeSerializer<...>` 中增加 pair

建议放在 `ARayCastLidar` 附近：

```cpp
std::pair<ALivoxAviaLidar *, s11n::LivoxLidarSerializer>,
```

例如：

```cpp
    std::pair<ARayCastSemanticLidar *, s11n::SemanticLidarSerializer>,
    std::pair<ARayCastLidar *, s11n::LidarSerializer>,
    std::pair<ALivoxAviaLidar *, s11n::LivoxLidarSerializer>,
    std::pair<ARssSensor *, s11n::NoopSerializer>,
```

### 8.4 sensor include 区增加

在底部 `#ifdef LIBCARLA_SENSOR_REGISTRY_WITH_SENSOR_INCLUDES` 区加入：

```cpp
#include "Carla/Sensor/LivoxAviaLidar.h"
```

---

## 9. 修改：`PythonAPI/carla/source/libcarla/SensorData.cpp`

### 9.1 头部 include 增加

在已有：

```cpp
#include <carla/sensor/data/LidarMeasurement.h>
```

后加入：

```cpp
#include <carla/sensor/data/LivoxLidarMeasurement.h>
```

### 9.2 ostream 输出增加

在 `LidarMeasurement` 的 `operator<<` 附近增加：

```cpp
  std::ostream &operator<<(std::ostream &out, const LivoxLidarMeasurement &meas) {
    out << "LivoxLidarMeasurement(frame=" << std::to_string(meas.GetFrame())
        << ", timestamp=" << std::to_string(meas.GetTimestamp())
        << ", number_of_points=" << std::to_string(meas.size())
        << ')';
    return out;
  }
```

### 9.3 Boost.Python binding 增加

在 `LidarMeasurement` binding 后面加入：

```cpp
  class_<csd::LivoxLidarMeasurement, bases<cs::SensorData>, boost::noncopyable, boost::shared_ptr<csd::LivoxLidarMeasurement>>("LivoxLidarMeasurement", no_init)
    .add_property("horizontal_angle", &csd::LivoxLidarMeasurement::GetHorizontalAngle)
    .add_property("channels", &csd::LivoxLidarMeasurement::GetChannelCount)
    .add_property("raw_data", &GetRawDataAsBuffer<csd::LivoxLidarMeasurement>)
    .def("get_point_count", &csd::LivoxLidarMeasurement::GetPointCount, (arg("channel")))
    .def("__len__", &csd::LivoxLidarMeasurement::size)
    .def("__iter__", iterator<csd::LivoxLidarMeasurement>())
    .def("__getitem__", +[](const csd::LivoxLidarMeasurement &self, size_t pos) -> csd::LivoxLidarDetection {
      return self.at(pos);
    })
    .def(self_ns::str(self_ns::self))
  ;

  class_<csd::LivoxLidarDetection>("LivoxLidarDetection")
    .def_readwrite("x", &csd::LivoxLidarDetection::x)
    .def_readwrite("y", &csd::LivoxLidarDetection::y)
    .def_readwrite("z", &csd::LivoxLidarDetection::z)
    .def_readwrite("intensity", &csd::LivoxLidarDetection::intensity)
    .def_readwrite("tag", &csd::LivoxLidarDetection::tag)
    .def_readwrite("line", &csd::LivoxLidarDetection::line)
    .def_readwrite("timestamp", &csd::LivoxLidarDetection::timestamp)
    .def(self_ns::str(self_ns::self))
  ;
```

注意：`LivoxLidarDetection` 是 packed struct，Boost.Python 对 `uint8_t` 可能显示为字符。如果 Python 层读 `line/tag` 不舒服，可以后续改成 getter 返回 `int`。第一版先保证编译和 raw_data 可用。

---

## 10. 修改：`PythonAPI/carla/source/carla/libcarla.pyi`

找到 `class LidarMeasurement(SensorData):` 附近，增加：

```python
class LivoxLidarDetection:
    x: float
    y: float
    z: float
    intensity: float
    tag: int
    line: int
    timestamp: float

class LivoxLidarMeasurement(SensorData):
    horizontal_angle: float
    channels: int
    raw_data: bytes

    def get_point_count(self, channel: int) -> int: ...
    def __len__(self) -> int: ...
    def __iter__(self) -> Iterator[LivoxLidarDetection]: ...
    def __getitem__(self, pos: int) -> LivoxLidarDetection: ...
```

如果文件里已经有 typing imports，复用即可；如果没有 `Iterator`，按原文件风格添加。

---

## 11. 新增：`LibCarla/source/carla/ros2/publishers/CarlaLivoxLidarPublisher.h`

```cpp
#pragma once

#include <memory>
#include <vector>

#include "CarlaPointCloudPublisher.h"

namespace carla {
namespace ros2 {

  class CarlaLivoxLidarPublisher : public CarlaPointCloudPublisher
  {
  public:

    CarlaLivoxLidarPublisher(std::string base_topic_name, std::string frame_id)
      : CarlaPointCloudPublisher(base_topic_name, frame_id) {}

  private:

    const size_t GetPointSize() override;
    std::vector<sensor_msgs::msg::PointField> GetFields() override;
    std::vector<uint8_t> ComputePointCloud(uint32_t height, uint32_t width, uint8_t *data) override;
  };

} // namespace ros2
} // namespace carla
```

---

## 12. 新增：`LibCarla/source/carla/ros2/publishers/CarlaLivoxLidarPublisher.cpp`

```cpp
#include "CarlaLivoxLidarPublisher.h"

#include "carla/sensor/data/LivoxLidarData.h"

#include <cstring>

namespace carla {
namespace ros2 {

const size_t CarlaLivoxLidarPublisher::GetPointSize()
{
  return sizeof(sensor::data::LivoxLidarDetection);
}

std::vector<sensor_msgs::msg::PointField> CarlaLivoxLidarPublisher::GetFields()
{
  sensor_msgs::msg::PointField x;
  x.name("x");
  x.offset(0);
  x.datatype(sensor_msgs::msg::PointField__FLOAT32);
  x.count(1);

  sensor_msgs::msg::PointField y;
  y.name("y");
  y.offset(4);
  y.datatype(sensor_msgs::msg::PointField__FLOAT32);
  y.count(1);

  sensor_msgs::msg::PointField z;
  z.name("z");
  z.offset(8);
  z.datatype(sensor_msgs::msg::PointField__FLOAT32);
  z.count(1);

  sensor_msgs::msg::PointField intensity;
  intensity.name("intensity");
  intensity.offset(12);
  intensity.datatype(sensor_msgs::msg::PointField__FLOAT32);
  intensity.count(1);

  sensor_msgs::msg::PointField tag;
  tag.name("tag");
  tag.offset(16);
  tag.datatype(sensor_msgs::msg::PointField__UINT8);
  tag.count(1);

  sensor_msgs::msg::PointField line;
  line.name("line");
  line.offset(17);
  line.datatype(sensor_msgs::msg::PointField__UINT8);
  line.count(1);

  sensor_msgs::msg::PointField timestamp;
  timestamp.name("timestamp");
  timestamp.offset(18);
  timestamp.datatype(sensor_msgs::msg::PointField__FLOAT64);
  timestamp.count(1);

  return {x, y, z, intensity, tag, line, timestamp};
}

std::vector<uint8_t> CarlaLivoxLidarPublisher::ComputePointCloud(
    uint32_t height,
    uint32_t width,
    uint8_t *data)
{
  auto *detections = reinterpret_cast<sensor::data::LivoxLidarDetection *>(data);
  const size_t total_points = static_cast<size_t>(height) * static_cast<size_t>(width);

  // CARLA lidar convention to ROS convention: flip Y.
  for (size_t i = 0; i < total_points; ++i)
  {
    detections[i].y *= -1.0f;
  }

  const size_t total_bytes = total_points * sizeof(sensor::data::LivoxLidarDetection);

  return std::vector<uint8_t>(
      reinterpret_cast<uint8_t *>(detections),
      reinterpret_cast<uint8_t *>(detections) + total_bytes);
}

} // namespace ros2
} // namespace carla
```

---

## 13. 修改：`LibCarla/source/carla/ros2/ROS2.h`

### 13.1 forward declaration 增加

在 lidar data declarations 附近加入：

```cpp
      class LivoxLidarData;
```

完整上下文类似：

```cpp
  namespace sensor {
    namespace data {
      struct DVSEvent;
      class LidarData;
      class LivoxLidarData;
      class SemanticLidarData;
      class RadarData;
    }
  }
```

### 13.2 public method 增加

在 `ProcessDataFromLidar(...)` 后加入：

```cpp
    void ProcessDataFromLivoxLidar(
      uint64_t sensor_type,
      const carla::geom::Transform sensor_transform,
      carla::sensor::data::LivoxLidarData &data,
      void *actor = nullptr);
```

---

## 14. 修改：`LibCarla/source/carla/ros2/ROS2.cpp`

### 14.1 include 增加

在：

```cpp
#include "carla/sensor/data/LidarData.h"
```

后加入：

```cpp
#include "carla/sensor/data/LivoxLidarData.h"
```

在 publishers include 区加入：

```cpp
#include "publishers/CarlaLivoxLidarPublisher.h"
```

### 14.2 enum 增加

在 `enum ESensors` 里，建议放在 `RayCastLidar` 后：

```cpp
  LivoxAviaLidar,
```

例如：

```cpp
  RayCastSemanticLidar,
  RayCastLidar,
  LivoxAviaLidar,
  RssSensor,
```

注意：这个 enum 顺序必须和 `SensorRegistry` 中对应 sensor 顺序一致。把 `ALivoxAviaLidar` 放在 `ARayCastLidar` 后面时，这里也放在 `RayCastLidar` 后面。

### 14.3 `GetOrCreateSensor()` 增加 case

在 `RayCastLidar` case 后加入：

```cpp
    case ESensors::LivoxAviaLidar:
      return create_and_register(std::make_shared<CarlaLivoxLidarPublisher>(topic_name, frame_id));
```

### 14.4 增加 `ProcessDataFromLivoxLidar`

在 `ProcessDataFromLidar(...)` 后加入：

```cpp
void ROS2::ProcessDataFromLivoxLidar(
    uint64_t sensor_type,
    const carla::geom::Transform sensor_transform,
    carla::sensor::data::LivoxLidarData &data,
    void *actor)
{
  auto base_publisher = GetOrCreateSensor(ESensors::LivoxAviaLidar, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaLivoxLidarPublisher>(base_publisher);
  auto transform_publisher = GetOrCreateTransformPublisher(actor);

  if (!sensor_publisher)
  {
    return;
  }

  size_t width = data._points.size();
  size_t height = 1;

  sensor_publisher->WritePointCloud(
      _seconds,
      _nanoseconds,
      static_cast<uint32_t>(height),
      static_cast<uint32_t>(width),
      reinterpret_cast<uint8_t *>(data._points.data()));

  sensor_publisher->Publish();

  if (transform_publisher)
  {
    transform_publisher->Write(
        _seconds,
        _nanoseconds,
        GetParentFrameId(actor),
        GetFrameId(actor),
        sensor_transform);
    transform_publisher->Publish();
  }
}
```

---

## 15. 新增示例：`PythonAPI/examples/livox_avia_spawn.py`

```python
#!/usr/bin/env python3

import argparse
import time
import weakref

import carla


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--csv", required=True, help="Absolute path to avia.csv")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    vehicle_bp = bp_lib.find("vehicle.lincoln.mkz_2017")
    vehicle_bp.set_attribute("role_name", "ego_vehicle")

    spawn_points = world.get_map().get_spawn_points()
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[0])

    if vehicle is None:
        raise RuntimeError("failed to spawn ego vehicle")

    lidar_bp = bp_lib.find("sensor.lidar.livox_avia")
    lidar_bp.set_attribute("role_name", "livox_avia")
    lidar_bp.set_attribute("range", "450.0")
    lidar_bp.set_attribute("points_per_second", "240000")
    lidar_bp.set_attribute("csv_path", args.csv)
    lidar_bp.set_attribute("pattern_duration", "4.0")
    lidar_bp.set_attribute("loop_pattern", "true")
    lidar_bp.set_attribute("relative_timestamp", "false")
    lidar_bp.set_attribute("default_intensity", "100.0")

    transform = carla.Transform(
        carla.Location(x=0.0, y=0.0, z=2.0),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )

    lidar = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)

    def callback(data):
        print(f"LivoxAvia frame={data.frame}, points={len(data)}, timestamp={data.timestamp}")

    lidar.listen(callback)

    try:
        while True:
            time.sleep(1.0)
    finally:
        lidar.stop()
        lidar.destroy()
        vehicle.destroy()


if __name__ == "__main__":
    main()
```

---

## 16. CSV 格式要求

默认 CSV 路径：

```text
CarlaAir/LivoxCsv/avia.csv
```

CSV 每行：

```text
time,azimuth,zenith,line
0.000000,0.123,89.456,0
0.000004,0.127,89.460,1
```

解释：

- `time`：该样本在扫描周期内的时间，单位秒。
- `azimuth`：水平角，单位 degree。
- `zenith`：天顶角，单位 degree。
- `line`：Livox laser line id，0-255。
- 代码里使用 `elevation = zenith - 90.0`。

如果你拿到的 CSV 第三列已经是 elevation，不是 zenith，需要把：

```cpp
const float Zenith = FCString::Atof(*Parts[2]);
const float Elevation = Zenith - 90.0f;
```

改成：

```cpp
const float Elevation = FCString::Atof(*Parts[2]);
```

---

## 17. 编译注意事项

### 17.1 新 C++ 文件需要被 Unreal Build Tool 识别

通常 CARLA UE 插件 Source 下新增 `.cpp/.h` 会被 UBT 自动包含。如果没有编译到，检查 `Carla.Build.cs` 是否有特殊排除规则。

### 17.2 ROS2 enum 顺序必须对齐 SensorRegistry

CarlaAir `ROS2.cpp` 内的 `enum ESensors` 注释说明它应与 `SensorRegistry` 传感器顺序一致。新增 `ALivoxAviaLidar` 时，要保证两边顺序一致。

推荐顺序：

`SensorRegistry.h`：

```cpp
std::pair<ARayCastSemanticLidar *, s11n::SemanticLidarSerializer>,
std::pair<ARayCastLidar *, s11n::LidarSerializer>,
std::pair<ALivoxAviaLidar *, s11n::LivoxLidarSerializer>,
```

`ROS2.cpp`：

```cpp
RayCastSemanticLidar,
RayCastLidar,
LivoxAviaLidar,
```

### 17.3 不要改普通 `LidarData`

不要把普通 `LidarData` 改成 5-float 或 7-field Livox 格式。CarlaAir 当前普通 lidar ROS2 publisher 默认按 4-float `x,y,z,intensity` 处理，改普通数据结构会破坏原始 CARLA LiDAR 和 HSS LiDAR。

Livox 使用独立：

```cpp
LivoxLidarData
LivoxLidarSerializer
LivoxLidarMeasurement
CarlaLivoxLidarPublisher
```

---

## 18. Codex 执行摘要

请 Codex 按以下顺序改：

1. 新建 `LivoxAviaLidar.h/.cpp`，不要使用 `RayCastLivoxLidar` 命名。
2. 新建 `LivoxLidarData.h`，使用 packed `LivoxLidarDetection`，点格式为：
   `float32 x,y,z,intensity + uint8 tag + uint8 line + float64 timestamp`。
3. 新建 `LivoxLidarMeasurement.h`。
4. 新建 `LivoxLidarSerializer.h/.cpp`。
5. 修改 `ActorBlueprintFunctionLibrary.cpp`，让 `MakeLidarDefinition("livox_avia")` 生成 `sensor.lidar.livox_avia` 的 blueprint variations。
6. 修改 `SensorRegistry.h`，注册：
   `ALivoxAviaLidar -> LivoxLidarSerializer`。
7. 修改 `SensorData.cpp` 和 `libcarla.pyi` 暴露 Python API。
8. 新建 `CarlaLivoxLidarPublisher.h/.cpp`。
9. 修改 `ROS2.h/.cpp` 增加 `ProcessDataFromLivoxLidar()`。
10. 添加 `PythonAPI/examples/livox_avia_spawn.py` 测试脚本。
11. 编译 CarlaAir。
12. 启动 CarlaAir 后运行 Python 示例，确认：
    - Python blueprint library 能找到 `sensor.lidar.livox_avia`
    - callback 类型为 `LivoxLidarMeasurement`
    - `len(data) > 0`
    - ROS2 topic 中 PointCloud2 字段包含 `x,y,z,intensity,tag,line,timestamp`

---

## 19. 设计边界

这个实现是“最像真实 Livox 的接口和扫描模式”：

- 类名是 `LivoxAviaLidar`，不叫 `RayCastLivoxLidar`。
- blueprint 是 `sensor.lidar.livox_avia`。
- 扫描模式由 Avia CSV 非重复角度序列驱动。
- 每个点保留 `line` 和 `timestamp`。
- ROS2 输出对齐 Livox ROS Driver 2 常见 PointXYZRTLT 字段。

但它仍然使用 Unreal line trace 做距离求交，这是仿真器中实现激光测距的常规方式，不等价于传统旋转 ray-cast 雷达模型。
