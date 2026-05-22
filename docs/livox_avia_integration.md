# Livox Avia LiDAR 传感器集成文档

## 概述

本文档描述如何在 CARLA 0.9.16 基础上添加 Livox Avia LiDAR 传感器支持。该传感器使用 CSV 文件驱动扫描角度序列，与传统旋转 ray-cast LiDAR 不同。

**传感器规格：**
- Blueprint ID: `sensor.lidar.livox_avia`
- 类名: `ALivoxAviaLidar`
- 点格式: `float32[x,y,z,intensity] + uint8[tag,line] + float64[timestamp]` (26 bytes/point)

---

## 1. 新增文件清单

### 1.1 Unreal Sensor Actor

| 文件路径 | 说明 |
|---------|------|
| `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.h` | UE4 传感器.actor 头文件 |
| `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.cpp` | UE4 传感器.actor 实现 |

### 1.2 LibCarla 数据结构

| 文件路径 | 说明 |
|---------|------|
| `LibCarla/source/carla/sensor/data/LivoxLidarData.h` | Livox 点云数据结构，包含 LivoxLidarDetection 定义 |
| `LibCarla/source/carla/sensor/data/LivoxLidarMeasurement.h` | Livox 测量数据类，用于 Python API |

### 1.3 序列化器

| 文件路径 | 说明 |
|---------|------|
| `LibCarla/source/carla/sensor/s11n/LivoxLidarSerializer.h` | Livox 序列化器头文件 |
| `LibCarla/source/carla/sensor/s11n/LivoxLidarSerializer.cpp` | Livox 序列化器实现 |

### 1.4 ROS2 发布器

| 文件路径 | 说明 |
|---------|------|
| `LibCarla/source/carla/ros2/publishers/CarlaLivoxLidarPublisher.h` | ROS2 发布器头文件 |
| `LibCarla/source/carla/ros2/publishers/CarlaLivoxLidarPublisher.cpp` | ROS2 发布器实现 |

---

## 2. 数据结构详解

### 2.1 LivoxLidarDetection (26 bytes)

```cpp
struct LivoxLidarDetection
{
  float x;           // X 坐标 (meters)
  float y;           // Y 坐标 (meters)
  float z;           // Z 坐标 (meters)
  float intensity;   // 强度值 (0-255)
  uint8_t tag;       // 标签
  uint8_t line;      // 激光线 ID (0-255)
  double timestamp;  // 时间戳 (seconds)
};
static_assert(sizeof(LivoxLidarDetection) == 26u);
```

### 2.2 LivoxLidarData

继承自 `SemanticLidarData`，使用基类的 `_header` 结构存储：
- `HorizontalAngle`: float
- `ChannelCount`: uint32
- 每通道点数的计数

### 2.3 LivoxLidarMeasurement

反序列化时使用的测量类，提供：
- `GetHorizontalAngle()`
- `GetChannelCount()`
- `GetPointCount(channel)`

---

## 3. 修改现有文件

### 3.1 `ActorBlueprintFunctionLibrary.cpp`

在 `MakeLidarDefinition()` 函数中添加 `livox_avia` 分支：

```cpp
else if (Id == "livox_avia") {
  Channels.RecommendedValues = { TEXT("1") };
  Range.RecommendedValues = { TEXT("450.0") };
  PointsPerSecond.RecommendedValues = { TEXT("240000") };
  // ... 其他属性配置
  
  // Livox 专用参数
  FActorVariation CsvPath;
  CsvPath.Id = TEXT("csv_path");
  // ...
  
  Definition.Variations.Append({...});
}
```

**参数说明：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `range` | float | 450.0 | 扫描范围 (meters) |
| `points_per_second` | int | 240000 | 每秒点数 |
| `csv_path` | string | "" | CSV 文件路径 |
| `pattern_duration` | float | 4.0 | 扫描模式周期 (seconds) |
| `loop_pattern` | bool | true | 是否循环扫描 |
| `relative_timestamp` | bool | false | 时间戳类型 |
| `default_intensity` | float | 100.0 | 默认强度 |
| `max_points_per_tick` | int | 20000 | 每帧最大点数 |

### 3.2 `SensorRegistry.h`

**a) 添加 forward declaration:**
```cpp
class ALivoxAviaLidar;
```

**b) 添加 include:**
```cpp
#include "carla/sensor/s11n/LivoxLidarSerializer.h"
```

**c) 注册 sensor 与 serializer:**
```cpp
std::pair<ALivoxAviaLidar *, s11n::LivoxLidarSerializer>,
```

**d) 添加 sensor include (在文件底部):**
```cpp
#include "Carla/Sensor/LivoxAviaLidar.h"
```

### 3.3 `ROS2.h`

**a) 添加 forward declaration:**
```cpp
class LivoxLidarData;
```

**b) 添加 method declaration:**
```cpp
void ProcessDataFromLivoxLidar(
    uint64_t sensor_type,
    const carla::geom::Transform sensor_transform,
    carla::sensor::data::LivoxLidarData &data,
    void *actor = nullptr);
```

### 3.4 `ROS2.cpp`

**a) 添加 include:**
```cpp
#include "carla/sensor/data/LivoxLidarData.h"
#include "publishers/CarlaLivoxLidarPublisher.h"
```

**b) 在 `enum ESensors` 中添加:**
```cpp
LivoxAviaLidar,
```
注意：顺序需与 SensorRegistry 中的顺序一致。

**c) 在 `GetOrCreateSensor()` 中添加:**
```cpp
case ESensors::LivoxAviaLidar:
  return create_and_register(std::make_shared<CarlaLivoxLidarPublisher>(topic_name, frame_id));
```

**d) 实现 `ProcessDataFromLivoxLidar()`:**
```cpp
void ROS2::ProcessDataFromLivoxLidar(
    uint64_t sensor_type,
    const carla::geom::Transform sensor_transform,
    carla::sensor::data::LivoxLidarData &data,
    void *actor)
{
  auto base_publisher = GetOrCreateSensor(ESensors::LivoxAviaLidar, actor);
  auto sensor_publisher = std::dynamic_pointer_cast<CarlaLivoxLidarPublisher>(base_publisher);
  // ... 发布逻辑
}
```

### 3.5 `SemanticLidarData.h`

**修改原因：** `LivoxLidarData` 需要访问基类的 `_ser_points` 成员。

**修改内容：**
1. 将 `_ser_points` 从 `private` 移到 `protected` 区域
2. 添加 friend 声明：
```cpp
friend class data::LivoxLidarData;
```

### 3.6 `LivoxLidarSerializer.h`

**关键修复：** `LivoxLidarHeaderView` 使用 `SemanticLidarData::Index` 而不是 `LivoxLidarData::Index`：

```cpp
class LivoxLidarHeaderView
{
  using Index = data::SemanticLidarData::Index;  // 不是 LivoxLidarData::Index
  // ...
};
```

### 3.7 `PythonAPI/carla/source/libcarla/SensorData.cpp`

**a) 添加 include:**
```cpp
#include <carla/sensor/data/LivoxLidarMeasurement.h>
```

**b) 添加 ostream operator:**
```cpp
std::ostream &operator<<(std::ostream &out, const LivoxLidarMeasurement &meas) {
  out << "LivoxLidarMeasurement(frame=" << std::to_string(meas.GetFrame())
      << ", timestamp=" << std::to_string(meas.GetTimestamp())
      << ", number_of_points=" << std::to_string(meas.size())
      << ')';
  return out;
}
```

**c) 添加 Boost.Python binding:**
```cpp
class_<csd::LivoxLidarMeasurement, bases<cs::SensorData>, ...>("LivoxLidarMeasurement", no_init)
  .add_property("horizontal_angle", &csd::LivoxLidarMeasurement::GetHorizontalAngle)
  .add_property("channels", &csd::LivoxLidarMeasurement::GetChannelCount)
  .add_property("raw_data", &GetRawDataAsBuffer<csd::LivoxLidarMeasurement>)
  .def("get_point_count", &csd::LivoxLidarMeasurement::GetPointCount, (arg("channel")))
  .def("__len__", &csd::LivoxLidarMeasurement::size)
  .def("__iter__", iterator<csd::LivoxLidarMeasurement>())
  .def("__getitem__", +[](const csd::LivoxLidarMeasurement &self, size_t pos) -> csd::LivoxLidarDetection {
    return self.at(pos);
  });

class_<csd::LivoxLidarDetection>("LivoxLidarDetection")
  .def_readwrite("x", &csd::LivoxLidarDetection::x)
  .def_readwrite("y", &csd::LivoxLidarDetection::y)
  .def_readwrite("z", &csd::LivoxLidarDetection::z)
  .def_readwrite("intensity", &csd::LivoxLidarDetection::intensity)
  .def_readwrite("tag", &csd::LivoxLidarDetection::tag)
  .def_readwrite("line", &csd::LivoxLidarDetection::line)
  .def_readwrite("timestamp", &csd::LivoxLidarDetection::timestamp);
```

### 3.8 `PythonAPI/carla/source/carla/libcarla.pyi`

添加类型提示：
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

---

## 4. ROS2 发布格式

PointCloud2 消息字段：

| 字段名 | 偏移 | 数据类型 | 说明 |
|--------|------|----------|------|
| x | 0 | FLOAT32 | X 坐标 |
| y | 4 | FLOAT32 | Y 坐标 |
| z | 8 | FLOAT32 | Z 坐标 |
| intensity | 12 | FLOAT32 | 强度 |
| tag | 16 | UINT8 | 标签 |
| line | 17 | UINT8 | 激光线 ID |
| timestamp | 18 | FLOAT64 | 时间戳 |

**注意：** Y 轴翻转以符合 CARLA 到 ROS 坐标转换约定。

---

## 5. CSV 格式要求

默认路径: `CarlaAir/LivoxCsv/avia.csv`

格式：
```csv
time,azimuth,zenith,line
0.000000,0.123,89.456,0
0.000004,0.127,89.460,1
```

**列说明：**
- `time`: 时间 (seconds)
- `azimuth`: 水平角 (degrees)
- `zenith`: 天顶角 (degrees)，代码中转换为 `elevation = zenith - 90`
- `line`: 激光线 ID (0-255)

---

## 6. Blueprint 使用示例

```python
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# 查找 Livox Avia 传感器
lidar_bp = bp_lib.find('sensor.lidar.livox_avia')
lidar_bp.set_attribute('csv_path', '/path/to/avia.csv')
lidar_bp.set_attribute('points_per_second', '240000')
lidar_bp.set_attribute('range', '450.0')

# 生成传感器
transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0))
lidar = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)

def callback(data):
    # data 是 LivoxLidarMeasurement 类型
    print(f"Frame {data.frame}, Points: {len(data)}")
    for point in data:
        print(f"({point.x}, {point.y}, {point.z}) intensity={point.intensity}")

lidar.listen(callback)
```

---

## 7. 编译注意事项

### 7.1 UE4 源码要求
编译 CARLA Server 需要 CARLA 版本的 UE4.26 源码（包含 CARLA 特定补丁）。

### 7.2 文件包含顺序
确保 `LivoxLidarSerializer.h` 中 `LivoxLidarHeaderView` 使用正确的 `Index` 枚举（来自 `SemanticLidarData`）。

### 7.3 RandomEngine 成员
`ALivoxAviaLidar` 需要 `RandomEngine` 成员用于噪声和丢弃模拟，必须声明为 `UPROPERTY()` 以便被 UE4 识别。

---

## 8. 已知问题

1. **大文件警告:** 某些 `.uasset` 文件超过 GitHub 50MB 限制，建议使用 Git LFS。

2. **Python 解析:** 默认示例使用 `float32[4]` 解析，但 Livox 实际为 26 字节/点格式，需要正确的数据结构解析。

---

## 9. 文件修改清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `LivoxAviaLidar.h` | 新增 | UE4 Actor 头文件 |
| `LivoxAviaLidar.cpp` | 新增 | UE4 Actor 实现 |
| `LivoxLidarData.h` | 新增 | 数据结构头文件 |
| `LivoxLidarMeasurement.h` | 新增 | 测量数据类 |
| `LivoxLidarSerializer.h` | 新增 | 序列化器头文件 |
| `LivoxLidarSerializer.cpp` | 新增 | 序列化器实现 |
| `CarlaLivoxLidarPublisher.h` | 新增 | ROS2 发布器头文件 |
| `CarlaLivoxLidarPublisher.cpp` | 新增 | ROS2 发布器实现 |
| `ActorBlueprintFunctionLibrary.cpp` | 修改 | 添加 livox_avia 分支 |
| `SensorRegistry.h` | 修改 | 注册 Livox 传感器 |
| `ROS2.h` | 修改 | 添加 ProcessDataFromLivoxLidar |
| `ROS2.cpp` | 修改 | 实现 ROS2 发布逻辑 |
| `SemanticLidarData.h` | 修改 | 开放 _ser_points 访问 |
| `SensorData.cpp` | 修改 | 添加 Python 绑定 |
| `libcarla.pyi` | 修改 | 添加类型提示 |