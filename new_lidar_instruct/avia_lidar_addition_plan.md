# CarlaAir 添加 Livox Avia 非重复扫描 LiDAR 方案

本文档基于当前仓库 `/media/ubuntu/data/home/ubuntu/source_code_carlaAir/CarlaAir` 的源码阅读，目标是在 CarlaAir 中新增一个 CARLA 侧 LiDAR 传感器，例如 `sensor.lidar.avia`，模拟 Livox Avia 的非重复扫描点云输出。

## 1. 当前 CarlaAir 版本和构建事实

- CARLA 版本：`0.9.16`
  - `README_CN.md` badge 标注 CARLA 0.9.16。
  - `Unreal/CarlaUE4/Plugins/Carla/Carla.uplugin` 的 `VersionName` 为 `0.9.16`。
  - `Unreal/CarlaUE4/Config/DefaultGame.ini` 的 `ProjectVersion=0.9.16`。
- Unreal Engine：`4.26`
  - `Unreal/CarlaUE4/CarlaUE4.uproject` 的 `EngineAssociation` 为 `4.26`。
  - AirSim 插件 `AirSim.uplugin` 的 `EngineVersion` 为 `4.26.0`。
- AirSim 版本存在两处说法：
  - `README_CN.md` 和示例文档建议 Python 端 `airsim>=1.8.1`，项目 badge 标注 AirSim 1.8.1。
  - 源码插件 `Unreal/CarlaUE4/Plugins/AirSim/AirSim.uplugin` 内部 `VersionName` 是 `1.7.0`。
  - 因此文档/客户端侧按 1.8.1，UE 插件源码元数据仍显示 1.7.0；添加 CARLA 侧 LiDAR 不依赖 AirSim 传感器实现，但编译 CarlaAir 时 AirSim 插件会一起参与 UE4 工程。

源码编译入口：

```bash
cd /media/ubuntu/data/home/ubuntu/source_code_carlaAir/CarlaAir
export UE4_ROOT=<your_ue4_root>

# 编译 UE/CARLA Editor 目标
make CarlaUE4Editor ARGS="-module=Carla"

# 如涉及 AirSim 插件变动再单独编译
make CarlaUE4Editor ARGS="-module=AirSim"

# Python API wheel
make PythonAPI.wheel

# 打包
./Util/BuildTools/Package.sh --config=Shipping --no-zip
```

## 2. 现有 CARLA LiDAR 封装和使用链路

### 2.1 Python 使用入口

用户通过 CARLA blueprint 生成传感器：

```python
bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
bp.set_attribute("range", "100")
bp.set_attribute("channels", "32")
bp.set_attribute("points_per_second", "240000")
sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
sensor.listen(lambda data: ...)
```

`examples/quick_start_showcase.py` 就是这种方式。点云数据在 Python 中是 `carla.LidarMeasurement`，`raw_data` 布局为连续 float32：`x, y, z, intensity`。

### 2.2 UE 侧 sensor 注册

关键文件：

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/SensorFactory.cpp`
- `LibCarla/source/carla/sensor/SensorRegistry.h`

`SensorFactory` 会遍历 `SensorRegistry`，对每个已注册 sensor 调用静态 `GetSensorDefinition()`，并设置 `Def.Class = SensorType::StaticClass()`。因此添加新 LiDAR 必须在 `SensorRegistry.h` 中完成 4 步：

1. include serializer。
2. forward declare sensor 类。
3. 在 `CompositeSerializer` 里加入 `std::pair<AAviaLidar *, s11n::LidarSerializer>`。
4. 在 `LIBCARLA_SENSOR_REGISTRY_WITH_SENSOR_INCLUDES` 分支 include 新 sensor 头文件。

### 2.3 蓝图属性定义和解析

关键文件：

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/ActorBlueprintFunctionLibrary.cpp`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LidarDescription.h`

`MakeLidarDefinition(Id)` 负责生成 blueprint id 和属性。`FillIdAndTags(Definition, "sensor", "lidar", Id)` 会让 `Id="ray_cast"` 变成 `sensor.lidar.ray_cast`。已有分支：

- `ray_cast`
- `ray_cast_semantic`
- `hss_lidar`

`SetLidar()` 从 actor description 读取属性并写入 `FLidarDescription`。注意 `range` 在 Python/blueprint 中是米，进入 UE 射线长度前转换成厘米。

### 2.4 标准旋转式 LiDAR 实现

关键文件：

- `RayCastSemanticLidar.h/.cpp`
- `RayCastLidar.h/.cpp`

`ARayCastSemanticLidar` 负责基础射线逻辑：

- `Set()` 解析 `FLidarDescription`。
- `CreateLasers()` 按 `channels` 在 `upper_fov` 到 `lower_fov` 之间生成竖直线束角。
- `SimulateLidar(DeltaTime)` 计算每帧每线束点数：
  - `points_per_laser = points_per_second * DeltaTime / channels`
  - 水平角随 `rotation_frequency * horizontal_fov * DeltaTime` 推进。
- `ShootLaser()` 调 UE `ParallelLineTraceSingleByChannel()`，碰撞通道为 `ECC_GameTraceChannel2`。
- `ComputeAndSaveDetections()` 写入 semantic 点。

`ARayCastLidar` 继承 `ARayCastSemanticLidar`，复用射线结果并输出普通 `LidarData`：

- `ComputeDetection()` 写 `x,y,z,intensity`。
- `PostprocessDetection()` 加距离方向噪声、dropoff。
- `PostPhysTick()` 调 `DataStream.SerializeAndSend(*this, LidarData, ...)`，ROS2 开启时走 `ROS2->ProcessDataFromLidar(...)`。

### 2.5 当前仓库已有自定义 LiDAR 样板：HSSLidar

关键文件：

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/HSSLidar.h/.cpp`
- `LibCarla/source/carla/sensor/SensorRegistry.h`
- `ActorBlueprintFunctionLibrary.cpp` 的 `hss_lidar` 分支

`AHSSLidar` 已经演示了“新增一个非标准扫描 LiDAR”的方式：

- 继承 `ARayCastSemanticLidar`。
- 复用 `LidarData` 和 `LidarSerializer`，所以 Python 端仍收到 `carla.LidarMeasurement`。
- 自己实现 `PostPhysTick()` 和 `SimulateLidar()`。
- 扫描方式从旋转式改为固定水平 FOV 内按 `horizontal_resolution` 扫描。

Avia 建议以 `HSSLidar` 为直接模板，不动 Python 序列化协议。

## 3. Livox Avia 官方参数提取

官方规格页：https://www.livoxtech.com/avia/specs

建议映射到 CARLA blueprint 属性：

| Avia 参数 | 官方值 | CarlaAir 属性建议 |
| --- | --- | --- |
| FOV, 非重复扫描 | 70.4 deg horizontal x 77.2 deg vertical | `horizontal_fov=70.4`, `upper_fov=38.6`, `lower_fov=-38.6` |
| FOV, 重复线扫描 | 70.4 deg horizontal x 4.5 deg vertical | 可作为后续 `scan_mode=repetitive`，首版不做 |
| Point rate | 240000 pts/s first/strongest return, 480000 dual, 720000 triple | 首版默认 `points_per_second=240000`，`return_mode` 先作为属性保留或暂不实现 |
| Detection range @ 100 klx | 190 m @ 10%, 230 m @ 20%, 320 m @ 80% reflectivity | 首版 `range=190` 或 `320` 可配置；若不模拟反射率，推荐默认 `190` 保守 |
| Detection range @ 0 klx | 190 m @ 10%, 260 m @ 20%, 450 m @ 80% reflectivity | 可选属性 `ambient_light_klx`/`reflectivity_mode` 后续扩展 |
| Range precision | 2 cm, 1 sigma @ 20 m | `noise_stddev=0.02` m，可按距离方向加高斯噪声 |
| Angular precision | < 0.05 deg, 1 sigma | 新增 `angular_noise_stddev=0.05` deg |
| Beam divergence | 0.28 deg vertical x 0.03 deg horizontal | 可作为注释/后续扩展；首版不做光斑面积 |
| Blind zone | < 1 m 不精确，1-2 m 可能失真 | 新增 `blind_zone=1.0` m，`near_distortion_range=2.0` m 可选 |
| Data latency | <= 2 ms | CARLA sensor tick 中暂不模拟 |

Livox 官方也说明 Avia 的非重复扫描覆盖率会随积分时间增加而提高，中心区域点云更密。首版应模拟“有限 FOV 内非重复、中心偏密、跨帧不完全重复”，而不是传统机械 360 度旋转。

## 4. 推荐实现目标

新增：

- UE 类：`AAviaLidar`
- blueprint id：`sensor.lidar.avia`
- Python 数据类型：继续使用 `carla.LidarMeasurement`
- `raw_data` 格式：继续保持 `x, y, z, intensity`
- 默认参数：
  - `channels=1` 或 `channels=6`。推荐 `channels=1`，表示固态 LiDAR 不暴露传统线束；如需并行可内部分块，不通过 channel 表示物理线束。
  - `range=190`
  - `points_per_second=240000`
  - `horizontal_fov=70.4`
  - `upper_fov=38.6`
  - `lower_fov=-38.6`
  - `noise_stddev=0.02`
  - `angular_noise_stddev=0.05`
  - `dropoff_general_rate=0.0`，先避免 CARLA 默认 45% 随机丢点破坏 Avia 点率。

## 5. 需要修改的文件清单

### 5.1 新增 sensor 类

新增：

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/AviaLidar.h`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/AviaLidar.cpp`

建议从 `HSSLidar` 复制骨架，并做这些差异：

- `GetSensorDefinition()` 返回 `MakeLidarDefinition(TEXT("avia"))`。
- `SimulateLidar()` 不使用固定网格，也不使用旋转式 `horizontal_angle`。
- 每帧点数按 `round(points_per_second * DeltaTime)`。
- 用一个持续递增的 `SampleIndex` 或 `TimeAccumulator` 生成非重复扫描角。
- 点云写入 `LidarData`，ROS2 继续走 `ProcessDataFromLidar()`。

### 5.2 扩展 LiDAR 描述结构

修改：

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LidarDescription.h`

建议新增字段：

```cpp
float AngularNoiseStdDev = 0.0f; // degrees
float BlindZone = 0.0f;          // centimeters after SetLidar conversion
FString ScanPattern = TEXT("non_repetitive");
```

如果希望首版最小侵入，也可以只用已有字段，不新增 `ScanPattern`，因为新类默认就是 Avia 非重复扫描。

### 5.3 扩展 blueprint 属性

修改：

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/ActorBlueprintFunctionLibrary.cpp`
- 如头文件声明需要变更，也检查 `ActorBlueprintFunctionLibrary.h`

在 `MakeLidarDefinition()` 中新增 `else if (Id == "avia")` 分支。推荐属性：

```cpp
Channels.RecommendedValues = { TEXT("1") };
Range.RecommendedValues = { TEXT("190") };
PointsPerSecond.RecommendedValues = { TEXT("240000") };
Frequency.RecommendedValues = { TEXT("10") }; // 作为帧/积分参考，不表示机械旋转
UpperFOV.RecommendedValues = { TEXT("38.6") };
LowerFOV.RecommendedValues = { TEXT("-38.6") };
HorizontalFOV.RecommendedValues = { TEXT("70.4") };
DropOffGenRate.RecommendedValues = { TEXT("0.0") };
StdDevLidar.RecommendedValues = { TEXT("0.02") };
```

同时增加可选属性：

- `angular_noise_stddev`
- `blind_zone`
- `return_mode`：`strongest` / `dual` / `triple`，首版可以只解析但不实现多回波。
- `scan_pattern`：`non_repetitive` / `repetitive`，首版只支持 `non_repetitive`。

`SetLidar()` 中读取新增字段。`range` 和 `blind_zone` 都要从米转厘米。

### 5.4 注册 sensor

修改：

- `LibCarla/source/carla/sensor/SensorRegistry.h`

加入：

```cpp
class AAviaLidar;
...
std::pair<AAviaLidar *, s11n::LidarSerializer>,
...
#include "Carla/Sensor/AviaLidar.h"
```

### 5.5 Python 文档和 smoke test

建议修改：

- `PythonAPI/docs/sensor.yml` 或相关传感器文档，加入 `sensor.lidar.avia`。
- `PythonAPI/test/smoke/test_lidar.py` 增加 blueprint 查找和一帧数据接收测试。
- 可新增示例 `PythonAPI/examples/avia_lidar_demo.py`。

首版最小验证脚本：

```python
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(10)
world = client.get_world()
bp = world.get_blueprint_library().find("sensor.lidar.avia")
print(bp)
for a in bp:
    print(a.id, a.recommended_values)
```

## 6. Avia 非重复扫描模型建议

官方没有公开完整 Avia 内部 Risley/光机控制方程，因此仿真应明确为“工程近似”。推荐目标是：

- 点数符合 `points_per_second`。
- 角度限制在 70.4 x 77.2 FOV 内。
- 扫描跨帧不重复。
- 中心区域密度高于边缘。
- 固定随机种子时可复现。

### 6.1 推荐首版：确定性 quasi-random + 中心偏密

用低差异序列生成二维 FOV 采样，再做中心偏密映射。

伪代码：

```cpp
uint64 SampleIndex = PersistentSampleIndex++;

float u = Frac(SampleIndex * 0.61803398875f);
float v = Frac(SampleIndex * 0.41421356237f + 0.17f * FMath::Sin(SampleIndex * 0.013f));

// 映射到 [-1, 1]
float x = 2.0f * u - 1.0f;
float y = 2.0f * v - 1.0f;

// 中心偏密：把均匀样本压向中心。gamma > 1 时中心更密。
float gamma = 1.7f;
x = FMath::Sign(x) * FMath::Pow(FMath::Abs(x), gamma);
y = FMath::Sign(y) * FMath::Pow(FMath::Abs(y), gamma);

float horiz = x * Description.HorizontalFov * 0.5f;
float vert_center = 0.5f * (Description.UpperFovLimit + Description.LowerFovLimit);
float vert_half = 0.5f * (Description.UpperFovLimit - Description.LowerFovLimit);
float vert = vert_center + y * vert_half;
```

然后调用 `ShootLaser(vert, horiz, HitResult, TraceParams)`。

优点：简单、稳定、不会形成传统网格，点云覆盖随时间自然增加。缺点：不是 Avia 真实光机曲线。

### 6.2 可选增强：Lissajous/Rosette 近似

如果希望视觉上更像 Livox 非重复玫瑰线，可以用两个不整比频率的正弦叠加：

```cpp
float t = (SampleIndex + PhaseOffset) / Description.PointsPerSecond;
float x = sin(2*pi*f1*t + 0.3f * sin(2*pi*f3*t));
float y = sin(2*pi*f2*t + 0.2f * sin(2*pi*f4*t));
```

选择互质/不整比频率，例如 `f1=37.0`, `f2=53.0`, `f3=7.0`, `f4=11.0`。再做 FOV 缩放。该方法会产生更强曲线纹理，但参数需要靠点云截图调。

### 6.3 近距离和噪声

Avia 官方有 1 m 盲区和 2 cm 测距随机误差。建议：

- 命中距离 `< blind_zone` 时丢弃点。
- `1 m <= distance < 2 m` 时可提高噪声，或首版仅保留普通噪声。
- 距离方向噪声用现有 `noise_stddev`，单位为米属性，进入 `Detection.point` 时该点坐标是 UE cm 还是 CARLA geom 单位需沿用现有实现。当前 `RayCastLidar` 直接对 `Detection.point` 加 `NoiseStdDev`，因此建议与现有 CARLA 行为保持一致；如果要严格按米，需要单独确认 CARLA 0.9.16 该字段实际单位。
- 角度噪声在发射前扰动 `vert/horiz`，单位度。

## 7. 编译和验证流程

### 7.1 编译

```bash
cd /media/ubuntu/data/home/ubuntu/source_code_carlaAir/CarlaAir
make CarlaUE4Editor ARGS="-module=Carla"
make PythonAPI.wheel
```

如 `SensorRegistry.h` 改动导致 LibCarla client/server 类型不一致，执行：

```bash
make LibCarla.server.release
make LibCarla.client.release
make CarlaUE4Editor ARGS="-module=Carla"
make PythonAPI.wheel
```

### 7.2 运行时验证

1. 启动 CarlaAir。
2. Python 检查 blueprint：

```python
bp = world.get_blueprint_library().find("sensor.lidar.avia")
print(bp.id)
print([a.id for a in bp])
```

3. 生成 sensor，监听 1 秒。
4. 检查：
   - `len(data)` 大约等于 `points_per_second * sensor_tick` 乘以命中率。
   - `raw_data` 可以 reshape 为 `(-1, 4)`。
   - 点云 FOV 在前向 70.4 x 77.2 度内。
   - 静态场景中连续帧角度分布不完全重复。

示例：

```python
points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
print(data.frame, data.channels, len(points), points[:5])
```

### 7.3 推荐可视化

- 复用 `PythonAPI/examples/open3d_lidar.py`，把 blueprint 改为 `sensor.lidar.avia`。
- 或写一个 BEV/FOV 投影图，累计 0.1 s、0.2 s、1.0 s 的点云，观察覆盖率增加。

## 8. 风险和注意点

- `ARayCastSemanticLidar::SimulateLidar()` 在基类中不是 virtual；新类必须像 `HSSLidar` 一样自己实现 `PostPhysTick()` 并调用自己的 `SimulateLidar()`。
- `ARayCastLidar` 的强度/后处理函数是 private，不能直接继承复用。若不重构现有类，就从 `HSSLidar` 复制普通 LiDAR 输出逻辑最稳。
- `LidarData::ResetMemory()` 中有一处 `DEBUG_ASSERT(GetChannelCount() > points_per_channel.size())`，按常理应为 `>=`，但现有代码已这样使用。不要在添加 Avia 时顺手改这个断言，除非专门验证所有 LiDAR。
- `points_per_second=240000` 在复杂地图上每秒射线很多，多个 Avia 同时运行会明显增加 PhysX raycast 压力。首版测试建议 `sensor_tick=0.05` 或 `0.1`，必要时先用 `points_per_second=60000` 调试。
- 多回波会改变点数和数据语义；如果继续使用 CARLA `LidarMeasurement` 的 `x,y,z,intensity`，可以通过增加点数模拟 dual/triple return，但无法表达 return id。若研究需要严格多回波，建议新增数据结构和 Python binding，而不是塞进 intensity。
- AirSim 自身也有 `UnrealLidarSensor`，但本任务是 CARLA blueprint sensor，应该走 CARLA `SensorRegistry`，不要混到 AirSim `settings.json` 传感器系统里。

## 9. 首版实施顺序

1. 复制 `HSSLidar.h/.cpp` 为 `AviaLidar.h/.cpp`，改类名、文件 include、`GetSensorDefinition()`。
2. 实现 quasi-random 非重复扫描 `SimulateLidar()`，保留 `LidarData` 输出和 ROS2 分支。
3. 在 `SensorRegistry.h` 注册 `AAviaLidar`。
4. 在 `ActorBlueprintFunctionLibrary.cpp` 为 `Id == "avia"` 添加默认属性。
5. 如新增 `angular_noise_stddev` / `blind_zone`，同步更新 `LidarDescription.h` 和 `SetLidar()`。
6. 编译 `make CarlaUE4Editor ARGS="-module=Carla"`。
7. 启动 CarlaAir，用 Python 查找 `sensor.lidar.avia` 并采集点云。
8. 用 Open3D 或 BEV 投影确认 FOV、点数、连续帧非重复覆盖。

首版完成后，再考虑：

- `scan_pattern=repetitive` 模式，模拟 70.4 x 4.5 度线扫描。
- `return_mode=dual/triple`。
- 根据材质/语义标签估算反射率并影响 range/intensity。
- 更接近真实 Livox 的光机曲线参数标定。

