# Livox Avia LiDAR 编译与构建说明

本文档只整理编译要求和构建顺序，不改 CarlaAir 核心逻辑。

## 1. 目标与版本基线

当前这次改动面向 CarlaAir v0.1.7，对应的源码基线是：

- CARLA 0.9.16
- AirSim 1.8.1
- Unreal Engine 4.26 CARLA fork

本次源码改动都限制在 `/media/ubuntu/data/home/ubuntu/source_code_carlaAir/CarlaAir` 内，新增了：

- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.h`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.cpp`
- `LibCarla/source/carla/sensor/SensorRegistry.h` 的注册项
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/ActorBlueprintFunctionLibrary.cpp` 的 `livox_avia` blueprint 分支
- `PythonAPI/examples/test_livox_avia_lidar.py`
- `docs/livox_avia_lidar.md`

补充说明：这次 Livox Avia 复用了 CARLA 现有的 `s11n::LidarSerializer` 和 `data::LidarMeasurement`，因此没有新增独立的 serializer / sensor data object 文件；如果未来要给 Livox 定制一套新的 Python 可见数据格式，再按官方教程补 `Serializer` 和 `SensorData` 子类即可。

## 2. CARLA 官方 Linux 编译要求

下面这一节整理自 CARLA 0.9.16 官方 Linux build 文档，以及新传感器教程中的编译要求。

### 2.1 系统与硬件要求

- 推荐 Ubuntu 20.04 或 22.04
- 不建议使用 24.04，官方文档没有对其进行内部测试
- 建议至少 130 GB 可用磁盘空间
- 建议 NVIDIA RTX 2000 系列或更高，显存至少 6 GB，最好 8 GB 以上
- 建议至少 4 核的高性能 CPU，例如 i7 或同级别处理器
- 默认需要可用 TCP 端口 2000 和 2001
- 推荐 Python 3.8 或更高版本

### 2.2 软件依赖

Ubuntu 22.04：

```bash
sudo apt-get update
sudo apt-get install build-essential g++-12 cmake ninja-build libvulkan1 python3 python3-dev python3-pip python3-venv autoconf wget curl rsync unzip git git-lfs libpng-dev libtiff5-dev libjpeg-dev
```

Ubuntu 20.04：

```bash
sudo apt-get update
sudo apt-get install build-essential g++-9 cmake ninja-build libvulkan1 python3 python3-dev python3-pip python3-venv autoconf wget curl rsync unzip git git-lfs libpng-dev libtiff5-dev libjpeg-dev
```

### 2.3 Unreal Engine 4.26 构建要求

CARLA 官方文档要求使用它的 UE4.26 fork。构建方式是：

```bash
git clone --depth 1 -b carla https://github.com/CarlaUnreal/UnrealEngine.git ~/UnrealEngine_4.26
cd ~/UnrealEngine_4.26
./Setup.sh && ./GenerateProjectFiles.sh && make
```

注意：官方明确不建议使用 `make -j$(nproc)` 之类的并行参数去强行拉满核心，可能导致构建失败。

构建完成后设置：

```bash
export UE4_ROOT=~/UnrealEngine_4.26
```

### 2.4 CARLA 源码构建要求

CARLA 官方构建流程要求：

```bash
git clone -b ue4-dev https://github.com/carla-simulator/carla
export CARLA_UE4_ROOT=/path/to/carla/folder
./Update.sh
make PythonAPI
make launch
```

官方额外列出的常用命令包括：

- `make help`
- `make launch`
- `make PythonAPI`
- `make LibCarla`
- `make package`
- `make clean`
- `make rebuild`

### 2.5 CARLA 新传感器教程要求

官方新传感器教程强调，服务器侧传感器要覆盖完整链路：

- Sensor actor
- Serializer
- Sensor data object
- Register your sensor

对这次 Livox Avia 而言，`Serializer` 和 `Sensor data object` 这两层没有“缺失”，而是直接复用了现有的 LiDAR 链路：

- 传感器 actor：`ALivoxAviaLidar`
- Serializer：`carla::sensor::s11n::LidarSerializer`
- Sensor data object：`carla::sensor::data::LidarMeasurement`

也就是说，新传感器至少要完成蓝图定义、UE4 侧 actor、LibCarla 注册，以及客户端能识别的输出链路。官方教程还明确提示，`SensorRegistry.h` 必须完成注册，且编译阶段很可能会先碰到模板相关错误。

## 3. CarlaAir 官方编译要求

CarlaAir 自己的构建指南在 `CarlaAir_Release/source/BUILD_GUIDE.md` 中给出的顺序是：

```bash
export UE4_ROOT=<your_ue4_root>
cd <your_carla_root>

make UnrealEngine
make CarlaUE4Editor ARGS="-module=AirSim"
make CarlaUE4Editor ARGS="-module=Carla"
make launch
./Util/BuildTools/Package.sh --config=Shipping --no-zip
```

这说明 CarlaAir 不是纯 CARLA 主线，而是带 AirSim 集成的定制版。因此在完整构建时，AirSim 模块和 Carla 模块都可能参与 UE4 Editor 构建。

## 4. 这次 Livox Avia 改动的最小编译顺序

这次 Livox Avia 传感器改动的核心影响点有两个：

- `LibCarla/source/carla/sensor/SensorRegistry.h`
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/` 和 `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Actor/`

因此，等你的 UE4 和 CarlaAir 构建环境就绪后，最小增量编译顺序是：

```bash
export UE4_ROOT=<your_ue4_root>
cd /media/ubuntu/data/home/ubuntu/source_code_carlaAir/CarlaAir

make LibCarla
make CarlaUE4Editor ARGS="-module=Carla"
```

如果你后续又改到 AirSim 插件，再补：

```bash
make CarlaUE4Editor ARGS="-module=AirSim"
```

但就当前 Livox Avia LiDAR 这次改动来说，AirSim 不是必须重编的目标。

## 5. 为什么是这个顺序

- `SensorRegistry.h` 在 `LibCarla`，注册表改动先编 `LibCarla` 最稳妥。
- `LivoxAviaLidar.h/.cpp` 和 `ActorBlueprintFunctionLibrary.cpp` 都在 Carla UE 插件侧，所以需要重新编译 `CarlaUE4Editor` 的 `Carla` 模块。
- 这次没有修改 `Unreal/CarlaUE4/Plugins/AirSim/`，所以没有必要为了这次 Livox 传感器额外重编 AirSim。

## 6. 运行前后的建议

如果你只是检查新传感器是否能找到蓝图并正确发点云，编完后可以先用：

```bash
python3 PythonAPI/examples/test_livox_avia_lidar.py --duration 5
```

如果你是从一个比较干净的环境开始，建议先确认：

- `UE4_ROOT` 已正确指向 CarlaAir 使用的 UE4.26 fork
- CarlaAir 的依赖已经装好
- 端口 2000 / 2001 没有被占用
- Python 端的 `carla` 包与当前二进制版本匹配

## 7. 结论

当前这份 Livox Avia 传感器改动已经到达可以开始编译验证的阶段。下一步不是继续补代码，而是等你确认 UE4 / CARLA / CarlaAir 构建环境后，再按上面的顺序执行最小编译。