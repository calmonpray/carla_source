# Livox Avia 最小编译步骤

本文档只保留第一次编译所需的最小步骤，并结合当前环境检查结果给出判断。

## 当前环境检查

已检查到的环境状态：

- 当前工作区根目录存在：`/media/ubuntu/data/home/ubuntu/source_code_carlaAir/CarlaAir`
- `UE4_ROOT` 当前未设置
- `python3 --version` 为 `Python 3.12.7`

结论：源码改动已经准备好进入编译阶段，但当前 shell 环境还不完整，至少需要先补 `UE4_ROOT`。另外，CARLA 0.9.16 官方构建文档推荐使用 Python 3.8 或更高；如果后续要构建 Python API wheel，建议使用一个与 CARLA 0.9.16 更匹配的 Python 版本环境。

## 需要先满足的前提

1. 先准备 Unreal Engine 4.26 CARLA fork，并把 `UE4_ROOT` 指向它。
2. 确认 CARLA / CarlaAir 源码目录可用。
3. 如果要构建 Python API，再准备一个兼容的 Python 环境。

## 最小编译步骤

进入 CarlaAir 根目录：

```bash
export UE4_ROOT=<your_ue4_root>
cd /media/ubuntu/data/home/ubuntu/source_code_carlaAir/CarlaAir
```

先编译 LibCarla，因为这次 Livox Avia 触及了 `SensorRegistry.h`：

```bash
make LibCarla
```

再编译 Carla UE4 Editor 的 Carla 模块，因为新传感器 actor 和蓝图分支都在 UE 插件侧：

```bash
make CarlaUE4Editor ARGS="-module=Carla"
```

## 可选补充步骤

如果你后续还改了 AirSim 插件，才需要额外编译：

```bash
make CarlaUE4Editor ARGS="-module=AirSim"
```

如果你要跑 Python 侧验证脚本，可以在编译完成后执行：

```bash
python3 PythonAPI/examples/test_livox_avia_lidar.py --duration 5
```

## 为什么只列这两步

- `LibCarla/source/carla/sensor/SensorRegistry.h` 是注册入口，必须先编译。
- `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/LivoxAviaLidar.*` 和 `ActorBlueprintFunctionLibrary.cpp` 都在 Carla UE 插件侧，所以还要重编 Carla 模块。
- 这次没有改 AirSim 插件，因此不需要为了 Livox Avia 额外重编 AirSim。

## 这份文档的使用方式

如果你的目标只是确认 Livox Avia 传感器能否顺利编译，按上面的两条 `make` 命令走即可。若后续编译报错，再根据报错定位到具体文件，不要一开始就跑完整包构建。