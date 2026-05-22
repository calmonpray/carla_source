# Livox Avia LiDAR

这是 CarlaAir v0.1.7 的第一版 Livox Avia LiDAR 传感器说明，目标蓝图名为 `sensor.lidar.livox_avia`。

## 当前约定

- 数据类型沿用 `carla.LidarMeasurement`
- `raw_data` 保持 `float32 XYZI`
- 第一版只实现 single return
- 默认点率为 `240000 points/s`
- 默认近似参数为 `70.4° x 77.2°` 非重复扫描

## 默认参数

- `channels = 1`
- `range = 45000.0 cm`
- `points_per_second = 240000`
- `horizontal_fov = 70.4`
- `upper_fov = 38.6`
- `lower_fov = -38.6`
- `rotation_frequency = 0.0`
- `horizontal_resolution = 0.1`
- `noise_stddev = 2.0`
- `atmosphere_attenuation_rate = 0.004`
- `dropoff_general_rate = 0.0`
- `dropoff_intensity_limit = 0.8`
- `dropoff_zero_intensity = 0.4`

## 验证脚本

创建并运行 `PythonAPI/examples/test_livox_avia_lidar.py`，脚本会尝试查找 `sensor.lidar.livox_avia`，挂载到一辆车上，并打印前几帧的点数与近似视场角。

## 后续步骤

下一步需要把新传感器接入 `SensorRegistry.h`，这样蓝图才会真正出现在 CARLA 的 blueprint library 中。

## 注册审核说明

这次注册只需要三处最小改动：

- 在 `LibCarla/source/carla/sensor/SensorRegistry.h` 里补 `ALivoxAviaLidar` 的前向声明。
- 在 `SensorRegistry` 的 `CompositeSerializer` 里增加 `std::pair<ALivoxAviaLidar *, s11n::LidarSerializer>`。
- 在该文件的 sensor include 区域补上 `Carla/Sensor/LivoxAviaLidar.h`。

这样 `sensor.lidar.livox_avia` 才会进入 blueprint registry，并沿用现有的 `s11n::LidarSerializer` 输出链路。