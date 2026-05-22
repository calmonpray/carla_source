# Unreal Engine 4.26 下载与构建（为 CarlaAir 准备）

本文档说明如何在本机下载并构建 Unreal Engine 4.26（CARLA 兼容分支），并把引擎根目录指向 CarlaAir 所需的 `UE4_ROOT`。建议在有稳定网络、足够磁盘（>100GB）和合适权限的机器上执行。

重要说明：从 GitHub 上获取 Unreal Engine 源码通常需要将你的 GitHub 账号与 Epic Games 账号关联；如果没有权限，克隆会失败。下面同时给出自动脚本（`ue4_fetch_build.sh`）与手工步骤。

目标路径（示例）：
`/media/ubuntu/data/home/ubuntu/source_code_carlaAir/UnrealEngine_4.26`

一、先决条件（Ubuntu 示例）

- 磁盘：至少 100 GB 可用空间
- 常用工具：`git`, `curl`, `unzip`, `python3`, `build-essential`, `clang`, `libc++-dev` 等（详见 CARLA 官方要求）
- 在仓库中已有的依赖安装脚本：`CarlaAir/env_setup/setup_env.sh`（用于 Python/conda 环境）

二、获取源码（手工）

1. 在 GitHub 与 Epic 绑定后，克隆 UE4.26 源码：

```bash
# 目标目录（可改）
cd /media/ubuntu/data/home/ubuntu/source_code_carlaAir
git clone -b 4.26 https://github.com/EpicGames/UnrealEngine.git UnrealEngine_4.26
cd UnrealEngine_4.26
```

2. 运行引擎提供的准备脚本并编译：

```bash
./Setup.sh
./GenerateProjectFiles.sh
make
```

3. 编译完成后，确认存在编辑器二进制：

```bash
ls -l Engine/Binaries/Linux/UE4Editor
```

三、获取源码（可用脚本）

仓库同时包含 `ue4_fetch_build.sh`，可在本机尝试自动化流程（脚本会先检测是否有权限克隆，如果无权限会输出说明）。脚本路径：

`/media/ubuntu/data/home/ubuntu/source_code_carlaAir/new_lidar_instruct/ue4_fetch_build.sh`

四、配置 `UE4_ROOT`（示例）

在 shell 配置或当前会话中：

```bash
export UE4_ROOT=/media/ubuntu/data/home/ubuntu/source_code_carlaAir/UnrealEngine_4.26
# 建议写入 ~/.bashrc 或 ~/.profile 保存
```

五、在 CarlaAir 中的最小编译顺序（引擎准备好后）

```bash
# 在 CarlaAir 根目录
cd /media/ubuntu/data/home/ubuntu/source_code_carlaAir/CarlaAir
make LibCarla
make CarlaUE4Editor ARGS="-module=Carla"
```

六、常见问题与提示

- 如果 `./Setup.sh` 因网络或权限失败，请参阅 GitHub→Epic Games 账号绑定步骤。
- UE 源码很大，下载与编译耗时长（数小时到十数小时），请使用稳定网络与足够内存/CPU。
- 若希望跳过源码编译，可使用已构建的 Engine（只要与 CARLA 兼容并有 `Engine/Binaries/Linux/UE4Editor`），并将 `UE4_ROOT` 指向该目录。

七、我能为你做的操作

- 我可以尝试运行仓库内的 `ue4_fetch_build.sh` 来开始克隆和启动构建，你需要确认允许网络下载并提供（或确认）你的 GitHub→Epic 绑定。
- 我也可以先检测系统上是否已有可用的 UE4 编辑器（搜索常见路径），然后帮你设置 `UE4_ROOT`。

——
（文档自动生成于仓库，为 CarlaAir 新增 Livox 传感器准备）
