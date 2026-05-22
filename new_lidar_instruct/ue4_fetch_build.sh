#!/usr/bin/env bash
set -euo pipefail

# Helper script to attempt fetching and building Unreal Engine 4.26
# WARNING: cloning Unreal Engine requires GitHub<>Epic Games account linkage and a lot of disk/CPU.

TARGET_DIR="/media/ubuntu/data/home/ubuntu/source_code_carlaAir/UnrealEngine_4.26"
BRANCH="4.26"

echo "目标目录: $TARGET_DIR"
mkdir -p "$(dirname "$TARGET_DIR")"

if [ -d "$TARGET_DIR/Engine" ]; then
  echo "检测到已存在 Engine 目录：$TARGET_DIR (跳过克隆)"
  exit 0
fi

echo "测试是否有权限从 GitHub 克隆 Unreal Engine..."
if git ls-remote --exit-code https://github.com/EpicGames/UnrealEngine.git refs/heads/$BRANCH &>/dev/null; then
  echo "可以访问，开始克隆（请耐心，体积很大）"
  git clone -b $BRANCH https://github.com/EpicGames/UnrealEngine.git "$TARGET_DIR"
  cd "$TARGET_DIR"
  echo "运行 Setup.sh (可能需要几分钟)..."
  ./Setup.sh
  echo "运行 GenerateProjectFiles.sh..."
  ./GenerateProjectFiles.sh
  echo "开始 make（这一步耗时很长）..."
  make
  echo "编译完成。请将 UE4_ROOT 指向: $TARGET_DIR"
else
  cat <<'EOF'
无法通过 HTTPS 访问 EpicGames/UnrealEngine（可能需要 GitHub 与 Epic Games 账号关联）。
请按以下步骤操作：

1) 在 https://www.unrealengine.com 上创建 Epic Games 账号（若没有）。
2) 到 GitHub 设置页面绑定你的 Epic Games 账户（参见 Epic 的文档）。
3) 在获得访问后，用如下命令手工克隆：

   git clone -b 4.26 https://github.com/EpicGames/UnrealEngine.git /path/to/UnrealEngine_4.26

4) 然后在引擎目录运行：

   ./Setup.sh
   ./GenerateProjectFiles.sh
   make

5) 编译好后在 shell 中导出：

   export UE4_ROOT=/path/to/UnrealEngine_4.26

然后回到 CarlaAir 根目录执行：

   make LibCarla
   make CarlaUE4Editor ARGS="-module=Carla"

EOF
  exit 2
fi
