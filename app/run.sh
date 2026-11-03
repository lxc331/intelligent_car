#!/usr/bin/env bash
# Linux / macOS 启动脚本
set -e
cd "$(dirname "$0")"

# 建图遥控默认速度（m/s，0.02-0.10）；已设置则沿用，不设置用 0.05。
export CARAPP_MAP_SPEED="${CARAPP_MAP_SPEED:-0.05}"

PY=""
for c in python3 python; do
    if command -v "$c" >/dev/null 2>&1; then
        PY="$c"
        break
    fi
done

if [ -z "$PY" ]; then
    echo "[X] 未找到 Python 3，请先安装：https://www.python.org/downloads/"
    exit 1
fi

echo "[启动] 使用 $PY"
exec "$PY" -m carapp.main
