#!/usr/bin/env bash
set -e

# 脚本位于 <ws>/src/pb2025_sentry_behavior/tools/groot2/
# 往上 4 级才是工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
PROJECT="$WS_DIR/src/pb2025_sentry_behavior/behavior_trees/Project.btproj"

export LD_LIBRARY_PATH="$SCRIPT_DIR/sysroot/usr/lib/x86_64-linux-gnu:$SCRIPT_DIR/squashfs-root/usr/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"

exec "$SCRIPT_DIR/Groot2.AppImage" "$PROJECT"