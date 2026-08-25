#!/usr/bin/env bash
set -e

WS="${1:-$HOME/catkin_ws}"
PKG_SRC="$WS/src/dk_search_rviz_panel"

echo "[1/4] Installing dk_search_rviz_panel into: $PKG_SRC"
mkdir -p "$WS/src"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$SCRIPT_DIR" != "$PKG_SRC" ]; then
  rm -rf "$PKG_SRC"
  mkdir -p "$PKG_SRC"
  cp -r "$SCRIPT_DIR"/* "$PKG_SRC"/
fi

echo "[2/4] Checking executable / package files"
test -f "$PKG_SRC/package.xml"
test -f "$PKG_SRC/CMakeLists.txt"
test -f "$PKG_SRC/plugin_description.xml"

echo "[3/4] Building workspace"
cd "$WS"
catkin_make

echo "[4/4] Done"
echo "Now run:"
echo "  source $WS/devel/setup.bash"
echo "  roslaunch dk_search_rviz_panel search_panel.launch"
