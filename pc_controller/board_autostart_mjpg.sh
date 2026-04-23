#!/bin/sh
# 板端开机自启 mjpg_streamer 模板（低负载参数）
# 使用说明：
# 1) 把本脚本复制到板子，例如 /usr/local/bin/start_snapshot.sh
# 2) chmod +x /usr/local/bin/start_snapshot.sh
# 3) 在 /etc/rc.local (exit 0 前) 加入：
#    /usr/local/bin/start_snapshot.sh &

MJPG_BIN="$(which mjpg_streamer)"
if [ -z "$MJPG_BIN" ]; then
  echo "[snapshot] mjpg_streamer not found"
  exit 1
fi

# 若已有实例在跑，先不重复启动
if pgrep -f "mjpg_streamer.*output_http.so -p 8081" >/dev/null 2>&1; then
  echo "[snapshot] already running on port 8081"
  exit 0
fi

# 常见 web 目录兜底
if [ -d /usr/local/share/mjpg-streamer/www ]; then
  WWW_DIR=/usr/local/share/mjpg-streamer/www
elif [ -d /usr/share/mjpg-streamer/www ]; then
  WWW_DIR=/usr/share/mjpg-streamer/www
else
  WWW_DIR=/tmp
fi

# 低负载配置：640x360, 5fps, 8081
# snapshot URL: http://<board_ip>:8081/?action=snapshot
"$MJPG_BIN" \
  -i "input_uvc.so -d /dev/video0 -r 640x360 -f 5" \
  -o "output_http.so -p 8081 -w $WWW_DIR" \
  >/tmp/mjpg_streamer.log 2>&1 &

echo "[snapshot] started: http://<board_ip>:8081/?action=snapshot"
