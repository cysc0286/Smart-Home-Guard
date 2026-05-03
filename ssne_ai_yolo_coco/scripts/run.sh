#!/bin/sh
# Smart Home Guard 启动脚本 (LF line endings required)
# 关键约束：stdout 不能接 | tee 等管道，否则 glibc 全缓冲会让 [FPS] 几分钟才出现
#         必须直接到 console 才是行缓冲，评委现场可实时看到输出
# 崩溃自恢复：sleep 2 后重启，保证 60s 评分窗口可继续展示

LOG=/tmp/guard.log

insmod /lib/modules/$(uname -r)/extra/gpio_kmod.ko 2>/dev/null || true
insmod /lib/modules/$(uname -r)/extra/uart_kmod.ko 2>/dev/null || true

chmod +x ./ssne_ai_yolo_coco

echo "[BOOT] $(date) Smart Home Guard launcher started" >> $LOG

while true; do
  echo "[BOOT] $(date) starting ssne_ai_yolo_coco" >> $LOG
  ./ssne_ai_yolo_coco
  RC=$?
  echo "[BOOT] $(date) exited rc=$RC, respawn in 2s" >> $LOG
  sleep 2
done
