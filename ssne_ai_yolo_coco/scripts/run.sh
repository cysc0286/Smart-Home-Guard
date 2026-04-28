#!/bin/sh

insmod /lib/modules/$(uname -r)/extra/gpio_kmod.ko || true
insmod /lib/modules/$(uname -r)/extra/uart_kmod.ko || true
chmod +x ./ssne_ai_yolo_coco
./ssne_ai_yolo_coco
