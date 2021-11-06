#!/bin/sh
clear
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- scripts prepare modules_prepare
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- -C . M=sound/arm
echo DONE
