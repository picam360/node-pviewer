#!/bin/bash

#ref : https://web.archive.org/web/20230920145735/https://www.waveshare.com/wiki/JETSON-ORIN-NX-16G-DEV-KIT

sudo apt-get update
sudo apt-get -y install qemu-user-static sshpass abootimg nfs-kernel-server libxml2-utils

mkdir r3521
cd r3532
wget https://developer.nvidia.com/downloads/jetson-linux-r3521-aarch64tbz2 -O Jetson_Linux_R35.2.1_aarch64.tbz2 
wget https://developer.nvidia.com/downloads/linux-sample-root-filesystem-r3521aarch64tbz2 -O Tegra_Linux_Sample-Root-Filesystem_R35.2.1_aarch64.tbz2

sudo tar -xjf Jetson_Linux_R35.2.1_aarch64.tbz2 
cd Linux_for_Tegra/rootfs/ 
sudo tar -xjf ../../Tegra_Linux_Sample-Root-Filesystem_R35.2.1_aarch64.tbz2
cd ../
sudo ./apply_binaries.sh

sudo ./tools/kernel_flash/l4t_initrd_flash.sh --no-flash --external-device sda1 \
     -c tools/kernel_flash/flash_l4t_external.xml -p "-c bootloader/t186ref/cfg/flash_t234_qspi.xml" \
     --showlogs --network usb0 p3509-a02+p3767-0000 internal