#!/bin/bash

#Short the FC REC and GND pins with jump caps or Dupont wires, positioned as shown above, located under the module.

sudo ./tools/kernel_flash/l4t_initrd_flash.sh --flash-only --external-device sda1 \
     -c tools/kernel_flash/flash_l4t_external.xml -p "-c bootloader/t186ref/cfg/flash_t234_qspi.xml" \
     --showlogs --network usb0 p3509-a02+p3767-0000 internal