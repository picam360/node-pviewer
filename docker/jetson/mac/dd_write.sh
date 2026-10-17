sudo diskutil unmountDisk /dev/disk4
sudo dd if=picam360-firmware.rc.img of=/dev/rdisk4 bs=4M status=progress