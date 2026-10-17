sudo diskutil unmountDisk /dev/disk4
sudo dd of=picam360-firmware.rc.img if=/dev/rdisk4 bs=4M status=progress