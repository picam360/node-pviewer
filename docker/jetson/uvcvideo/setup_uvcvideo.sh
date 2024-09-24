#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

#sudo apt-get update
#sudo apt-get -y install 

L4T_VERSION_INFO=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-core)
L4T_VERSION=${L4T_VERSION_INFO%%-*}

IFS='.' read -r major minor patch <<< "${L4T_VERSION}"

SOURCE_URL=$(printf "https://developer.nvidia.com/downloads/embedded/l4t/r%d_release_v%d.%d/sources/public_sources.tbz2" "$major" "$minor" "$patch")
echo "use SOURCE_URL=${SOURCE_URL}"
#wget ${SOURCE_URL}
#tar -xvjf public_sources.tbz2
cd Linux_for_Tegra/source/public/
#tar -xvjf kernel_src.tbz2
cd kernel/kernel-5.10
zcat /proc/config.gz > .config
cd drivers/media/usb/uvc
bash $SCRIPT_DIR/insert_code.sh
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
sudo mv /usr/lib/modules/5.10.216-tegra/kernel/drivers/media/usb/uvc/uvcvideo.ko /usr/lib/modules/5.10.216-tegra/kernel/drivers/media/usb/uvc/uvcvideo.ko.bk
sudo cp uvcvideo.ko /usr/lib/modules/5.10.216-tegra/kernel/drivers/media/usb/uvc/uvcvideo.ko
sudo cp $SCRIPT_DIR/../etc/modeprobe.d/uvcvideo.conf /etc/modeprobe.d/uvcvideo.conf

cd $SCRIPT_DIR
#rm public_sources.tbz2
#rm -r Linux_for_Tegra