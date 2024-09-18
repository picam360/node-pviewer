#!/bin/bash

#if some related modules are in problem, next command would help
#rm -rf /home/picam360/.local/lib/

sudo apt-get install python3-smbus -y
#sudo apt-get install python-smbus -y
git clone --depth=1 https://github.com/waveshare/UPS-Power-Module
cd UPS-Power-Module
sudo ./install.sh
cd ..
sudo rm -rf UPS-Power-Module
