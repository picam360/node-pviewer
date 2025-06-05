git clone --depth=1 https://github.com/Pyrestone/jetson-fan-ctl
cd jetson-fan-ctl
./install.sh
cd ..
rm -rf jetson-fan-ctl


#orin nx
#sudo nano /etc/nvfancontrol.conf
#FAN_DEFAULT_PROFILE quiet -> FAN_DEFAULT_PROFILE cool
#sudo rm /var/lib/nvfancontrol/status
#sudo systemctl restart nvfancontrol.service