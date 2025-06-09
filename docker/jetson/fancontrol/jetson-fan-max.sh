sudo cp jetson-fan-max.service /etc/systemd/system/jetson-fan-max.service
sudo systemctl daemon-reload
sudo systemctl enable jetson-fan-max.service
sudo systemctl restart jetson-fan-max.service