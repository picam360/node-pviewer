sudo cp set-usb-role-device.sh /usr/local/bin/set-usb-role-device.sh
sudo chmod +x /usr/local/bin/set-usb-role-device.sh
sudo cp set-usb-role-device.service /etc/systemd/system/set-usb-role-device.service
sudo systemctl daemon-reload
sudo systemctl enable set-usb-role-device.service
sudo systemctl restart set-usb-role-device.service