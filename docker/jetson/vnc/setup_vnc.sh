#!/bin/bash

# Install x0vncserver (included in TigerVNC)
sudo apt update
sudo apt-get -y install tigervnc-common tigervnc-standalone-server tigervnc-scraping-server

echo "Please enter the VNC password:"
mkdir -p ~/.vnc
vncpasswd ~/.vnc/passwd

# Create VNC password file (optional)
# To set a password, uncomment the following lines and modify as needed
# mkdir -p ~/.vnc
# x0vncserver -SecurityTypes=VncAuth -PasswordFile=~/.vnc/passwd

# Create the service file
sudo tee /etc/systemd/system/x0vncserver.service > /dev/null <<EOF
[Unit]
Description=TigerVNC Server (x0vncserver)
After=syslog.target network.target

[Service]
Type=simple
ExecStart=/usr/bin/x0vncserver -display :0 -PasswordFile=/home/$USER/.vnc/passwd -rfbport 5900
User=$USER
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

# Reload the service files, start, and enable the service
sudo systemctl daemon-reload
sudo systemctl enable x0vncserver
sudo systemctl start x0vncserver

echo "x0vncserver service has been installed and started."

