#!/bin/bash

# Update system packages
sudo apt update
#sudo apt upgrade -y

# Download the latest ARM64 version of Visual Studio Code
# This URL points to the latest ARM64 build directly from Microsoft's repository.
echo "Downloading Visual Studio Code for ARM64..."
wget https://update.code.visualstudio.com/1.58.2/linux-deb-arm64/stable -O code_arm64.deb

# Install the downloaded .deb package
echo "Installing Visual Studio Code..."
sudo dpkg -i code_arm64.deb

# Clean up the downloaded .deb file
rm code_arm64.deb

# Verify the installation
echo "Visual Studio Code has been installed. Launch it with the command: code"

