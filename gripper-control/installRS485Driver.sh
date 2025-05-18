#!/bin/bash

set -e

# Update package lists
sudo apt update

# Install dependencies
sudo apt install -y build-essential dkms linux-headers-$(uname -r)

# Detect architecture
ARCH=$(uname -m)

if [[ "$ARCH" == "x86_64" || "$ARCH" == "amd64" ]]; then
  echo "Detected AMD/Intel architecture."
elif [[ "$ARCH" == "aarch64" ]]; then
  echo "Detected ARM architecture (NVIDIA Jetson Orin or similar)."
else
  echo "Unsupported architecture: $ARCH"
  exit 1
fi

# Clone and install CH340 driver repository
if [ -d "ch340-ubuntu-22.04-fix" ]; then
  rm -rf ch340-ubuntu-22.04-fix
fi

git clone https://github.com/Vishal-Mutha/ch340-ubuntu-22.04-fix.git
cd ch340-ubuntu-22.04-fix

chmod +x install.sh
sudo ./install.sh

# Add current user to dialout group
sudo usermod -aG dialout $USER

# Remove conflicting software
sudo apt remove -y brltty

# Inform the user

echo "CH340 driver installed successfully. Please reboot your system."

read -p "Reboot now? [y/N]: " response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
  sudo reboot
fi
