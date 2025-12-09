#!/bin/bash

echo "==============================================="
echo " Installing Dependencies for ROS Donkey Kong Game"
echo "==============================================="

# Navigate to script directory (ensures requirements.txt is found)
cd "$(dirname "$0")"

# Update system packages
sudo apt update

echo "Installing system packages (apt)..."
sudo apt install -y \
    python3-pygame \
    xterm \
    python3-pip

echo "Installing Python packages (pip)..."
pip3 install --user -r requirements.txt

echo "==============================================="
echo " All dependencies Installed Successfully!"
echo " You can now run the game with:"
echo "   roslaunch project_game game.launch"
echo "==============================================="
