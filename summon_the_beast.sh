#!/bin/bash

# === Set up workspace ===
WS_DIR=~/snake_ros2_ws
SRC_DIR=$WS_DIR/src

mkdir -p $SRC_DIR
cd $SRC_DIR

git clone https://github.com/joel-kallarackal/snakelib_ros2.git

mv snakelib_ros2/* .

rmdir snakelib_ros2

echo "Workspace Setup Complete."
echo "Starting Build..."

# === Build workspace ===
cd $WS_DIR
source /opt/ros/humble/setup.bash
colcon build

echo "Installing Dependencies..."
pip install hebi-py
pip install urdf-parser-py
sudo apt install python3-pykdl

# === Create run script ===
LAUNCH_SCRIPT=~/snake_ros2_ws/src/obey_me.sh

touch $LAUNCH_SCRIPT

cat <<EOF > ~/snake_ros2_ws/src/obey_me.sh
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/snake_ros2_ws/install/setup.bash
ros2 launch snakelib_control snake_demo_lite.launch.py
EOF

chmod +x ~/snake_ros2_ws/src/obey_me.sh

# === Create desktop icon ===
cat <<EOF > ~/.local/share/applications/snake_app.desktop
[Desktop Entry]
Name=Snake Control
Comment=Launches snake control
Exec=gnome-terminal -- bash -c "~/snake_ros2_ws/src/obey_me.sh; exec bash"
Icon=utilities-terminal
Terminal=false
Type=Application
Categories=Development;
EOF

chmod +x ~/.local/share/applications/snake_app.sh

echo "Setup complete."
