sudo apt update
sudo apt upgrade
sudo apt install tmux
sudo apt install python3-pip
locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
nano ~/.bashrc
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2
sudo apt install python3-pip
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
gazebo --verbose /opt/ros/galactic/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
gazebo --verbose /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
sudo apt install ros-humble-turtlebot3-teleop
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2
pip install numpy
sudo apt install code
snap install code
snap install code --classic
code
ros2 run demo_nodes_cpp talker
nano .bashrc
printenv | grep -i ros
ros2 run demo_nodes_cpp talker
nano .bashrc
sudo apt install python3-numpy
sudo apt install libboost-python-dev
sudo apt install python3-opencv
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
gazebo --verbose /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
cd ~/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
nano ~/.bashrc
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
printenv | grep -i ROS
echo $ROS_DOMAIN_ID
cd ~/
git clone https://github.com/Purdue-ME597/sim_ws.git
cd ~
nano ~/.bashrc
cd ~/ros2_ws/src
ros2 run task_4 auto_navigator.py
colcon build --packages-select task_4
source install/setup.bash
ros2 run task_4 auto_navigator.py
colcon build --packages-select task_4
source install/setup.bash
ros2 run task_4 auto_navigator.py
colcon build --packages-select task_4
source install/local_setup.bash
colcon build --packages-select task_4
source install/local_setup.bash
ros2 run task_4 auto_navigator.py
colcon build --packages-select task_4
source install/local_setup.bash
ros2 run task_4 auto_navigator.py
colcon build --packages-select task_4
source install/local_setup.bash
ros2 run task_4 auto_navigator.py
cd ~/ros2_ws
colcon build --packages-select task_4
source install/setup.bash
ros2 run task_4 auto_navigator.py
colcon build --packages-select task_4
source install/setup.bash
ros2 run task_4 auto_navigator.py
ros2 launch task_4 auto_navigator.py
ros2 run task_4 auto_navigator.py
cd ~/ros2_ws/src/task_4/scripts
ls -1 auto_navigator.py
cd ~/ros2_ws
colcon build --packages-select task_4
source install/setup.bash
ros2 run task_4 auto_navigator.py
colcon build
source install/setup.bash
ros2 run task_4 auto_navigator.py
ls -l ~/ros2_ws/src/task_4/scripts/ | grep auto_navigator.py
ls -l ~/ros2_ws/src/task_4/task_4/ | grep auto_navigator.py
chmod +x ~/ros2_ws/src/task_4/task_4/auto_navigator.py
ls -l ~/ros2_ws/src/task_4/task_4/ | grep auto_navigator.py
source install/setup.bash
ros2 run task_4 auto_navigator.py
colcon build
source install/setup.bash
ros2 run task_4 auto_navigator.py
source /opt/ros/humble/setup.bash
cd ~/sim_ws
ros2 launch turtlebot3_gazebo navigator.launch.py
colcon build --packages-select <package_name>
colcon build --packages-select task_4
source install/setup.bash
ros2 run task_4 auto_navigator.py
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/.bashrc
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
sudo apt install ros-humble-turtlebot4-navigation
colcon build
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
ros2 topic list
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
nano ~/.bashrc
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
source /opt/ros/humble/setup.bash
source install/setup.bash
ping 192.168.1.114
ros2 topic list
ros2 daemon stop; ros2 daemon start
ros2 topic list
nano ~/.bashrc
source/opt/ros/humble/setup.bash
source install/setup.bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/ros2/src
cd ~ros2_ws/src
cd ~/ros2_ws/src
source install/setup.bash
ros2 run task_4 auto_navigator
colcon build
ros2 run task_4 auto_navigator
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
source ~/.bashrc
echo $ROS_DOMAIN_ID
ping 192.168.1.114
ros2 topic list
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
nano ~/.bashrc
source install/setup.bash
ros2 pkg list | grep task_4
ros2 launch turtlebot3_gazebo mapper.launch.py
ros2 launch turtlebot3_gazebo navigator.launch.py
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
colcon build
source install/setup.bash
ros2 launch turtlebot3_gazebo navigator.launch.py
ros2 launch mapper.launch.py
ros2 launch turtlebot3_gazebo mapper.launch.py world:=turtlebot3_house
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo mapper.launch.py world:=turtlebot3_house
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
colcon build
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
ros2 launch turtlebot3_gazebo navigator.launch.py
