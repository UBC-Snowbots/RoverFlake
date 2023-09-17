clear
sleep 0.5
echo ROVERFLAKE QUICKSTART INITIATE
sleep 0.8
echo WILL BE RUNNING APT UPGRADE, AND USING -y FOR ALL COMMANDS
echo THIS SCRIPT IS INTENDED TO RUN ON A NEW INSTALL OF UBUNTU 20.04
sleep 4
#!/bin/bash



# Function to check if ROS is installed
check_ros() {
    if dpkg -l | grep -E "ros-noetic-desktop-full" > /dev/null; then
        toilet -f future "ROS Noetic is already installed."
        return 0
    else
        install_ros
        return 1
    fi
}

# Function to check if toilet is installed
check_toilet() {
    if dpkg -l | grep -E "toilet" > /dev/null; then
        toilet -f future "toilet is already installed."
        return 0
    else
        sudo apt-get update
        sudo apt-get install -y toilet
        return 1
    fi
}

# Function to check if lolcat is installed (makes rainbow output)
check_lolcat() {
    if dpkg -l | grep -E "lolcat" > /dev/null; then
        echo "lolcat is already installed." | lolcat
        return 0
    else
        sudo apt-get update
        sudo apt-get install -y lolcat
        return 1
    fi
}

# Function to check if ros-noetic-serial is installed (a great serial package, made by one of the major creators of ROS, so it also has a ROS compatible package)
check_serial() {
    if dpkg -l | grep -E "ros-noetic-serial" > /dev/null; then
        toilet -f future "wjwwood's serial package is already installed."
        return 0
    else
        #sudo apt-get update
        sudo apt-get install -y ros-noetic-serial
        toilet -f future "wjwwood's serial package installed." | lolcat

        return 1
    fi
}

# Function to check if ros-noetic-geodessy is installed (package with GNSS tools)
check_ros_geodesy() {
    if dpkg -l | grep -E "ros-noetic-geodesy" > /dev/null; then
        toilet -f future "ros-noetic-geodesy package is already installed."
        return 0
    else
        #sudo apt-get update
        sudo apt-get install -y ros-noetic-geodesy
        toilet -f future "ros-noetic-geodesy package installed." | lolcat

        return 1
    fi
}

# Function to install ROS
install_ros() {
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-get install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install -y ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update
}

# Main execution
sudo apt-get update
sudo apt-get upgrade
check_toilet
check_lolcat

check_ros
check_serial
check_ros_geodesy
sudo apt-get install ros-noetic-controller-manager | lolcat
sudo apt-get install ros-noetic-cv-bridge | lolcat
sudo apt-get install ros-noetic-moveit
git submodule update --init --recursive
rm -r ~/Roverflake/src/external_pkgs/qt_ros/qt_tutorials/

chmod +x setup_scripts/install_phidgets.sh
./setup_scripts/install_phidgets.sh

toilet -f future -F crop "You're good to go!" | lolcat

