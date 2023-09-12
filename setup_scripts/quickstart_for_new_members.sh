echo ROVERFLAKE QUICKSTART INITIATE
#!/bin/bash



# Function to check if ROS is installed
check_ros() {
    if dpkg -l | grep -E "ros-noetic-desktop-full" > /dev/null; then
        toilet -f future "ROS Noetic is already installed."
        return 0
    else
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
    sudo apt-get update
    sudo apt-get install -y ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

# Main execution
check_toilet
check_lolcat
sudo apt-get update | lolcat
check_ros
check_serial
check_ros_geodesy

toilet -f future -F crop "You're good to go!" | lolcat

