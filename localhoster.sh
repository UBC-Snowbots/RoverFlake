echo Make sure to source this script, or your env variables wont update
echo THIS SCRIPT WILL ONLY EFFECT THE CURRENT TERMINAL
echo 
echo ROS_HOSTNAME was $ROS_HOSTNAME
echo ROS_IP was $ROS_IP
echo ROS_MASTER_URI was at $ROS_MASTER_URI
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
echo 
echo ROS 1 enviroment variables configured for localhost AKA 127.0.0.1
echo 
echo ROS_HOSTNAME is now $ROS_HOSTNAME
echo ROS_IP is now $ROS_IP
echo ROS_MASTER_URI is now at $ROS_MASTER_URI
echo Sourcing roverflake to save you some time
source devel/setup.bash
echo 
