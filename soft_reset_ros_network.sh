echo Killing gazebo, rviz, and roscore
sudo killall -9 gzserver
sudo killall -9 gzclient
sudo killall -9 rviz
sudo killall -9 roscore
sudo killall -9 rosmaster
echo restarting roscore
roscore
