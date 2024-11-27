sudo chmod 777 /dev/ttyACM0 & sleep 2;

source /home/dinhnambkhn/UGV_Autonomy_ws/devel/setup.bash;
roslaunch vesc_driver vesc_driver_node.launch & sleep 2;

source /home/dinhnambkhn/UGV_Autonomy_ws/devel/setup.bash;
roslaunch vesc_ackermann ackermann_to_vesc_node.launch & sleep 2;

source /home/dinhnambkhn/UGV_Autonomy_ws/devel/setup.bash;
roslaunch vesc_ackermann vesc_to_odom_node.launch & sleep 2;
