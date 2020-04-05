##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4 five_uav_mavros_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch formation_simulation formation_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch formation_simulation gcs_sitl.launch; exec bash"' \
