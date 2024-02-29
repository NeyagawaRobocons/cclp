# CCLP (Coordinate Correction with Line and Points)
```
ros2 run nucleo_agent rp_encoder_agent_node

ros2 run odometry_calculator odometry_node --ros-args -p output_topic:=robot_pose

ros2 run calc_vel robot_tf_node

ros2 run ldlidar ldlidar --ros-args -p topic_name:=scan1 -p serial_port_candidates:=[/dev/ttySerial564D005091]
ros2 run ldlidar ldlidar --ros-args -p topic_name:=scan2 -p serial_port_candidates:=[/dev/ttySerial564D004832]

ros2 run cclp cord_correction_node

ros2 run cclp monitor

ros2 run cclp line_map_server.py
```