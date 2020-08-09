#rosbag record -o $HOME/Downloads/b2_downstairs.bag /camera/scan /odom /camera/depth/camera_info /camera/depth/image_rect_raw /cmd_vel /tf /tf_static
#rosbag record -o $HOME/Downloads/b2_downstairs.bag /camera/scan /odom /imu /tf /tf_static
rosbag record -q --lz4 -o $HOME/Downloads/b2_downstairs.bag /camera/depth/camera_info /camera/depth/color/points /odom /imu /tf /tf_static
