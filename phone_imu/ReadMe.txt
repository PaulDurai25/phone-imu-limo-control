LIMO PRO ReadMe

Terminal 1

	source /opt/ros/foxy/setup.bash
	
	ros2 launch limo_base limo_base.launch.py
	
Terminal 2

	source /opt/ros/foxy/setup.bash
	source ~/limo_ros2_ws/install/setup.bash

	ros2 run phone_imu phone_imu_node
	
Terminal 2

	source /opt/ros/foxy/setup.bash
	source ~/limo_ros2_ws/install/setup.bash
	
	ros2 run phone_imu phone_orientation_controller
