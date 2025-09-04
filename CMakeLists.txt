cmake_minimum_required(VERSION 2.8.3)
project(utm_ros)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	message_generation
	std_msgs
)

add_service_files(
	FILES
	LLAToUTM.srv
	UTMToLLA.srv
	LLAToENU.srv
	ENUToLLA.srv
)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	CATKIN_DEPENDS std_msgs sensor_msgs message_runtime 
)
