cmake_minimum_required( VERSION 2.8.3 )
project( person_tracking )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    ar_track_alvar_msgs
)

catkin_package()


include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable( person_tracking src/person_tracking.cpp )
target_link_libraries( person_tracking ${catkin_LIBRARIES} )
