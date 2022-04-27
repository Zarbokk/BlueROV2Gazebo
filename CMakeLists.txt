cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CATKIN_ENABLE_TESTING OFF)
project(bluerov2gazebo)

find_package(catkin REQUIRED COMPONENTS gazebo_dev gazebo_plugins gazebo_ros roscpp std_msgs pcl_ros ping360_sonar bluerov2common commonbluerovmsg)
find_package(gazebo REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
        ${GAZEBO_MSG_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


catkin_package(INCLUDE_DIRS
        CATKIN_DEPENDS
        roscpp
        gazebo_dev
        gazebo_plugins
        gazebo_ros)


install(DIRECTORY launch meshes rviz urdf world
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


add_library(rotating_sonar_control_plugin SHARED src/rotatingSonarControlPlugin.cc)
target_link_libraries(rotating_sonar_control_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


add_library(rotating_sonar_ros_plugin SHARED src/rotatingSonarRosPlugin.cc)
target_link_libraries(rotating_sonar_ros_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} RayPlugin)

add_executable(pcl_sonar_grab_full_scan src/publishingSonarScanCompleteNew.cpp)
target_link_libraries(pcl_sonar_grab_full_scan ${catkin_LIBRARIES})

add_executable(generate_dataset_gazebo src/generateDatasetGazebo.cpp)
target_link_libraries(generate_dataset_gazebo ${catkin_LIBRARIES})

add_executable(evaluationgazboposition src/evaluationHelper/gazeboState2Position.cpp)
target_link_libraries(evaluationgazboposition ${catkin_LIBRARIES})

add_executable(dvlSimulation src/simulationDVL.cpp)
target_link_libraries(dvlSimulation ${catkin_LIBRARIES})
