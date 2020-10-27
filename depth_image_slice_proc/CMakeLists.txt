cmake_minimum_required(VERSION 2.8)
project(depth_image_slice_proc)
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    cmake_modules
    eigen_conversions
    image_geometry
    image_transport
    message_filters
    nodelet
    sensor_msgs
    geometry_msgs
    stereo_msgs
    tf2
    tf
    tf2_ros
    pcl_ros
    roscpp
    pluginlib
    )

#if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
#  add_compile_options(-std=c++11)
#endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        eigen_conversions
        image_geometry
        image_transport
        message_filters
        nodelet
        sensor_msgs
        geometry_msgs
        stereo_msgs
        tf2
        tf
        tf2_ros
        pcl_ros
        roscpp
        pluginlib
)

find_package(Boost REQUIRED)

find_package(Eigen3 QUIET)

find_package(PCL)

if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

#find_package(OpenCV REQUIRED)

include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/nodelets/point_cloud_xyz_slice.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
