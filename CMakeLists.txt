cmake_minimum_required(VERSION 3.0.2)
project(L2I_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  # pcl_ros
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Open3D REQUIRED)
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
	message("OPENMP FOUND")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
# include_directories(${Open3D_INCLUDE_DIRS})
# link_libraries(${Open3D_LIBRARIES})
catkin_package()

###########
## Build ##
###########

set(PCL_COMPONENT common io)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
message("PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message("PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(${PROJECT_SOURCE_DIR}/LAStools/LASlib/inc/)
include_directories(${PROJECT_SOURCE_DIR}/LAStools/LASzip/src/)
set(LASLIB ${PROJECT_SOURCE_DIR}/LAStools/LASlib/lib/libLASlib.a)

file(GLOB LIB_SOURCES
        ${PROJECT_SOURCE_DIR}/src/*.cpp
        ${PROJECT_SOURCE_DIR}/src/*/*.cpp
        ${PROJECT_SOURCE_DIR}/include/*/*.h
        ${PROJECT_SOURCE_DIR}/include/*/*.cpp
        ${PROJECT_SOURCE_DIR}/include/*.h
        ${PROJECT_SOURCE_DIR}/include/*.hpp
    )

# set( THIRD_PARTY_LIBS 
#     -llas 
#   )


add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} proj ${PCL_LIBRARIES}  ${LASLIB} -llas -lglog -ltbb -lfmt)

# add_executable(read_bag test/read_bag.cpp)
# set_target_properties(read_bag PROPERTIES LINKER_LANGUAGE CXX)
# target_link_libraries(read_bag ${catkin_LIBRARIES} ${PROJECT_NAME} proj ${PCL_LIBRARIES}  ${LASLIB} -llas -lglog -ltbb)

add_executable(calibration test/L2I_calibration.cpp)
set_target_properties(calibration PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(calibration ${catkin_LIBRARIES} ${PROJECT_NAME} proj ${PCL_LIBRARIES}  ${LASLIB} -llas -lglog -ltbb)

# add_executable(test_kdtree test/test_kdtree.cpp)
# set_target_properties(test_kdtree PROPERTIES LINKER_LANGUAGE CXX)
# target_link_libraries(test_kdtree ${catkin_LIBRARIES} ${PROJECT_NAME} proj ${PCL_LIBRARIES}  ${LASLIB} -llas -lglog -ltbb)

add_executable(test_icp test/test_icp.cpp)
set_target_properties(test_icp PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test_icp ${catkin_LIBRARIES} ${PROJECT_NAME} proj ${PCL_LIBRARIES}  ${LASLIB} -llas -lglog -ltbb -lfmt)

add_executable(test_trans test/test_trans.cpp)
set_target_properties(test_trans PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test_trans ${catkin_LIBRARIES}  proj ${PCL_LIBRARIES}  ${LASLIB} -llas -lglog -ltbb)

# add_executable(test_rosbagread test/rosbagread.cpp)
# set_target_properties(test_rosbagread PROPERTIES LINKER_LANGUAGE CXX)
# target_link_libraries(test_rosbagread ${catkin_LIBRARIES} ${PROJECT_NAME} proj ${PCL_LIBRARIES}  ${LASLIB} -llas -lglog -ltbb)

install(
   DIRECTORY include/${PROJECT_NAME}
   DESTINATION include
)
# install(
#    TARGETS ${PROJECT_NAME}
#    LIBRARY DESTINATION lib)


pybind11_add_module(HDmap_L2I_py SHARED pybind/l2i_calibration_py.cpp)
target_link_libraries(HDmap_L2I_py PUBLIC ${catkin_LIBRARIES} ${PROJECT_NAME} proj ${PCL_LIBRARIES} ${LASLIB} -llas -lglog -ltbb)