#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5)
project(neuvition_driver)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

    
set  (LIBCURL ${PROJECT_SOURCE_DIR}/lib/libcurl.so)
set  (LIBNEUSDK ${PROJECT_SOURCE_DIR}/lib/libneusdk_boost_1_70.so)
set(OpenCV_DIR /usr/share/OpenCV)
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

#catkin_package()
#include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS})  
add_executable(neuvition_driver neuvition_node.cpp driver.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})


target_link_libraries(neuvition_driver
${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${LIBCURL}
  ${LIBNEUSDK}
)
ament_target_dependencies(neuvition_driver
  rclcpp
  std_msgs sensor_msgs image_transport cv_bridge)
  
#install(TARGETS talker
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS neuvition_driver
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS neuvition_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
                        ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/libneusdk_boost_1_70.so
						${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libneusdk_boost_1_70.so)"
)

 
install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()

