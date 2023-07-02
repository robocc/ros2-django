cmake_minimum_required(VERSION 3.5)
project(example_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs)

set(ROS2_DJANGO_INTERFACES_PATH generated_interfaces)
execute_process(COMMAND python3 ./manage.py gen_ros_msgs example_app ./${ROS2_DJANGO_INTERFACES_PATH} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB GENERATED_MSGS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${ROS2_DJANGO_INTERFACES_PATH}/msg/*.msg)
file(GLOB GENERATED_SRVS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${ROS2_DJANGO_INTERFACES_PATH}/srv/*.srv) 

rosidl_generate_interfaces(${PROJECT_NAME} 
  ${GENERATED_MSGS} 
  ${GENERATED_SRVS} 
  DEPENDENCIES geometry_msgs 
)

# Install Python modules
ament_python_install_package(example_django_project)
ament_python_install_package(example_app)

# Install Python executables
install(PROGRAMS example_pkg_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS manage.py DESTINATION share/${PROJECT_NAME}/)

ament_package()