cmake_minimum_required(VERSION 3.8)
project(basic-sub-prom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tachimawari_interfaces REQUIRED)
find_package(prometheus-cpp CONFIG REQUIRED)

add_executable(sub main.cpp)
target_link_libraries(sub PRIVATE #Prometheus library
                        prometheus-cpp::push
                        prometheus-cpp::pull
                        prometheus-cpp::core)
target_link_libraries(sub #ROS2 library
                        rclcpp
                        tachimawari_interfaces)

install(TARGETS
      sub
      DESTINATION lib/${PROJECT_NAME})

ament_package()
