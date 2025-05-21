cmake_minimum_required(VERSION 3.5)
project(serial_uart_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(uart_node src/uart_node.cpp)
ament_target_dependencies(uart_node rclcpp)

install(TARGETS
  uart_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
