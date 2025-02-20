cmake_minimum_required(VERSION 3.8)
project(p_04_cpp_base_class_for_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED) 

# Create an interface library (no source files, only header)
add_library(base_classes INTERFACE)

# Specify the include directory for the interface library
target_include_directories(base_classes INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Install the interface library
install(TARGETS base_classes
  EXPORT base_classes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  # Install the entire include/ directory, which contains header files
  DIRECTORY include/
  
  # Specify that the include/ directory should be installed
  # under the include directory of the installation prefix:
  # install/${PROJECT_NAME}/include/${PROJECT_NAME}          
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(base_classes HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})

# ament_export_libraries(base_classes)

ament_package()
