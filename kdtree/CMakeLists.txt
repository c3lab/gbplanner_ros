cmake_minimum_required(VERSION 3.8)
project(kdtree LANGUAGES C)

# kdtree.c calls alloca() (:215, :416, :492) without including <alloca.h>. GCC provides
# __builtin_alloca via stdlib.h under the GNU dialect, so keep gnu99 rather than c99.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED src/kdtree.c)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
