cmake_minimum_required(VERSION 3.8)
project(fast_astar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-pointer-arith)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


include_directories(include/fast_astar)
set(HEADER_FILES include/fast_astar/AStar.h)
add_library(fast_astar SHARED src/AStar.c ${HEADER_FILES})

ament_export_targets(fast_astar HAS_LIBRARY_TARGET)


ament_package()


install(
  DIRECTORY include/fast_astar
  DESTINATION include
)

install(
  TARGETS fast_astar
  EXPORT fast_astar
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)