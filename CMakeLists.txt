cmake_minimum_required(VERSION 3.16)
project(semantic_nav_agent VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(semantic_nav
  src/semantic_map.cpp
  src/planner.cpp
  src/exploration.cpp
  src/task_agent.cpp
  src/metrics.cpp)
target_include_directories(semantic_nav PUBLIC include)

add_executable(nav_demo apps/nav_demo.cpp)
target_link_libraries(nav_demo PRIVATE semantic_nav)

enable_testing()
add_executable(core_tests tests/test_core.cpp)
target_link_libraries(core_tests PRIVATE semantic_nav)
add_test(NAME semantic_nav_core COMMAND core_tests)

install(TARGETS nav_demo RUNTIME DESTINATION bin)
