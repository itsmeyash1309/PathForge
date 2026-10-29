cmake_minimum_required(VERSION 3.20)
project(route_engine CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")
add_compile_options(-Wall -Wextra -Wpedantic)

if(EXISTS "/opt/homebrew/include")
    include_directories(/opt/homebrew/include)
endif()

include_directories(src)

find_package(ZLIB REQUIRED)
find_package(EXPAT REQUIRED)
find_package(Threads REQUIRED)

set_target_properties(ZLIB::ZLIB PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")
set_target_properties(EXPAT::EXPAT PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")

add_executable(test_dijkstra tests/test_dijkstra.cpp)
add_executable(test_astar tests/test_astar.cpp)

add_executable(route src/main.cpp src/osm_loader.cpp)
target_link_libraries(route PRIVATE ZLIB::ZLIB EXPAT::EXPAT Threads::Threads)
