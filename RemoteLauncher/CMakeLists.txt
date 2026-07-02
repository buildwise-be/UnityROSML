cmake_minimum_required(VERSION 3.10)
project(RemoteLauncher CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(remote_launcher remote_launcher.cpp)
add_executable(example_task example_task.cpp)

if(WIN32)
    target_link_libraries(remote_launcher ws2_32)
endif()
