project(instrument_websocket)


cmake_minimum_required(VERSION 3.0)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lrt")

include_directories(include)

add_executable(webserver webserver.cpp)
