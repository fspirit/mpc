cmake_minimum_required(VERSION 3.5)
project(mpc)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories(Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.13.1/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

set(SOURCE_FILES main.cpp WebSocketMessageHandler.h WebSocketMessageHandler.cpp ControlData.h MPCController.cpp MPCController.h Polynomial.cpp Polynomial.h ControllerOutput.h Setup.h CostAndConstraintsCalculator.h
        CostAndConstraintsCalculator.cpp)

add_executable(mpc ${SOURCE_FILES})

target_link_libraries(mpc ipopt z ssl uv uWS)