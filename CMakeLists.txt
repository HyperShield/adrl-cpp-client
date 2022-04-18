cmake_minimum_required(VERSION 3.5)

project(client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules")
#INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
#CommonSetup()

find_package(Threads REQUIRED)

set(AIRSIM_ROOT "AIRSIM_ROOT_FOLDER")
message(STATUS "found AIRSIM_ROOT=${AIRSIM_ROOT} and ${PROJECT_NAME}")

#setup include and lib for rpclib which will be referenced by other projects
set(RPCLIB_VERSION_FOLDER rpclib-2.2.1)
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/AirLib/deps/rpclib/include")
#name of .a file with lib prefix
set(RPC_LIB rpc)

string( TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

set(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")
set(BUILD_TYPE "linux")

set(CMAKE_CXX_FLAGS "\
    -std=c++17 -stdlib=libstdc++ -ggdb -Wall -Wextra -Wstrict-aliasing -Wunreachable-code -Wcast-qual -Wctor-dtor-privacy \
    -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wswitch-default \
    -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wswitch-default -Wundef \
    -Wno-variadic-macros -Wno-parentheses -Wno-unused-function -Wno-unused -fdiagnostics-show-option \
    -pthread \
    ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")

if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-stdlib=libc++ -Wno-documentation ${CMAKE_CXX_FLAGS}")
    set(CXX_EXP_LIB "-lc++fs -ferror-limit=10")
else()
    message(STATUS "gcc")
    set(CXX_EXP_LIB "-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")
endif ()

set(BUILD_PLATFORM "x64")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT "${CMAKE_STATIC_LINKER_FLAGS}" STREQUAL "")
  string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
endif()

include_directories(${AIRSIM_ROOT}/AirLib/deps/eigen3)


#SetupConsoleBuild()

## Specify additional locations of header files
include_directories(
  #${AIRSIM_ROOT}/HelloDrone
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  #${AIRSIM_ROOT}/MavLinkCom/include
  #${AIRSIM_ROOT}/MavLinkCom/common_utils
)

link_directories(${AIRSIM_ROOT}/build_debug/output/lib/)

#AddExecutableSource()
add_executable(client main.cpp)
#CommonTargetLink()
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
