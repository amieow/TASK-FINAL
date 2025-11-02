cmake_minimum_required(VERSION 3.15)

get_filename_component(PROJECT ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT})

if(NOT DEFINED ENV{WEBOTS_HOME})
  message(FATAL_ERROR "WEBOTS_HOME environment variable not set. Please set it to your Webots installation directory.")
endif()

set(WEBOTS_HOME "$ENV{WEBOTS_HOME}")
string(REPLACE "\\" "/" WEBOTS_HOME "${WEBOTS_HOME}")

set(SOURCES
    controller-fp.cpp
)

set(LIBRARIES
    ${WEBOTS_HOME}/lib/controller/libCppController.a
    ws2_32
    advapi32
    kernel32
    user32
    gdi32
    winspool
    shell32
    ole32
    oleaut32
    uuid
    comdlg32
)

include_directories(
    ${WEBOTS_HOME}/include/controller/cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../rapidjson/include
    "D:/Project C++/ROS/TASK-FINAL/Webots/asio/include"
)

add_executable(${PROJECT} ${SOURCES})
target_compile_definitions(${PROJECT} PRIVATE WEBOTS_USED_BY_CONTROLLER)
target_link_libraries(${PROJECT} PRIVATE ${LIBRARIES})

add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy $<TARGET_FILE:${PROJECT}> ${CMAKE_CURRENT_SOURCE_DIR}
)