cmake_minimum_required(VERSION 3.15)
project(KeySender)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include(FetchContent)
FetchContent_Declare(
  asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
  GIT_TAG asio-1-36-0
)
FetchContent_MakeAvailable(asio)

find_package(SDL2 REQUIRED)

set(SDL_STATIC ON)
set(SDL_SHARED OFF)

add_executable(KeySender main.cpp comm.cpp)

target_compile_definitions(KeySender PRIVATE ASIO_STANDALONE)

target_include_directories(KeySender PRIVATE build/_deps/asio-src/asio/include)

target_link_libraries(KeySender PRIVATE 
    SDL2::SDL2-static
    SDL2::SDL2main
    -static
)

if(WIN32)
    target_link_libraries(KeySender PRIVATE 
        setupapi
        ws2_32 
    )
endif()

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_CURRENT_SOURCE_DIR}
)