cmake_minimum_required(VERSION 3.20)
project(SmartMarsRover LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)

add_executable(smart_mars_rover main.cpp)

target_include_directories(smart_mars_rover PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(smart_mars_rover PRIVATE SFML::Graphics SFML::Window SFML::System)
