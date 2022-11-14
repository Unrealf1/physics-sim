cmake_minimum_required(VERSION 3.13)

project(collision-simulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/CPM.cmake)
CPMAddPackage("gh:g-truc/glm@0.9.9.8#0.9.9.8")
CPMAddPackage(
  NAME      "SDL2"         
  VERSION   "2.0.18"
  OPTIONS
    "SDL_X11 ON"
    "SDL_WAYLAND OFF"
  URL "https://www.libsdl.org/release/SDL2-2.0.18.zip"
)

CPMAddPackage("gh:gabime/spdlog@1.9.2")

add_library(project_options INTERFACE)
add_library(project_warnings INTERFACE)

include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

file(GLOB_RECURSE SOURCES . source/*.cpp)
add_library(lib STATIC ${SOURCES})
target_include_directories(lib PUBLIC source)
target_link_libraries(lib PRIVATE project_options project_warnings)
target_link_libraries(lib PUBLIC SDL2 SDL2::SDL2main spdlog glm)

add_executable(small examples/small.cpp)
target_link_libraries(small lib)

add_executable(random examples/random.cpp)
target_link_libraries(random lib)

add_executable(galton_board examples/galton_board.cpp)
target_link_libraries(galton_board lib)

add_executable(billiard examples/billiard.cpp)
target_link_libraries(billiard lib)

add_executable(rigid examples/rigid.cpp)
target_link_libraries(rigid lib)

add_executable(rigid2 examples/rigid2.cpp)
target_link_libraries(rigid2 lib)

if (WIN32)
    add_custom_command(TARGET galton_board POST_BUILD 
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}\\_deps\\sdl2-build\\SDL2.dll" 
            "${CMAKE_BINARY_DIR}\\SDL2.dll")
endif()
