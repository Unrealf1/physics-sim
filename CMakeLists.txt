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
add_executable(main ${SOURCES})

target_link_libraries(main project_options project_warnings)
target_link_libraries(main SDL2 spdlog glm)

