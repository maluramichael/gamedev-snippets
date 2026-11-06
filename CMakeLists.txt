cmake_minimum_required(VERSION 3.21)
project(gamedev_snippets LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_TESTS OFF CACHE BOOL "" FORCE)
set(SDL_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.4.20
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(SDL3)

enable_testing()
include(cmake/AddDemo.cmake)
include(cmake/AddPythonDemo.cmake)

add_demo(update-loop SOURCES update-loop/main.cpp)
add_demo(delta-time SOURCES delta-time/main.cpp)
add_demo(fixed-timestep SOURCES fixed-timestep/main.cpp)
add_python_demo(update-loop-py)
