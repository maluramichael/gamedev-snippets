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

# Every folder with a demo.json is a demo: main.cpp (C++/SDL3) or main.py (Python/Pyodide).
# Extra *.cpp next to main.cpp are linked in (except test.cpp, see AddDemo.cmake). A file named requires-gl marks
# a GPU demo (OpenGL ES 3 / WebGL2), which is only built for the web.
file(GLOB demo_candidates RELATIVE "${PROJECT_SOURCE_DIR}" CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/*")
list(SORT demo_candidates)
foreach(dir ${demo_candidates})
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${dir}/demo.json")
        continue()
    endif()
    if(EXISTS "${PROJECT_SOURCE_DIR}/${dir}/main.cpp")
        file(GLOB demo_sources RELATIVE "${PROJECT_SOURCE_DIR}" CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/${dir}/*.cpp")
        list(FILTER demo_sources EXCLUDE REGEX "/test\\.cpp$")
        set(demo_gl "")
        if(EXISTS "${PROJECT_SOURCE_DIR}/${dir}/requires-gl")
            set(demo_gl GL)
        endif()
        add_demo(${dir} SOURCES ${demo_sources} ${demo_gl})
    elseif(EXISTS "${PROJECT_SOURCE_DIR}/${dir}/main.py")
        add_python_demo(${dir})
    endif()
endforeach()
