# add_demo(<name> SOURCES <files>... [GL])
#
# Native: executable plus a CTest smoke test that runs 120 frames headless. A GPU demo (GL) needs OpenGL ES 3, which
# the headless container does not have, so it is only built for the web; its logic is covered by test.cpp.
# <name>/test.cpp (optional, native only): plain assert-based test of the demo's logic headers, no SDL needed,
# run by CTest as <name>-logic. Include the demo's own headers as "logic.hpp" (the folder is on the include path).
# Emscripten: build/web/<name>/index.{html,js,wasm} plus demo.json.
function(add_demo name)
    cmake_parse_arguments(ARG "GL" "" "SOURCES" ${ARGN})
    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "add_demo(${name}): SOURCES missing")
    endif()

    if(NOT EMSCRIPTEN AND EXISTS "${PROJECT_SOURCE_DIR}/${name}/test.cpp")
        add_executable(${name}-logic "${PROJECT_SOURCE_DIR}/${name}/test.cpp")
        target_compile_features(${name}-logic PRIVATE cxx_std_20)
        target_compile_options(${name}-logic PRIVATE -Wall -Wextra -Wpedantic)
        target_include_directories(${name}-logic PRIVATE "${PROJECT_SOURCE_DIR}/${name}" "${PROJECT_SOURCE_DIR}/common")
        add_test(NAME ${name}-logic COMMAND ${name}-logic)
    endif()

    if(ARG_GL AND NOT EMSCRIPTEN)
        message(STATUS "${name}: GPU demo, built for the web only")
        return()
    endif()

    add_executable(${name} ${ARG_SOURCES})
    target_compile_features(${name} PRIVATE cxx_std_20)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(${name} PRIVATE SDL3::SDL3)
    target_include_directories(${name} PRIVATE "${PROJECT_SOURCE_DIR}/${name}" "${PROJECT_SOURCE_DIR}/common")

    if(EMSCRIPTEN)
        set(out "${PROJECT_SOURCE_DIR}/build/web/${name}")
        set_target_properties(${name} PROPERTIES
            OUTPUT_NAME index
            SUFFIX ".html"
            RUNTIME_OUTPUT_DIRECTORY "${out}")
        target_link_options(${name} PRIVATE
            "--shell-file=${PROJECT_SOURCE_DIR}/cmake/shell.html"
            -sALLOW_MEMORY_GROWTH=1)
        if(ARG_GL)
            target_link_options(${name} PRIVATE -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2)
        endif()
        set(extras "")
        foreach(extra thumb.png cover.webp)
            if(EXISTS "${PROJECT_SOURCE_DIR}/${name}/${extra}")
                list(APPEND extras "${PROJECT_SOURCE_DIR}/${name}/${extra}")
            endif()
        endforeach()
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${PROJECT_SOURCE_DIR}/${name}/demo.json" ${extras} "${out}/")
    else()
        add_test(NAME ${name} COMMAND ${name} --frames 120)
        set_tests_properties(${name} PROPERTIES ENVIRONMENT "SDL_VIDEODRIVER=dummy")
    endif()
endfunction()
