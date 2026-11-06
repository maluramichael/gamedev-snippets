# add_demo(<name> SOURCES <files>...)
#
# Native: executable plus a CTest smoke test that runs 120 frames headless.
# Emscripten: build/web/<name>/index.{html,js,wasm} plus demo.json.
function(add_demo name)
    cmake_parse_arguments(ARG "" "" "SOURCES" ${ARGN})
    if(NOT ARG_SOURCES)
        message(FATAL_ERROR "add_demo(${name}): SOURCES missing")
    endif()

    add_executable(${name} ${ARG_SOURCES})
    target_compile_features(${name} PRIVATE cxx_std_20)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(${name} PRIVATE SDL3::SDL3)

    if(EMSCRIPTEN)
        set(out "${PROJECT_SOURCE_DIR}/build/web/${name}")
        set_target_properties(${name} PROPERTIES
            OUTPUT_NAME index
            SUFFIX ".html"
            RUNTIME_OUTPUT_DIRECTORY "${out}")
        target_link_options(${name} PRIVATE
            "--shell-file=${PROJECT_SOURCE_DIR}/cmake/shell.html"
            -sALLOW_MEMORY_GROWTH=1)
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
