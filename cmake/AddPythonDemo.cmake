# add_python_demo(<name>)
#
# A Python demo is a folder with main.py, helper modules, index.html and demo.json.
# Native: CTest runs the folder's unittest suite (pure logic, no pygame needed).
# Emscripten: the files are copied to build/web/<name>/. Pyodide itself lives once under
# build/web/_shared/pyodide/ (see scripts/fetch-pyodide.sh).
function(add_python_demo name)
    set(src "${PROJECT_SOURCE_DIR}/${name}")
    if(NOT EXISTS "${src}/index.html" OR NOT EXISTS "${src}/demo.json" OR NOT EXISTS "${src}/main.py")
        message(FATAL_ERROR "add_python_demo(${name}): main.py, index.html and demo.json are required")
    endif()

    if(EMSCRIPTEN)
        set(out "${PROJECT_SOURCE_DIR}/build/web/${name}")
        file(GLOB py_files CONFIGURE_DEPENDS "${src}/*.py")
        list(FILTER py_files EXCLUDE REGEX "/test_[^/]*\\.py$")
        add_custom_target(${name} ALL
            COMMAND ${CMAKE_COMMAND} -E make_directory "${out}"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${py_files} "${src}/index.html" "${src}/demo.json" "${out}")
    else()
        find_package(Python3 COMPONENTS Interpreter REQUIRED)
        add_test(NAME ${name}-unittest
            COMMAND ${Python3_EXECUTABLE} -m unittest discover -s "${src}"
            WORKING_DIRECTORY "${src}")
    endif()
endfunction()
