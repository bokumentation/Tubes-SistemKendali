cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(.)

# Fix compile commands for clangd.
# Removes '-m*' and '-f*' flags from compile_commands.json so clangd
# doesn't choke on target-specific compiler flags during IDE indexing.
if(EXISTS "${CMAKE_SOURCE_DIR}/fix_compile_commands.py")
    find_package(Python3 QUIET)
    if(Python3_FOUND)
        add_custom_target(fix_clangd ALL
            COMMAND ${CMAKE_COMMAND} -E echo "Running fix_compile_commands.py..."
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/fix_compile_commands.py
            COMMENT "Cleaning compile_commands.json for clangd"
            VERBATIM
        )
    endif()
endif()
