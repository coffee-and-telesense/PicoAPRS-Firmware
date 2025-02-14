cmake_minimum_required(VERSION 3.22)
project(common)

add_library(common STATIC
    Src/logging.c
)

target_include_directories(common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_compile_definitions(common PUBLIC DEBUG)
