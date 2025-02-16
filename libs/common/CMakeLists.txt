cmake_minimum_required(VERSION 3.22)
project(common)

add_library(common STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/logging.c
)

target_compile_definitions(common PUBLIC
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Config/Inc
)

target_compile_definitions(common PUBLIC DEBUG)

# Link against the stm32cubemx INTERFACE Library
target_link_libraries(common PUBLIC
    stm32cubemx
)
