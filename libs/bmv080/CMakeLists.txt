cmake_minimum_required(VERSION 3.22)
project(bmv080)

add_library(bmv080 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/lib_bmv080.a
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/lib_postProcessor.a
)

target_include_directories(bmv080 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_link_libraries(bmv080 PUBLIC
    stm32cubemx
    common  # Inherit common's includes
)
