cmake_minimum_required(VERSION 3.22)
project(sensors)

add_library(sensors STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/bme688.c
)

target_include_directories(sensors PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc  # sensor specific headers
)

target_link_libraries(sensors PUBLIC
    common  # Inherit common's includes
)
