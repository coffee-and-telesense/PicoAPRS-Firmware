# Create an executable object type
add_executable(test_bmv080 test_bmv080.c)

target_sources(test_bmv080 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test_bmv080.c
)

target_link_libraries(test_bmv080 PRIVATE 
    stm32cubemx
    common
    bmv080
)
