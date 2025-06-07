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

# Enable float printing, which is used to output BMV080 data
# Likely increases the binary by 1-2KB, but avoids manual conversion
# of float to int format strings
target_link_options(test_bmv080 PRIVATE -u _printf_float)
