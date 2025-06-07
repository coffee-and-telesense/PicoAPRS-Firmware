add_executable(si4463_test test_si4463.c)

target_sources(si4463_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test_si4463.c
)

target_link_libraries(si4463_test PRIVATE 
    stm32cubemx
    si4463
)