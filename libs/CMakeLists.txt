#[[
********************************************************************************
Root Level CMake Configuration for Libraries
********************************************************************************

This CMakeLists.txt manages all libraries in the project. It establishes build
order and handles interdependencies between libraries.

--------------------------------------------------------------------------------
Adding a New Library:
--------------------------------------------------------------------------------
1. Create your library directory structure:
   libs/
   └── your_library/
       ├── CMakeLists.txt
       ├── Inc/
       └── Src/

2. Create your library's CMakeLists.txt:
example:
   cmake_minimum_required(VERSION 3.22)
   project(your_library)

   add_library(your_library STATIC
       ${CMAKE_CURRENT_SOURCE_DIR}/Src/your_source.c
   )

   target_include_directories(your_library PUBLIC
       ${CMAKE_CURRENT_SOURCE_DIR}/Inc
   )

   # If your library needs common utilities:
   target_link_libraries(your_library PUBLIC
       common   # Link your library against the common library if needed
       stm32cubemx  # STM32 HAL is needed
   )

3. Add your library to this file using add_subdirectory()

--------------------------------------------------------------------------------
Dependencies:
--------------------------------------------------------------------------------
- The 'common' library should be built first as it provides shared utilities
- Libraries that depend on 'common' should list it in their target_link_libraries()
- Build order is determined by add_subdirectory() order in this file
********************************************************************************
]]

cmake_minimum_required(VERSION 3.22)

# Build options for the GPS libraries
option(BUILD_BLOCKING_GPS_LIB "Build the blocking GPS library" OFF)
option(BUILD_NON_BLOCKING_GPS_LIB "Build the non-blocking GPS library" OFF)

# Add the common and gps subdirectories
add_subdirectory(common)

# Conditional blocks for building the GPS libraries
if(BUILD_BLOCKING_GPS_LIB)
    add_subdirectory(gps/blocking)
endif()

if(BUILD_NON_BLOCKING_GPS_LIB)
    add_subdirectory(gps/non-blocking)
endif()
