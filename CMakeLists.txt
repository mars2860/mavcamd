cmake_minimum_required(VERSION 3.10)
project(mavcamd C)

# Force CMake to support older submodule configuration styles (fixes cJSON build error)
cmake_policy(SET CMP0048 NEW)
set(CMAKE_POLICY_VERSION_MINIMUM "3.5")

# Set C standard and compiler flags
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-address-of-packed-member")

# 1. Configure official header-only MAVLink Submodule paths
# Point directly to the root of c_library_v2 where dialects are located
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavlink)

# 2. Configure cJSON Submodule build
set(ENABLE_CJSON_TEST OFF CACHE BOOL "Disable cJSON testing" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)
add_subdirectory(third_party/cJSON)
include_directories(third_party/cJSON)

# 3. Find standard Linux realtime extensions for clock_gettime() and sockets
find_library(RT_LIBRARY rt)

# 4. Find system libcurl
find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIRS})

# Define the final daemon executable
add_executable(mavcamd main.c mavcam.c)

# Link local static cJSON and system math library
target_link_libraries(mavcamd PRIVATE cjson m ${RT_LIBRARY} ${CURL_LIBRARIES})
