cmake_minimum_required(VERSION 3.20...3.30)

project(RagdollBulletASI
    VERSION 0.1.0
    DESCRIPTION "Ragdoll physics with Bullet for GTA San Andreas ASI plugin"
    LANGUAGES CXX
)

# =============================================================================
# C++23 & Ninja settings
# =============================================================================
set(CMAKE_CXX_STANDARD          23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# Ninja sebagai generator utama
set(CMAKE_GENERATOR "Ninja" CACHE STRING "Preferred generator" FORCE)

# Output folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Set extension .asi
set(CMAKE_PROJECT_NAME "${PROJECT_NAME}.asi")

# =============================================================================
# Fetch plugin-sdk (branch latest-cmake)
# =============================================================================
include(FetchContent)

FetchContent_Declare(
    plugin-sdk
    GIT_REPOSITORY https://github.com/AthallahDzaki/plugin-sdk.git
    GIT_TAG        latest-cmake
    GIT_SHALLOW    TRUE
)

message(STATUS "Fetching plugin-sdk (branch: latest-cmake)...")
set(PSDK_BUILD_SA ON)
FetchContent_MakeAvailable(plugin-sdk)

set(PLUGIN_SDK_DIR ${plugin-sdk_SOURCE_DIR} CACHE PATH "Path to plugin-sdk" FORCE)
message(STATUS "PLUGIN_SDK_DIR = ${PLUGIN_SDK_DIR}")

# =============================================================================
# Fetch Bullet Physics
# =============================================================================
FetchContent_Declare(
    bullet
    GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
)

set(BUILD_SHARED_LIBS               OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET2_DEMOS             OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS                 OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS                    OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS             OFF CACHE BOOL "" FORCE)
set(BUILD_UNIT_TESTS                OFF CACHE BOOL "" FORCE)
set(INSTALL_LIBS                    OFF CACHE BOOL "" FORCE)
set(USE_GLUT                        OFF CACHE BOOL "" FORCE)
set(USE_GRAPHICAL_BENCHMARK         OFF CACHE BOOL "" FORCE)

message(STATUS "Fetching Bullet Physics...")
FetchContent_MakeAvailable(bullet)

# =============================================================================
# MINHOOK
# =============================================================================
# Download MinHook
include(FetchContent)
FetchContent_Declare(
    minhook
    GIT_REPOSITORY https://github.com/TsudaKageyu/minhook.git
    GIT_TAG master
)
FetchContent_MakeAvailable(minhook)

# =============================================================================
# ASI Plugin target
# =============================================================================
add_library(${PROJECT_NAME} SHARED
    src/main.cpp
    src/BoneNodePhysics.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${PLUGIN_SDK_DIR}/plugin_sa
        ${PLUGIN_SDK_DIR}/shared
        ${PLUGIN_SDK_DIR}/vendor/rw
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${bullet_SOURCE_DIR}/src
        ${bullet_BINARY_DIR}/src   # generated headers jika ada
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        plugin_sa

        BulletDynamics
        BulletCollision
        LinearMath

        minhook
)

# Compile definitions
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        GTASA
        $<$<CONFIG:Debug>:_DEBUG>
)

# Compile options (Ninja-friendly)
target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /MP /EHsc /Zc:__cplusplus>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Wshadow>
)

# =============================================================================
# Copy .asi ke GTA SA folder setelah build (ubah path sesuai komputer kamu)
# =============================================================================
set(GTA_SA_PATH "D:/GTA SA RANDOMISER/" 
    CACHE PATH "Lokasi instalasi GTA San Andreas")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${PROJECT_NAME}>
        "${GTA_SA_PATH}/$<TARGET_FILE_NAME:${PROJECT_NAME}>"
    COMMENT "Menyalin ${PROJECT_NAME}.asi ke folder GTA SA..."
)

# =============================================================================
# Pesan akhir
# =============================================================================
message(STATUS "")
message(STATUS "==============================================================")
message(STATUS "Proyek           : ${PROJECT_NAME}")
message(STATUS "C++ Standard     : 23")
message(STATUS "Generator        : Ninja")
message(STATUS "plugin-sdk       : branch latest-cmake")
message(STATUS "Bullet           : master (FetchContent)")
message(STATUS "Output .asi      : ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "")
message(STATUS "Cara build:")
message(STATUS "  mkdir build && cd build")
message(STATUS "  cmake .. -G Ninja")
message(STATUS "  ninja               # atau ninja -j8 untuk lebih cepat")
message(STATUS "  ninja install       # kalau mau copy ke game")
message(STATUS "==============================================================")