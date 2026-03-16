cmake_minimum_required(VERSION 3.14)
project(OpenGL_Demo LANGUAGES CXX)

include(FetchContent)

# Force DYNAMIC runtime (/MDd di Debug) untuk fix mismatch sebelumnya
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebugDLL")
endif()

# Force Bullet ke dynamic runtime juga
set(USE_MSVC_RUNTIME_LIBRARY_DLL ON CACHE BOOL "Force Bullet to /MD(d)" FORCE)

# Fetch Bullet
FetchContent_Declare(
    bullet
    GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
    GIT_TAG master  # atau pakai tag spesifik seperti "3.25" untuk stabil
)

set(BUILD_UNIT_TESTS       OFF CACHE BOOL "" FORCE)
set(BUILD_CPU_DEMOS        OFF CACHE BOOL "" FORCE)
set(BUILD_OPENGL3_DEMOS    OFF CACHE BOOL "" FORCE)
set(BUILD_BULLET2_DEMOS    OFF CACHE BOOL "" FORCE)
set(BUILD_EXTRAS           OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(bullet)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Fetch FreeGLUT
FetchContent_Declare(
    freeglut
    GIT_REPOSITORY https://github.com/FreeGLUTProject/freeglut.git
    GIT_TAG master
)

set(FREEGLUT_BUILD_DEMOS       OFF CACHE BOOL "" FORCE)
set(FREEGLUT_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(freeglut)

# Sources dengan GLOB
file(GLOB SOURCES "*.cpp" "*.h")

if(NOT SOURCES)
    message(FATAL_ERROR "No sources found! Pastikan file .cpp dan .h ada di folder yang sama.")
endif()

add_executable(AppGenericJointDemo 
    GenericJointDemo.cpp
    Ragdoll.cpp
    Win32GenericJointDemo.cpp
    Win32AppMain.cpp
    Win32DemoApplication.cpp
    DemoApplication.cpp
    GLDebugDrawer.cpp
    GL_DialogDynamicsWorld.cpp
    GL_DialogWindow.cpp
    GL_ShapeDrawer.cpp
    GL_Simplex1to4.cpp
    GLDebugFont.cpp
)

if(MSVC)
    set_target_properties(AppGenericJointDemo PROPERTIES
        LINK_FLAGS_DEBUG "/SUBSYSTEM:WINDOWS"
        LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS"
    )
endif()

target_compile_definitions(AppGenericJointDemo PRIVATE
    BT_USE_DEBUG_COUNTER=1   # kalau Bullet pakai macro ini
)

# Include directories penting
target_include_directories(AppGenericJointDemo PRIVATE
    ${bullet_SOURCE_DIR}/src
    ${bullet_SOURCE_DIR}/examples/CommonInterfaces  # untuk header seperti CommonExampleInterface.h dll
    ${CMAKE_CURRENT_SOURCE_DIR}                     # untuk header project kamu sendiri
)

target_link_libraries(AppGenericJointDemo PRIVATE
    BulletDynamics
    BulletCollision
    LinearMath
    OpenGL::GL
    OpenGL::GLU
    freeglut_static
    # Tambahan Windows libs kalau perlu (sering dibutuhkan untuk freeglut Win32)
    winmm
    gdi32
)

set_target_properties(AppGenericJointDemo PROPERTIES
    CXX_STANDARD          17
    CXX_STANDARD_REQUIRED ON
)