cmake_minimum_required(VERSION 3.16)

project(SparseWatertight LANGUAGES CXX)

# =========================================================
# C++ 标准
# =========================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =========================================================
# MSVC 统一运行库（必须在 add_subdirectory 之前）
# =========================================================
if (MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# =========================================================
# vcpkg 工具链（可选）
# =========================================================
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE 
        "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "")
endif()

# =========================================================
# Polyscope（本地 third_party）
# =========================================================
add_subdirectory(third_party/polyscope)

# =========================================================
# 查找依赖
# =========================================================
find_package(OpenMP REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(libigl CONFIG REQUIRED)
find_package(CGAL CONFIG REQUIRED)

# =========================================================
# 源文件
# =========================================================
file(GLOB_RECURSE SRC_FILES
    "src/*.cpp"
    "src/*.h"
    "src/*.hpp"
)

add_executable(SparseWatertight ${SRC_FILES})

# =========================================================
# 链接库
# =========================================================
target_link_libraries(SparseWatertight PRIVATE
    Eigen3::Eigen
    igl::igl_core
    igl::igl_embree
    igl_copyleft::igl_copyleft_cgal
    igl_copyleft::igl_copyleft_core
    CGAL::CGAL
    polyscope
    OpenMP::OpenMP_CXX
)

# =========================================================
# 优化建议
# =========================================================
if (MSVC)
    target_compile_options(SparseWatertight PRIVATE /bigobj)
endif()

# =========================================================
# Release 默认优化
# =========================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()