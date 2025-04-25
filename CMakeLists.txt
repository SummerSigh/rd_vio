# =========================  ROOT  CMakeLists.txt  ============================
cmake_minimum_required(VERSION 3.16)        # 3.16+ for CMAKE_<LANG>_COMPILER_LAUNCHER
cmake_policy(SET CMP0072 NEW)               # OpenSSL / GnuTLS selection
cmake_policy(SET CMP0079 NEW)               # Public link interface in targets

project(rdvio LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# ░░ 1. Compiler and build-type settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(DEFAULT_BUILD_TYPE "Debug")
if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected — defaulting to ${DEFAULT_BUILD_TYPE}")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "" FORCE)
endif()

# Per-target optimisation flags — attached later with target_compile_options()
set(RDVIO_OPT_FLAGS
    -Og          # debug-friendly optimisation
    -msse -msse2 -msse3
    -ffast-math  # be aware of FP strictness!
    -mtune=native)

# Silence noisy warnings from some third-party headers
set(RDVIO_SUPPRESS_WARN "-Wno-unused-result -Wno-ignored-attributes")

# ---------------------------------------------------------------------------
# ░░ 2. ccache (optional)
# ---------------------------------------------------------------------------
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "ccache found — enabling compiler launcher")             # :contentReference[oaicite:4]{index=4}
    set(CMAKE_C_COMPILER_LAUNCHER   "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# ---------------------------------------------------------------------------
# ░░ 3. Third-party dependencies
# ---------------------------------------------------------------------------
find_package(Threads        REQUIRED)                                       # :contentReference[oaicite:5]{index=5}
find_package(OpenCV 4       REQUIRED)                                       # :contentReference[oaicite:6]{index=6}
find_package(Eigen3         REQUIRED)                                       # :contentReference[oaicite:7]{index=7}
find_package(Ceres          REQUIRED)                                       # :contentReference[oaicite:8]{index=8}
find_package(yaml-cpp       REQUIRED)                                       # :contentReference[oaicite:9]{index=9}
find_package(realsense2     REQUIRED)                                       # :contentReference[oaicite:10]{index=10}

# Bundle all external libraries into one list (order matters → RS2 last)
set(RDVIO_THIRD_PARTY
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Ceres::ceres
    yaml-cpp
    Threads::Threads
    realsense2::realsense2)

# ---------------------------------------------------------------------------
# ░░ 4. Multithreading toggle
# ---------------------------------------------------------------------------
option(USE_MULTITHREADING "Enable multithreading in rdvio" ON)
if(USE_MULTITHREADING)
    add_compile_definitions(USE_MULTI_THREADING)
endif()

# ---------------------------------------------------------------------------
# ░░ 5. Build the core library
# ---------------------------------------------------------------------------
add_subdirectory(src)   # assumes src/CMakeLists.txt defines 'rdvio' target

# Apply global compile flags to every target created **after** this point
add_library(rdvio_compile_flags INTERFACE)
target_compile_options(rdvio_compile_flags INTERFACE
    ${RDVIO_OPT_FLAGS} ${RDVIO_SUPPRESS_WARN})
# Any target that wants the flags just links PRIVATE rdvio_compile_flags

# ---------------------------------------------------------------------------
# ░░ 6. Examples
# ---------------------------------------------------------------------------
add_subdirectory(examples)
