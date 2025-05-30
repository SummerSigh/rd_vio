# Third party libraries

# Set position independent code for all targets
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build g2o with required components
set(G2O_BUILD_APPS OFF CACHE BOOL "Build g2o apps" FORCE)
set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "Build g2o examples" FORCE)
set(G2O_BUILD_SLAM2D_TYPES OFF CACHE BOOL "Build SLAM2D types" FORCE)
set(G2O_BUILD_SLAM3D_TYPES ON CACHE BOOL "Build SLAM3D types" FORCE)
set(G2O_BUILD_SBA_TYPES ON CACHE BOOL "Build SBA types" FORCE)
set(G2O_BUILD_SIM3_TYPES OFF CACHE BOOL "Build sim3 types" FORCE)
set(G2O_BUILD_ICP_TYPES OFF CACHE BOOL "Build ICP types" FORCE)
set(G2O_USE_OPENGL OFF CACHE BOOL "Build g2o with OpenGL support" FORCE)
set(G2O_USE_CHOLMOD ON CACHE BOOL "Build g2o with CHOLMOD support" FORCE)
set(G2O_USE_CSPARSE ON CACHE BOOL "Build g2o with CSParse support" FORCE)
set(G2O_USE_LOGGING OFF CACHE BOOL "Disable g2o logging to avoid fmt conflicts" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)

add_subdirectory(g2o)

# Build DBoW3
set(BUILD_UTILS OFF CACHE BOOL "Build DBoW3 utils" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Build DBoW3 tests" FORCE)
add_subdirectory(DBoW3)