## library
add_library(${PROJECT_NAME} SHARED "")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        # ${PROJECT_SOURCE_DIR}/3rd/spdlog/include  # Disabled due to fmt compatibility issues
)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        Threads::Threads
        ${OpenCV_LIBS}
        Eigen3::Eigen
        yaml-cpp
        core
        stuff
        types_sba
        types_slam3d
        solver_csparse
        solver_cholmod
        solver_eigen
        fmt
)

## sub directories
add_subdirectory(rdvio)
add_subdirectory(rdvio_estimation)
add_subdirectory(rdvio_extra)
add_subdirectory(rdvio_geometry)
add_subdirectory(rdvio_map)
add_subdirectory(rdvio_util)
add_subdirectory(rdvio_loop_closure)