# Find RealSense library
find_package(realsense2 QUIET)

# ───────────────────── EuRoC ─────────────────────
add_executable(test_euroc
    ${CMAKE_CURRENT_SOURCE_DIR}/test_euroc.cpp
)
target_link_libraries(test_euroc
    PRIVATE
        ${PROJECT_NAME}
        $<$<BOOL:${realsense2_FOUND}>:realsense2::realsense2>   # <-- add this
)

# ───────────────────── ADVIO ─────────────────────
add_executable(test_advio
    ${CMAKE_CURRENT_SOURCE_DIR}/test_advio.cpp
)
target_link_libraries(test_advio
    PRIVATE
        ${PROJECT_NAME}
        $<$<BOOL:${realsense2_FOUND}>:realsense2::realsense2>   # <-- add if needed
)

# ─────────────────── RealSense demos ─────────────
if(realsense2_FOUND)
    add_executable(realsense_d455     ${CMAKE_CURRENT_SOURCE_DIR}/realsense_d455.cpp)
    target_link_libraries(realsense_d455  PRIVATE ${PROJECT_NAME} realsense2::realsense2)

    message(STATUS "RealSense support enabled")
else()
    message(STATUS "RealSense library not found – skipping RealSense examples")
endif()
