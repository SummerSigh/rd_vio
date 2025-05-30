# rdvio loop closure
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/keyframe_database.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/loop_closure_system.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/loop_detector.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/orb_extractor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pose_graph_optimizer.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        DBoW3
)