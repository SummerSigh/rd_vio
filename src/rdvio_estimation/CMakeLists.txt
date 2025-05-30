# rdvio estimation
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/preintegrator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/stub_solver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/g2o_solver_impl.cpp
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/g2o_minimal_solver.cpp  # Replaced by g2o_solver_impl.cpp
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/solver.cpp     # Disabled - requires Ceres
        # ${CMAKE_CURRENT_SOURCE_DIR}/src/g2o_solver.cpp  # Disabled - fmt linking issues
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
