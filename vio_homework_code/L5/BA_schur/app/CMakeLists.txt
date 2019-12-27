set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(testMonoBA TestMonoBA.cpp)
target_link_libraries(testMonoBA ${PROJECT_NAME}_backend)

add_executable(testCurveFitting CurveFitting.cpp)
target_link_libraries(testCurveFitting ${PROJECT_NAME}_backend)

add_executable(testCurveFitting_g2o CurveFitting_g2o.cpp)
target_link_libraries(testCurveFitting_g2o ${G2O_LIBS})
