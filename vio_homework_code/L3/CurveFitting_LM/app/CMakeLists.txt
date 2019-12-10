add_executable(testCurveFitting CurveFitting.cpp)
target_link_libraries(testCurveFitting ${PROJECT_NAME}_backend)

add_executable(CurveFittingHomework CurveFitting2.cpp)
target_link_libraries(CurveFittingHomework ${PROJECT_NAME}_backend)
