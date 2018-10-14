set(GTEST_SHUFFLE 1)

add_executable(
    smartLane-test
    main.cpp
    LaneDetectionModuleTest.cpp
    ../app/LaneDetectionModule.cpp
)

target_include_directories(smartLane-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(smartLane-test PUBLIC gtest ${OpenCV_LIBS})
