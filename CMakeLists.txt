cmake_minimum_required(VERSION 3.16)

project(Stage_LAB619 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(main
    src/main.cpp
    src/camera/WebcamCamera.cpp
    src/camera/RecordedRGB.cpp
    src/thermalCam/fakeThermalCamera.cpp
    src/thermalCam/RecordedThermal.cpp
    src/thermalCam/generateFakeHeatmapFromImage.cpp
    src/processing/HotspotDetector.cpp
    src/processing/PCBDetector.cpp
    src/processing/Registration.cpp
    src/processing/JointBilateralUpsample.cpp
    src/camera/IPWebcamCamera.cpp
)

target_include_directories(main PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/camera" 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/thermalCam"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/processing"
)

target_link_libraries(main PRIVATE
    ${OpenCV_LIBS}
)