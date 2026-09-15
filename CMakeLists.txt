cmake_minimum_required(VERSION 3.16)

project(Stage_LAB619 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# Device selection
set(DEVICE "UBUNTU" CACHE STRING "Target device: UBUNTU or RASPBERRY_PI")

add_executable(main) 

target_sources(main PRIVATE 

    src/main.cpp

    src/camera/WebcamCamera.cpp

    src/camera/RecordedRGB.cpp

    src/thermalCam/fakeThermalCamera.cpp

    src/thermalCam/RecordedThermal.cpp

    src/thermalCam/generateFakeHeatmapFromImage.cpp

    src/processing/HotspotDetector.cpp

    src/processing/ChessboardCalibration.cpp

    src/processing/PCBDetector.cpp

    src/processing/Registration.cpp

    src/processing/JointBilateralUpsample.cpp

    src/camera/IPWebcamCamera.cpp

)

# Raspberry Pi hardware
if(DEVICE STREQUAL "RASPBERRY_PI")

    target_sources(main PRIVATE

        src/drivers/MLX90641/MLX90641_API.cpp

        src/drivers/MLX90641/MLX90641_I2C_Driver.cpp

    )

    target_compile_definitions(main PRIVATE
        DEVICE_RASPBERRY_PI
    )

    target_link_libraries(main PRIVATE
        bcm2835
    )

else()

    target_compile_definitions(main PRIVATE
        DEVICE_UBUNTU
    )

endif()

target_include_directories(main PRIVATE

    "${CMAKE_CURRENT_SOURCE_DIR}/include/camera" 

    "${CMAKE_CURRENT_SOURCE_DIR}/include/thermalCam"

    "${CMAKE_CURRENT_SOURCE_DIR}/include/processing"

    "${CMAKE_CURRENT_SOURCE_DIR}/include/drivers"

)

target_link_libraries(main PRIVATE

    ${OpenCV_LIBS}

)