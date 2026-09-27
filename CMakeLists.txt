cmake_minimum_required(VERSION 3.16)

project(Stage_LAB619 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")

find_package(OpenCV REQUIRED)

# Device selection
set(DEVICE "UBUNTU" CACHE STRING "Target device: UBUNTU or RASPBERRY_PI")

add_executable(main) 

target_sources(main PRIVATE 

    src/GUI.cpp

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

    src/thermalCam/ThermalCamera.cpp



)

# Raspberry Pi hardware
if(DEVICE STREQUAL "RASPBERRY_PI")
    add_custom_target(bcm
        COMMAND ${CMAKE_COMMAND} -E echo "Downloading bcm2835..."
        COMMAND wget -O ${CMAKE_CURRENT_BINARY_DIR}/${BCM2835_ARCHIVE}
                ${BCM2835_URL}

        COMMAND ${CMAKE_COMMAND} -E echo "Extracting bcm2835..."
        COMMAND tar xzf ${CMAKE_CURRENT_BINARY_DIR}/${BCM2835_ARCHIVE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}

        COMMAND ${CMAKE_COMMAND} -E echo "Configuring bcm2835..."
        COMMAND ./configure
        COMMAND make
        COMMAND sudo make install
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${BCM2835_DIR}

        COMMENT "Building and installing bcm2835"
    )

    target_sources(main PRIVATE

        src/drivers/MLX90641/MLX90641_API.cpp

        src/drivers/MLX90641/MLX90641_I2C_Driver.cpp

        # src/drivers/MLX90641/MLX90641_SWI2C_Driver.cpp

        src/thermalCam/ThermalCamera.cpp

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

    "${CMAKE_CURRENT_SOURCE_DIR}/include/drivers/MLX90641"

)

target_link_libraries(main PRIVATE

    ${OpenCV_LIBS}

)

