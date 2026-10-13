cmake_minimum_required(VERSION 3.16)

project(Stage_LAB619 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")

find_package(OpenCV REQUIRED)

# ---------------------------------------------------------------------------
# Device selection
# ---------------------------------------------------------------------------

set(DEVICE "UBUNTU" CACHE STRING "Target device: UBUNTU or RASPBERRY_PI")
set_property(CACHE DEVICE PROPERTY STRINGS UBUNTU RASPBERRY_PI)

# ---------------------------------------------------------------------------
# Shared code: everything except the two entry points
# ---------------------------------------------------------------------------

add_library(thermal_core STATIC
    src/camera/WebcamCamera.cpp
    src/camera/IPWebcamCamera.cpp
    src/camera/RecordedRGB.cpp

    src/thermalCam/fakeThermalCamera.cpp
    src/thermalCam/RecordedThermal.cpp
    src/thermalCam/generateFakeHeatmapFromImage.cpp

    src/processing/HotspotDetector.cpp
    src/processing/ChessboardCalibration.cpp
    src/processing/PCBDetector.cpp
    src/processing/Registration.cpp
    src/processing/JointBilateralUpsample.cpp
)

target_include_directories(thermal_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/camera"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/thermalCam"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/processing"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/drivers/MLX90641"
)

target_link_libraries(thermal_core PUBLIC
    ${OpenCV_LIBS}
)

# ---------------------------------------------------------------------------
# Raspberry Pi hardware (MLX90641 over bcm2835)
# ---------------------------------------------------------------------------

if(DEVICE STREQUAL "RASPBERRY_PI")

    set(BCM2835_VERSION "1.71")
    set(BCM2835_DIR     "bcm2835-${BCM2835_VERSION}")
    set(BCM2835_ARCHIVE "${BCM2835_DIR}.tar.gz")
    set(BCM2835_URL     "http://www.airspayce.com/mikem/bcm2835/${BCM2835_ARCHIVE}")

    # Manual one-time step:  make bcm
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

    target_sources(thermal_core PRIVATE
        src/drivers/MLX90641/MLX90641_API.cpp
        src/drivers/MLX90641/MLX90641_I2C_Driver.cpp
        # src/drivers/MLX90641/MLX90641_SWI2C_Driver.cpp
        src/thermalCam/ThermalCamera.cpp
    )

    # PUBLIC so every executable linking thermal_core sees the define too
    target_compile_definitions(thermal_core PUBLIC
        DEVICE_RASPBERRY_PI
    )

    target_link_libraries(thermal_core PUBLIC
        bcm2835
    )

else()

    target_compile_definitions(thermal_core PUBLIC
        DEVICE_UBUNTU
    )

endif()

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------

# Command-line driven app (mode / RGB source / thermal source via args)
add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE thermal_core)
