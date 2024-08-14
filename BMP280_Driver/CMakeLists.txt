# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Set the project name
project(BMP280_Driver)

# Add the source files for the library
add_library(${PROJECT_NAME} STATIC
    Src/BMP280.cpp
    Src/SPIHandler.cpp
)

# Set the include directories for the library
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

add_library(SHARC::ENV ALIAS BMP280_Driver)
