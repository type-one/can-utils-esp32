cmake_minimum_required(VERSION 3.16)

# ESP32 espressif SDK boiler plate
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# example: for the ESP32S3
# source ~/esp-idf/export.sh
# idf.py set-target esp32s3
# idf.py build
# idf.py flash monitor

# set the project name and version
project(can_utils_esp32 VERSION 1.0)
