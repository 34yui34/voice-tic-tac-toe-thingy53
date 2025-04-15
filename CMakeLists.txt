# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXTRA_DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/boards/thingy53.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tic-tac-toe-voice-recognition-thingy53)

include(utils/cmake/utils.cmake)

# Needed for colorful output
zephyr_compile_options(-fdiagnostics-color=always)

# Use hardware acceleration for DSP and Neural Network code
# You'll need to disable these on non-Arm cores
add_definitions(-DEI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN=1
                -DARM_MATH_LOOPUNROLL
                )

# Add the Edge Impulse SDK
add_subdirectory(edge-impulse-sdk/cmake/zephyr)

target_include_directories(app PRIVATE .)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/model-parameters)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tflite-model)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/include)

target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/source/ssd1327_oled.cpp)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/source/table_image.cpp)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/source/cross_image.cpp)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/source/circle_image.cpp)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/source/black_square_image.cpp)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tflite-model/tflite_learn_3_compiled.cpp)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/source/dmic_voice_recognition.cpp)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/source/main.cpp)
