cmake_minimum_required(VERSION 3.5)


project(online_Calibration)
set(CMAKE_CXX_FLAGS "-std=c++17 -g -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -g3")
SET(CMAKE_BUILD_TYPE "Debug")

enable_language(CUDA)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -lineinfo")

include_directories("/usr/include/eigen3")
find_package(OpenCV REQUIRED HINTS /opt/opencv/opencv-3.4.5)

include_directories(/opt/mpc/3rdparty/protobuf/include)
include_directories(/opt/mpc/3rdparty/spdlog/include)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH}/lib)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/image_processor)
include_directories(${PROJECT_SOURCE_DIR}/image_processor/include)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(image_processor)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(run_AVM_Calibration_F src/calibration_fixedF.cpp)
target_link_libraries(run_AVM_Calibration_F ${PROJECT_NAME} image_processor)
# add_executable(run_AVM_Calibration_B src/calibration_fixedB.cpp)
# target_link_libraries(run_AVM_Calibration_B ${PROJECT_NAME})
