cmake_minimum_required(VERSION 3.0.0)

project(ssne_ai_yolo_coco)

include(cmake_config/Paths.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
add_compile_options(-Wno-psabi)

include_directories(${M1_SDK_INC_DIR})
include_directories(${M1_SDK_INC_DIR}/smartsoc)
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(
  ssne_ai_yolo_coco
  ${PROJECT_SOURCE_DIR}/demo_yolo_coco.cpp
  ${PROJECT_SOURCE_DIR}/src/coco_detector.cpp
  ${PROJECT_SOURCE_DIR}/src/gpio_alarm_controller.cpp
  ${PROJECT_SOURCE_DIR}/src/osd-device.cpp
  ${PROJECT_SOURCE_DIR}/src/pipeline_image.cpp
  ${PROJECT_SOURCE_DIR}/src/utils.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${M1_SSNE_LIB}
  ${M1_CMABUFFER_LIB}
  ${M1_OSD_LIB}
  ${M1_GPIO_LIB}
  ${M1_SSZLOG_LIB}
  ${M1_ZLOG_LIB}
  ${M1_EMB_LIB}
)

install(TARGETS ssne_ai_yolo_coco DESTINATION bin)
