cmake_minimum_required(VERSION 3.15)

if(NOT DEFINED RKNN_MODEL_ZOO_ROOT)
    message(FATAL_ERROR "RKNN_MODEL_ZOO_ROOT must be set before adding Thirdparty/yolo_npu")
endif()

if(NOT TARGET imageutils OR NOT TARGET fileutils OR NOT TARGET imagedrawing)
    message(FATAL_ERROR "rknn_model_zoo utils targets must be added before Thirdparty/yolo_npu")
endif()

find_package(Threads REQUIRED)

set(_yolov8_cpp "${RKNN_MODEL_ZOO_ROOT}/examples/yolov8/cpp")

add_library(userapp_yolo_npu STATIC
    yolo_npu.cpp
    "${_yolov8_cpp}/postprocess.cc"
    "${_yolov8_cpp}/rknpu2/yolov8.cc"
)

set_target_properties(userapp_yolo_npu PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(userapp_yolo_npu PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_include_directories(userapp_yolo_npu PRIVATE
    "${_yolov8_cpp}"
    "${LIBRKNNRT_INCLUDES}"
    "${RKNN_MODEL_ZOO_ROOT}/utils"
)

target_link_libraries(userapp_yolo_npu PUBLIC
    imageutils
    fileutils
    imagedrawing
    ${LIBRKNNRT}
    dl
    Threads::Threads
)

if(EXISTS "${LIBRKNNRT}")
    get_filename_component(_LIBRKNNRT_DIR "${LIBRKNNRT}" DIRECTORY)
    target_link_options(userapp_yolo_npu INTERFACE "LINKER:-rpath-link=${_LIBRKNNRT_DIR}")
endif()
