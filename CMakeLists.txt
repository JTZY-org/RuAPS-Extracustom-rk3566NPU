cmake_minimum_required(VERSION 3.15)

project(ExtracustomUserApp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(UserApp SHARED
    UserApp.cpp
)

target_compile_features(UserApp PRIVATE cxx_std_17)

target_include_directories(UserApp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(RKNN_MODEL_ZOO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/rknn_model_zoo")

set(TARGET_SOC "rk356x" CACHE STRING "Rockchip SoC" FORCE)

add_subdirectory("${RKNN_MODEL_ZOO_ROOT}/3rdparty" userapp_rknn_3rdparty.out)
add_subdirectory("${RKNN_MODEL_ZOO_ROOT}/utils" userapp_rknn_utils.out)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/yolo_npu" userapp_yolo_npu.out)
target_link_libraries(UserApp PRIVATE userapp_yolo_npu)

target_compile_definitions(UserApp PRIVATE
    USERAPP_DEFAULT_YOLO_MODEL="/etc/rknn/yolov8n.rknn"
    USERAPP_DEFAULT_YOLO_LABELS="/etc/rknn/coco_80_labels_list.txt"
)

set_target_properties(UserApp PROPERTIES
    OUTPUT_NAME "UserApp"
    INSTALL_RPATH "$ORIGIN:$ORIGIN/Thirdparty/lib"
)
