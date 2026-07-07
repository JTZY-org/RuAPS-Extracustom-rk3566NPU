cmake_minimum_required(VERSION 3.15)

project(ExtracustomUserApp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)

add_library(UserApp SHARED
    UserApp.cpp
    src/image/RgaProcessor.cpp
    src/npu/YoloEngine.cpp
)

target_compile_features(UserApp PRIVATE cxx_std_17)
target_compile_options(UserApp PRIVATE -O3)

target_include_directories(UserApp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/librga/im2d_api
    ${CMAKE_CURRENT_SOURCE_DIR}/librga/include
)

set(RKNN_MODEL_ZOO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/rknn_model_zoo")

set(TARGET_SOC "rk356x" CACHE STRING "Rockchip SoC" FORCE)

set(CMAKE_BUILD_TARGET buildroot CACHE STRING "RGA build target" FORCE)
set(RGA_SAMPLES_ENABLE OFF CACHE BOOL "RGA samples enable" FORCE)
add_subdirectory(librga)

add_subdirectory("${RKNN_MODEL_ZOO_ROOT}/3rdparty" userapp_rknn_3rdparty.out)
add_subdirectory("${RKNN_MODEL_ZOO_ROOT}/utils" userapp_rknn_utils.out)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/yolo_npu" userapp_yolo_npu.out)
target_link_libraries(UserApp PRIVATE userapp_yolo_npu rga)

target_compile_definitions(UserApp PRIVATE
    USERAPP_DEFAULT_YOLO_MODEL="/etc/rknn/yolov8n.rknn"
    USERAPP_DEFAULT_YOLO_LABELS="/etc/rknn/coco_80_labels_list.txt"
)

set_target_properties(UserApp PROPERTIES
    OUTPUT_NAME "UserApp"
    INSTALL_RPATH "$ORIGIN:$ORIGIN/Thirdparty/lib"
)

# Installation Rules
install(TARGETS UserApp DESTINATION lib)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/lib/librknnrt.so" DESTINATION lib)
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/model/yolov8n.rknn"
    "${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/model/coco_80_labels_list.txt"
    DESTINATION etc/rknn
)

# CPack configuration
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "extracustom-userapp")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_CONTACT "support@extracustom.com")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Extracustom UserApp and NPU dependencies")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Developer")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_DEBIAN_FILE_NAME "DEB-DEFAULT")

# Ensure installation goes to /lib and /etc instead of /usr/lib and /usr/etc
set(CPACK_SET_DESTDIR ON)
set(CPACK_INSTALL_PREFIX "/")

include(CPack)

