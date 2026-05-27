#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

TARGET_IP="192.168.222.1"
TARGET_USER="root"
TARGET_DEST="${TARGET_USER}@${TARGET_IP}"

# Source directories
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${WORKSPACE_DIR}/build"
THIRDPARTY_DIR="${WORKSPACE_DIR}/Thirdparty"

echo "========================================="
echo " Deploying Extracustom-rk3566NPU to Target"
echo " Target: ${TARGET_DEST}"
echo "========================================="

# 1. Deploy all .so files to /lib/
echo ">>> Step 1: Deploying shared libraries (.so) to target's /lib/..."

# Collect target .so files
SO_FILES=(
    "${BUILD_DIR}/libUserApp.so"
    "${THIRDPARTY_DIR}/lib/librga.so"
    "${THIRDPARTY_DIR}/lib/librknnrt.so"
)

for so_file in "${SO_FILES[@]}"; do
    if [ -f "$so_file" ]; then
        echo "Uploading: $(basename "$so_file") -> /lib/"
        proxychains scp "$so_file" "${TARGET_DEST}:/lib/"
    else
        echo "Error: Required shared library '$so_file' not found!"
        exit 1
    fi
done

# 2. Deploy model and label files to /etc/rknn
echo ""
echo ">>> Step 2: Deploying model and label files to target's /etc/rknn/..."

# Ensure target directory /etc/rknn exists
echo "Creating remote directory /etc/rknn..."
proxychains ssh "${TARGET_DEST}" "mkdir -p /etc/rknn"

MODEL_FILES=(
    "${THIRDPARTY_DIR}/model/yolov8n.rknn"
    "${THIRDPARTY_DIR}/model/coco_80_labels_list.txt"
)

for model_file in "${MODEL_FILES[@]}"; do
    if [ -f "$model_file" ]; then
        echo "Uploading: $(basename "$model_file") -> /etc/rknn/"
        proxychains scp "$model_file" "${TARGET_DEST}:/etc/rknn/"
    else
        echo "Error: Required model/label file '$model_file' not found!"
        exit 1
    fi
done

echo ""
echo "========================================="
echo " Deployment Completed Successfully!"
echo "========================================="
