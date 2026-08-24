#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

TARGET_IP="192.168.222.1"
TARGET_USER="root"
TARGET_DEST="${TARGET_USER}@${TARGET_IP}"

# Source directories
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${WORKSPACE_DIR}/build"

echo "========================================="
echo " Deploying Extracustom-rk3566NPU via CPack"
echo " Target: ${TARGET_DEST}"
echo "========================================="

# 1. Build and package using CPack
echo ">>> Step 1: Compiling with Ninja..."
ninja -C "${BUILD_DIR}"

echo ">>> Step 2: Generating DEB package with CPack..."
(cd "${BUILD_DIR}" && cpack)

DEB_FILE=$(find "${BUILD_DIR}" -maxdepth 1 -name "extracustom-userapp_*.deb" -print -quit)

if [ -z "$DEB_FILE" ] || [ ! -f "$DEB_FILE" ]; then
    echo "Error: CPack DEB package not found in ${BUILD_DIR}!"
    exit 1
fi

DEB_NAME=$(basename "$DEB_FILE")
echo ">>> Generated package: ${DEB_NAME}"

# 2. Upload DEB package to target /tmp/
echo ""
echo ">>> Step 3: Uploading ${DEB_NAME} to target /tmp/..."
proxychains scp "$DEB_FILE" "${TARGET_DEST}:/tmp/"

# 3. Install package on target via opkg
echo ""
echo ">>> Step 4: Installing package on target via opkg..."
proxychains ssh "${TARGET_DEST}" "opkg install --force-overwrite --force-reinstall --force-downgrade /tmp/${DEB_NAME} && rm -f /tmp/${DEB_NAME} && sync"

echo ""
echo "========================================="
echo " Deployment & Installation Completed Successfully!"
echo "========================================="
