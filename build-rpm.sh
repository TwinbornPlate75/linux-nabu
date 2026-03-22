#!/bin/bash
set -e

# 内核 RPM 构建脚本 - 在 aarch64 目标设备上原生编译
# 用法: ./build-rpm.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPEC_FILE="$SCRIPT_DIR/kernel-sm8150.spec"

echo "=== Kernel SM8150 RPM Build Script ==="

# 检查源码目录
if [ ! -d "$SCRIPT_DIR" ]; then
    echo "Error: Source directory $SCRIPT_DIR not found"
    exit 1
fi

# 查找 spec 文件
if [ ! -f "$SPEC_FILE" ]; then
    echo "Error: Spec file not found at $SPEC_FILE"
    exit 1
fi

echo "=== Checking RPM build environment ==="

# 设置 RPM 构建目录
mkdir -p ~/rpmbuild/{SPECS,RPMS,SOURCES,SRPMS,BUILD,BUILDROOT}

# 复制 spec 文件
cp "$SPEC_FILE" ~/rpmbuild/SPECS/

echo "=== Starting kernel build ==="
cd ~/rpmbuild

# 构建 RPM
rpmbuild -bb ~/rpmbuild/SPECS/kernel-sm8150.spec \
    --without debuginfo

echo "=== Build complete ==="
echo "=== RPM packages are in ~/rpmbuild/RPMS/aarch64/ ==="
ls -la ~/rpmbuild/RPMS/aarch64/
