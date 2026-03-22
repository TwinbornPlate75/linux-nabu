%undefine        _debugsource_packages

# 本地源码目录
%global LOCAL_SOURCE_DIR /home/TwinbornPlate75/linux-nabu

# 从 Makefile 动态获取内核版本
%global KERNEL_VER %(awk 'BEGIN {FS=" = "} /^(VERSION|PATCHLEVEL|SUBLEVEL|EXTRAVERSION) =/ {gsub(/[[:space:]]/, "", $2); vars[$1]=$2} END {printf "%s.%s.%s%s", vars["VERSION"], vars["PATCHLEVEL"], vars["SUBLEVEL"], vars["EXTRAVERSION"]}' %{LOCAL_SOURCE_DIR}/Makefile)

%global KERNEL_CUSTOM_VER 1
%global RELEASE_VER 4
%global DEVICE_NAME nabu
%global PLATFORM_NAME sm8150

%global KERNEL_EXTRA_VER -%{PLATFORM_NAME}-%{KERNEL_CUSTOM_VER}-%{RELEASE_VER}
%global KERNEL_FULL_VER %{KERNEL_VER}%{KERNEL_EXTRA_VER}

%global TAG_NAME %{KERNEL_VER}-%{PLATFORM_NAME}-%{KERNEL_CUSTOM_VER}

Version:         %{KERNEL_VER}.%{PLATFORM_NAME}.%{KERNEL_CUSTOM_VER}
Release:         %{RELEASE_VER}.%{DEVICE_NAME}%{?dist}
ExclusiveArch:   aarch64
Name:            kernel-%{PLATFORM_NAME}
Summary:         Mainline Linux kernel for %{PLATFORM_NAME} devices
License:         GPLv2
URL:             https://github.com/jhuang6451/Linux
# 使用本地源码，不下载远程归档
Source0:         %{LOCAL_SOURCE_DIR}

BuildRequires:   bc bison dwarves diffutils elfutils-devel findutils gcc gcc-c++ git-core hmaccalc hostname make openssl-devel perl-interpreter rsync tar which flex bzip2 xz zstd python3 python3-devel python3-pyyaml rust rust-src bindgen rustfmt clippy opencsd-devel net-tools dracut

Provides:        kernel               = %{version}-%{release}
Provides:        kernel-core          = %{version}-%{release}
Provides:        kernel-modules       = %{version}-%{release}
Provides:        kernel-modules-core  = %{version}-%{release}

%description
Mainline kernel for %{PLATFORM_NAME}, packaged for standard Fedora systems with UEFI boot support

%prep
# 不解压，直接使用本地源码目录
# 创建符号链接将源码链接到 rpmbuild 的 BUILD 目录
rm -rf %{_builddir}/Linux-%{TAG_NAME}
ln -sf %{LOCAL_SOURCE_DIR} %{_builddir}/Linux-%{TAG_NAME}
cd %{_builddir}/Linux-%{TAG_NAME}

# 准备默认配置
make defconfig %{PLATFORM_NAME}.config

%build
# 切换到源码目录
cd %{_builddir}/Linux-%{TAG_NAME}

# 禁用 CONFIG_LOCALVERSION_AUTO，防止自动追加 git 版本信息
sed -i 's/CONFIG_LOCALVERSION_AUTO=y/# CONFIG_LOCALVERSION_AUTO is not set/' .config
# 移除既有的 CONFIG_LOCALVERSION，通过 make 命令的参数来控制它
sed -i '/^CONFIG_LOCALVERSION=/d' .config
# 确保没有 localversion 文件影响版本号
rm -f localversion*

make olddefconfig
make EXTRAVERSION="%{KERNEL_EXTRA_VER}" LOCALVERSION= -j%{?_smp_build_ncpus} Image modules dtbs

%install
# 切换到源码目录
cd %{_builddir}/Linux-%{TAG_NAME}

# 1. 安装内核模块到 buildroot
make EXTRAVERSION="%{KERNEL_EXTRA_VER}" LOCALVERSION= \
    INSTALL_MOD_PATH=%{buildroot}/usr \
    modules_install

# 2. 安装内核镜像、System.map 和配置文件到 /boot 目录
install -Dm644 arch/arm64/boot/Image %{buildroot}/boot/vmlinuz-%{KERNEL_FULL_VER}
install -Dm644 System.map %{buildroot}/boot/System.map-%{KERNEL_FULL_VER}
install -Dm644 .config    %{buildroot}/boot/config-%{KERNEL_FULL_VER}

# 3. 安装设备树文件 (DTB)
install -d %{buildroot}/usr/lib/modules/%{KERNEL_FULL_VER}/dtb/qcom
install -Dm644 arch/arm64/boot/dts/qcom/sm8150-xiaomi-%{DEVICE_NAME}.dtb %{buildroot}/usr/lib/modules/%{KERNEL_FULL_VER}/dtb/qcom/sm8150-xiaomi-%{DEVICE_NAME}.dtb

%files
/boot/vmlinuz-%{KERNEL_FULL_VER}
/boot/System.map-%{KERNEL_FULL_VER}
/boot/config-%{KERNEL_FULL_VER}
/usr/lib/modules/%{KERNEL_FULL_VER}

%posttrans
set -e

KERNEL_FULL_VER="%{KERNEL_FULL_VER}"
DEVICE_NAME="%{DEVICE_NAME}"

# --- 为新内核生成模块依赖 ---
depmod -a "${KERNEL_FULL_VER}"

echo "--- Generating UKI for ${KERNEL_FULL_VER} using dracut + ukify ---"

# --- 定义路径 ---
UKI_DIR="/boot/efi/EFI/fedora"
INITRD_PATH="/boot/initramfs-${KERNEL_FULL_VER}.img"
KERNEL_PATH="/boot/vmlinuz-${KERNEL_FULL_VER}"
DTB_PATH="/usr/lib/modules/${KERNEL_FULL_VER}/dtb/qcom/sm8150-xiaomi-${DEVICE_NAME}.dtb"
UKI_OUTPUT_PATH="${UKI_DIR}/fedora-${KERNEL_FULL_VER}.efi"
mkdir -p "$UKI_DIR"

# --- 步骤 1: 使用 dracut 生成 initramfs ---
echo "Generating initramfs with dracut..."
dracut --kver "${KERNEL_FULL_VER}" --force
if [ ! -f "${INITRD_PATH}" ]; then
    echo "CRITICAL: dracut failed to generate initramfs at ${INITRD_PATH}" >&2
    exit 1
fi
echo "Initramfs generated at ${INITRD_PATH}"

# --- 步骤 2: 使用 systemd-ukify 生成 UKI ---
echo "Generating UKI with systemd-ukify..."
ukify build \
    --linux="${KERNEL_PATH}" \
    --initrd="${INITRD_PATH}" \
    --devicetree="${DTB_PATH}" \
    --output="${UKI_OUTPUT_PATH}"

if [ ! -f "${UKI_OUTPUT_PATH}" ]; then
    echo "CRITICAL: ukify failed to generate UKI at ${UKI_OUTPUT_PATH}" >&2
    rm -f "${INITRD_PATH}"
    exit 1
fi
echo "SUCCESS: UKI generated at ${UKI_OUTPUT_PATH}"

# --- 步骤 3: 清理独立的 initramfs ---
echo "Cleaning up standalone initramfs..."
rm -f "${INITRD_PATH}"

echo "--- UKI generation complete for ${KERNEL_FULL_VER} ---"

%postun

%changelog
* Sun Dec 7 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.17.0.sm8150.1-3.nabu
- Comment out screen rotation patch.

* Wed Dec 2 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.17.0-2.nabu
- Switch source to https://github.com/jhuang6451/Linux.

* Thu Oct 16 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.17.0-1.nabu
- Initial release of 6.17.

* Mon Oct 13 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.16.0-6.sm8150
- Add screen rotation property patch to dts.

* Fri Oct 10 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.16.0-5.sm8150
- Change UKI name.
- Change UKI output directory to "/boot/efi/EFI/fedora".

* Thu Sep 25 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.16.0-4.sm8150
- Switched UKI generation to a dracut + systemd-ukify two-step process.
- dracut is now only responsible for creating the initramfs.
- systemd-ukify is used to assemble the kernel, initramfs, cmdline, and DTB into the final UKI.

* Thu Sep 25 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.16.0-3.sm8150
- Replaced `kernel-install` with a direct `dracut` call in %posttrans scriptlet.
- This fixes a critical bug where the device tree (DTB) was not being
  included in the generated UKI, ensuring the kernel is bootable.

* Sat Sep 20 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.16.0-2.sm8150
- Aligned post-install script with kernel-install framework for consistent UKI generation.
- Removed redundant dracut call from %posttrans.

* Sat Sep 13 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.16-1.sm8150
- Modified spec for standard Fedora systems, removing ostree logic.
- Adopted standard file paths (/boot) and kernel-install for UEFI/bootloader integration.

* Fri Sep 12 2025 jhuang6451 <xplayerhtz123@outlook.com> - 6.16.0-0.sm8150
- Added post-transaction script to copy kernel and DTB to ESP for UEFI boot.

* Fri Jul 25 2025 gmanka 6.16.0
- update to 6.16.0
