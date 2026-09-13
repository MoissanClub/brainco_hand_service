#!/usr/bin/env bash
# Adapted from BrainCoTech/brainco-hand-sdk/download-lib.sh for this bridge.
set -euo pipefail
export LC_ALL=C

SDK_VERSION="v2.0.5"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    printf '%s\n' \
        "Usage: $0 [--version vX.Y.Z]" \
        "Download the matching SDK header and BOTH Linux libraries (amd64 + arm64)." \
        "Default release: ${SDK_VERSION}. No system files are modified." \
        "Stop the bridge before upgrading, then reconfigure and rebuild it."
}

fail() { printf 'SDK update failed: %s\n' "$*" >&2; exit 1; }

while (($#)); do
    case "$1" in
        --version)
            (($# >= 2)) || fail "--version requires a release such as v2.0.5"
            SDK_VERSION="$2"
            shift 2
            ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; fail "unknown argument: $1" ;;
    esac
done
[[ "$SDK_VERSION" =~ ^v[0-9]+\.[0-9]+\.[0-9]+(-[A-Za-z0-9.-]+)?$ ]] || fail "invalid version: $SDK_VERSION"
[[ "$(uname -s)" == Linux ]] || fail "only Linux is supported"
case "$(uname -m)" in
    x86_64|amd64|aarch64|arm64) ;;
    *) fail "supported Linux architectures: amd64 (x86_64), arm64 (aarch64)" ;;
esac
for dependency in curl unzip readelf sha256sum flock cmp awk grep install mktemp cp mv rm mkdir dirname date; do
    command -v "$dependency" >/dev/null || fail "missing dependency: $dependency"
done

# Lock the script inode without creating a persistent lock file in the repo.
exec 9< "${BASH_SOURCE[0]}"
flock -n 9 || fail "another SDK upgrade is already running"

# Stage on the same filesystem for atomic per-file renames. Download or validation
# failures never modify the current SDK; installation failures restore backups.
STAGING_DIR="$(mktemp -d "${SCRIPT_DIR}/.brainco-sdk.XXXXXX")"
INSTALLING=0
TARGETS=(include/stark-sdk.h lib/x86_64/libbc_stark_sdk.so lib/aarch64/libbc_stark_sdk.so lib/SHA256SUMS lib/VERSION)
cleanup() {
    local result=$? target restore_failed=0
    trap - EXIT HUP INT TERM
    if ((result != 0 && INSTALLING)); then
        printf '%s\n' 'Restoring the previous SDK...' >&2
        for target in "${TARGETS[@]}"; do
            if [[ -f "${STAGING_DIR}/backup/${target}" ]]; then
                mv -f -- "${STAGING_DIR}/backup/${target}" "${SCRIPT_DIR}/${target}" || restore_failed=1
            else
                rm -f -- "${SCRIPT_DIR}/${target}" || restore_failed=1
            fi
        done
        if ((restore_failed)); then
            printf 'Restore incomplete; backups retained in %s/backup\n' "$STAGING_DIR" >&2
            exit "$result"
        fi
    fi
    rm -rf -- "$STAGING_DIR"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

BASE_URL="https://app.brainco.cn/universal/bc-stark-sdk/libs/${SDK_VERSION}"
for arch in x86_64 aarch64; do
    case "$arch" in
        x86_64) archive=linux.zip; machine='Advanced Micro Devices X86-64' ;;
        aarch64) archive=linux-arm64.zip; machine=AArch64 ;;
    esac
    printf 'Downloading BrainCo SDK %s for %s...\n' "$SDK_VERSION" "$arch"
    curl --fail --location --show-error --silent --proto '=https' --proto-redir '=https' \
        --retry 3 --connect-timeout 20 --max-time 300 \
        --output "${STAGING_DIR}/${archive}" "${BASE_URL}/${archive}"
    mkdir -p "${STAGING_DIR}/${arch}"
    # Extract only the two expected members, never archive-supplied paths.
    unzip -p "${STAGING_DIR}/${archive}" dist/include/stark-sdk.h > "${STAGING_DIR}/${arch}/stark-sdk.h"
    unzip -p "${STAGING_DIR}/${archive}" dist/shared/linux/libbc_stark_sdk.so > "${STAGING_DIR}/${arch}/libbc_stark_sdk.so"
    header="${STAGING_DIR}/${arch}/stark-sdk.h"
    library="${STAGING_DIR}/${arch}/libbc_stark_sdk.so"
    [[ -s "$header" && -s "$library" ]] || fail "empty header or library for $arch"
    grep -F '#define STARK_SDK_H' "$header" >/dev/null || fail "invalid SDK header for $arch"
    grep -F 'STARK_HARDWARE_TYPE_REVO2_BASIC' "$header" >/dev/null || fail "missing Revo2 support for $arch"
    readelf -h "$library" > "${STAGING_DIR}/${arch}/elf-header"
    grep -E 'Class: +ELF64' "${STAGING_DIR}/${arch}/elf-header" >/dev/null || fail "$arch library is not ELF64"
    grep -E 'Type: +DYN ' "${STAGING_DIR}/${arch}/elf-header" >/dev/null || fail "$arch library is not shared ELF"
    grep -E "Machine: +${machine}$" "${STAGING_DIR}/${arch}/elf-header" >/dev/null || fail "wrong architecture for $arch"
    readelf --wide --dyn-syms "$library" | awk '$7 != "UND" {print $8}' > "${STAGING_DIR}/${arch}/symbols"
    for symbol in init_logging modbus_open modbus_close stark_get_device_info free_device_info \
        stark_read_input_registers stark_read_holding_registers stark_set_finger_unit_mode \
        stark_set_finger_positions_and_speeds stark_set_finger_positions_and_durations \
        stark_set_finger_speeds stark_get_motor_status free_motor_status_data; do
        grep -Fx "$symbol" "${STAGING_DIR}/${arch}/symbols" >/dev/null || fail "$arch library is missing $symbol"
        grep -F "${symbol}(" "$header" >/dev/null || fail "$arch header is missing $symbol"
    done
done
cmp -s "${STAGING_DIR}/x86_64/stark-sdk.h" "${STAGING_DIR}/aarch64/stark-sdk.h" || fail "architecture headers differ; refusing a mixed SDK"

mkdir -p "${STAGING_DIR}/new/include" "${STAGING_DIR}/new/lib/x86_64" "${STAGING_DIR}/new/lib/aarch64"
install -m 644 "${STAGING_DIR}/x86_64/stark-sdk.h" "${STAGING_DIR}/new/include/stark-sdk.h"
for arch in x86_64 aarch64; do
    install -m 755 "${STAGING_DIR}/${arch}/libbc_stark_sdk.so" "${STAGING_DIR}/new/lib/${arch}/libbc_stark_sdk.so"
done
(
    cd "${STAGING_DIR}/new"
    sha256sum include/stark-sdk.h lib/x86_64/libbc_stark_sdk.so lib/aarch64/libbc_stark_sdk.so > lib/SHA256SUMS
)
{
    printf '[bc-stark-sdk] Version: %s\n' "$SDK_VERSION"
    printf 'Update Time: %s\n' "$(date -u '+%Y-%m-%d %H:%M:%S UTC')"
    printf 'Linux amd64 (x86_64): %s/linux.zip\n' "$BASE_URL"
    printf 'Linux arm64 (aarch64): %s/linux-arm64.zip\n' "$BASE_URL"
} > "${STAGING_DIR}/new/lib/VERSION"

for target in "${TARGETS[@]}"; do
    mkdir -p -- "${STAGING_DIR}/backup/$(dirname -- "$target")" "${SCRIPT_DIR}/$(dirname -- "$target")"
    [[ ! -e "${SCRIPT_DIR}/${target}" || -f "${SCRIPT_DIR}/${target}" ]] || fail "not a regular file: $target"
    if [[ -e "${SCRIPT_DIR}/${target}" ]]; then
        cp -p -- "${SCRIPT_DIR}/${target}" "${STAGING_DIR}/backup/${target}"
    fi
done
INSTALLING=1
for target in "${TARGETS[@]}"; do
    mv -f -- "${STAGING_DIR}/new/${target}" "${SCRIPT_DIR}/${target}"
done
INSTALLING=0
printf 'Installed SDK %s: matching header, Linux amd64 and arm64 libraries.\n' "$SDK_VERSION"
printf '%s\n' 'Verify with: sha256sum -c lib/SHA256SUMS (from the repo root).' \
    'Reconfigure CMake and rebuild the bridge before restarting it.'
