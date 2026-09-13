#!/usr/bin/env bash
# Offline updater regression tests. Only temporary files are modified.
set -euo pipefail
REPO_DIR="$(cd -- "$1" && pwd)"
CMAKE_BIN="$2"
for dependency in curl unzip readelf sha256sum flock; do
    if ! command -v "$dependency" >/dev/null; then
        printf 'SKIP: updater tests require %s\n' "$dependency"
        exit 77
    fi
done
TEST_DIR="$(mktemp -d "${TMPDIR:-/tmp}/brainco-sdk-test.XXXXXX")"
trap 'rm -rf -- "$TEST_DIR"' EXIT
export SDK_TEST_DIR="$TEST_DIR"
export SDK_TEST_ARCH=x86_64 SDK_TEST_OS=Linux SDK_TEST_FAILURE=none

# Construct real ZIP fixtures from the bundled header and both ELF libraries.
mkdir -p "$TEST_DIR/fixture/dist/include" "$TEST_DIR/fixture/dist/shared/linux" "$TEST_DIR/install repo"
cp "$REPO_DIR/download-lib.sh" "$TEST_DIR/install repo/download-lib.sh"
cp "$REPO_DIR/include/stark-sdk.h" "$TEST_DIR/fixture/dist/include/stark-sdk.h"
for arch in x86_64 aarch64; do
    cp "$REPO_DIR/lib/$arch/libbc_stark_sdk.so" "$TEST_DIR/fixture/dist/shared/linux/libbc_stark_sdk.so"
    (cd "$TEST_DIR/fixture"; "$CMAKE_BIN" -E tar cf "$TEST_DIR/$arch.zip" --format=zip dist)
done

# Exported Bash functions simulate the host and download failures without network.
uname() {
    case "$1" in
        -s) printf '%s\n' "$SDK_TEST_OS" ;;
        -m) printf '%s\n' "$SDK_TEST_ARCH" ;;
        *) command uname "$@" ;;
    esac
}
curl() {
    local output='' url='' arch
    while (($#)); do
        case "$1" in
            --output) output="$2"; shift 2 ;;
            *) url="$1"; shift ;;
        esac
    done
    case "$url" in
        https://app.brainco.cn/universal/bc-stark-sdk/libs/v*/linux.zip) arch=x86_64 ;;
        https://app.brainco.cn/universal/bc-stark-sdk/libs/v*/linux-arm64.zip) arch=aarch64 ;;
        *) printf 'Unexpected download URL: %s\n' "$url" >&2; return 1 ;;
    esac
    if [[ "$arch" == aarch64 ]]; then
        case "$SDK_TEST_FAILURE" in
            download) return 22 ;;
            corrupt_zip) printf 'not a ZIP' > "$output"; return ;;
            architecture) arch=x86_64 ;;
        esac
    fi
    cp "$SDK_TEST_DIR/$arch.zip" "$output"
}
unzip() {
    if [[ "$2" == */linux-arm64.zip ]]; then
        case "$SDK_TEST_FAILURE:$3" in
            missing_header:dist/include/stark-sdk.h) return 11 ;;
            header_mismatch:dist/include/stark-sdk.h)
                command unzip "$@"
                printf '\n// mismatched archive header\n'
                return ;;
            invalid_elf:dist/shared/linux/libbc_stark_sdk.so)
                printf 'not an ELF library'
                return ;;
        esac
    fi
    command unzip "$@"
}
readelf() {
    if [[ "$SDK_TEST_FAILURE" == missing_symbol && "$*" == *--dyn-syms*aarch64/libbc_stark_sdk.so ]]; then
        command readelf "$@" | awk '!/init_logging/'
    else
        command readelf "$@"
    fi
}
mv() {
    # Fail after the header has been installed, but allow all backup restores.
    if [[ "$SDK_TEST_FAILURE" == install && "$*" == */new/lib/x86_64/libbc_stark_sdk.so* ]]; then
        return 1
    fi
    command mv "$@"
}
export -f uname curl unzip readelf mv

cd "$TEST_DIR/install repo"
sdk_files=(include/stark-sdk.h lib/x86_64/libbc_stark_sdk.so lib/aarch64/libbc_stark_sdk.so lib/SHA256SUMS lib/VERSION)
snapshot() { sha256sum "${sdk_files[@]}"; }
assert_clean() {
    local stages
    shopt -s nullglob
    stages=(.brainco-sdk.*)
    ((${#stages[@]} == 0)) || { printf 'Staging directory leaked\n' >&2; exit 1; }
}
expect_failure() {
    local before
    before="$(snapshot)"
    if bash ./download-lib.sh "$@" > "$TEST_DIR/run.log" 2>&1; then
        printf 'Expected failure for %s (%s, %s)\n' "$SDK_TEST_FAILURE" "$SDK_TEST_OS" "$SDK_TEST_ARCH" >&2
        exit 1
    fi
    [[ "$(snapshot)" == "$before" ]] || { printf 'Failed update modified SDK files\n' >&2; exit 1; }
    assert_clean
}

for SDK_TEST_ARCH in x86_64 amd64 aarch64 arm64; do
    bash ./download-lib.sh --version v2.0.5 > "$TEST_DIR/run.log" 2>&1
    sha256sum -c lib/SHA256SUMS
    cmp include/stark-sdk.h "$REPO_DIR/include/stark-sdk.h"
    for arch in x86_64 aarch64; do
        cmp "lib/$arch/libbc_stark_sdk.so" "$REPO_DIR/lib/$arch/libbc_stark_sdk.so"
    done
    assert_clean
    printf 'PASS: Linux %s installs both architectures\n' "$SDK_TEST_ARCH"
done

bash ./download-lib.sh --version v9.8.7 > "$TEST_DIR/run.log" 2>&1
grep -Fx '[bc-stark-sdk] Version: v9.8.7' lib/VERSION
printf 'PASS: explicit version override recorded\n'

# Same-version installs must repair missing/corrupt files, not skip by VERSION.
bash ./download-lib.sh > "$TEST_DIR/run.log" 2>&1
rm lib/aarch64/libbc_stark_sdk.so
printf 'corrupt header\n' > include/stark-sdk.h
bash ./download-lib.sh > "$TEST_DIR/run.log" 2>&1
sha256sum -c lib/SHA256SUMS
printf 'PASS: same-version repair\n'

# Use distinct old contents so rollback cannot pass merely by leaving new files.
printf 'previous header\n' > include/stark-sdk.h
for SDK_TEST_FAILURE in download corrupt_zip missing_header header_mismatch architecture invalid_elf missing_symbol install; do
    expect_failure
    printf 'PASS: %s failure preserves previous SDK\n' "$SDK_TEST_FAILURE"
done
SDK_TEST_FAILURE=none
SDK_TEST_OS=Darwin
expect_failure
SDK_TEST_OS=Linux
SDK_TEST_ARCH=armv7l
expect_failure
SDK_TEST_ARCH=x86_64
expect_failure --version
expect_failure --version ../../invalid
expect_failure --unknown
printf 'PASS: unsupported hosts and invalid arguments rejected\n'

(
    exec 9< ./download-lib.sh
    flock -n 9
    expect_failure
)
printf 'PASS: concurrent updater rejected\n'

# Rollback also removes newly installed files when there was no previous SDK.
mkdir "$TEST_DIR/fresh"
cp ./download-lib.sh "$TEST_DIR/fresh/download-lib.sh"
cd "$TEST_DIR/fresh"
SDK_TEST_FAILURE=install
if bash ./download-lib.sh > "$TEST_DIR/run.log" 2>&1; then
    printf 'Expected fresh-install failure\n' >&2
    exit 1
fi
for target in "${sdk_files[@]}"; do
    [[ ! -e "$target" ]] || { printf 'Partial install left %s\n' "$target" >&2; exit 1; }
done
assert_clean
printf 'PASS: failed fresh install leaves no SDK files\n'
