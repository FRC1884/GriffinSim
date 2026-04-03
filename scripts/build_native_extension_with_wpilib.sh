#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

WPILIB_VERSION="${WPILIB_VERSION:-2026.2.1}"
WPILIB_MAVEN_ROOT="${WPILIB_MAVEN_ROOT:-$HOME/wpilib/2026/maven}"
SDK_ROOT="$ROOT_DIR/build/wpilib-hal-sdk/$WPILIB_VERSION"

detect_classifier() {
  case "$(uname -s):$(uname -m)" in
    Darwin:*) echo "osxuniversal" ;;
    Linux:x86_64|Linux:amd64) echo "linuxx86-64" ;;
    Linux:aarch64|Linux:arm64) echo "linuxarm64" ;;
    MINGW*:x86_64|MSYS*:x86_64|CYGWIN*:x86_64) echo "windowsx86-64" ;;
    *)
      echo "Unsupported host platform for WPILib HAL auto-discovery: $(uname -s):$(uname -m)" >&2
      exit 1
      ;;
  esac
}

detect_library_name() {
  case "$(uname -s)" in
    Darwin) echo "libwpiHal.dylib" ;;
    Linux) echo "libwpiHal.so" ;;
    MINGW*|MSYS*|CYGWIN*) echo "wpiHal.dll" ;;
    *)
      echo "Unsupported host platform for WPILib HAL library naming: $(uname -s)" >&2
      exit 1
      ;;
  esac
}

auto_discover_wpilib_hal() {
  if [[ -n "${GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR:-}" && -n "${GRIFFINSIM_WPILIB_HAL_LIBRARY:-}" ]]; then
    return
  fi

  local classifier
  classifier="$(detect_classifier)"
  local library_name
  library_name="$(detect_library_name)"
  local headers_zip="$WPILIB_MAVEN_ROOT/edu/wpi/first/hal/hal-cpp/$WPILIB_VERSION/hal-cpp-$WPILIB_VERSION-headers.zip"
  local native_zip="$WPILIB_MAVEN_ROOT/edu/wpi/first/hal/hal-cpp/$WPILIB_VERSION/hal-cpp-$WPILIB_VERSION-$classifier.zip"
  local wpiutil_native_zip="$WPILIB_MAVEN_ROOT/edu/wpi/first/wpiutil/wpiutil-cpp/$WPILIB_VERSION/wpiutil-cpp-$WPILIB_VERSION-$classifier.zip"
  local include_dir="$SDK_ROOT/include"
  local native_dir="$SDK_ROOT/native"

  if [[ ! -f "$headers_zip" ]]; then
    echo "Missing WPILib HAL headers zip: $headers_zip" >&2
    exit 1
  fi
  if [[ ! -f "$native_zip" ]]; then
    echo "Missing WPILib HAL native zip: $native_zip" >&2
    exit 1
  fi
  if [[ ! -f "$wpiutil_native_zip" ]]; then
    echo "Missing WPILib wpiutil native zip: $wpiutil_native_zip" >&2
    exit 1
  fi

  mkdir -p "$include_dir" "$native_dir"
  unzip -oq "$headers_zip" -d "$include_dir"
  unzip -oq "$native_zip" -d "$native_dir"
  unzip -oq "$wpiutil_native_zip" -d "$native_dir"

  export GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR="$include_dir"
  export GRIFFINSIM_WPILIB_HAL_LIBRARY
  GRIFFINSIM_WPILIB_HAL_LIBRARY="$(find "$native_dir" -name "$library_name" | head -n 1)"

  if [[ -z "$GRIFFINSIM_WPILIB_HAL_LIBRARY" ]]; then
    echo "Could not locate $library_name after extracting $native_zip" >&2
    exit 1
  fi
}

auto_discover_wpilib_hal

BUILD_DIR="$ROOT_DIR/build/native-halsim-extension-wpilib"
cmake \
  -S native/halsim-extension \
  -B "$BUILD_DIR" \
  -DGRIFFINSIM_ENABLE_WPILIB_HALSIM=ON \
  -DGRIFFINSIM_WPILIB_HAL_INCLUDE_DIR="$GRIFFINSIM_WPILIB_HAL_INCLUDE_DIR" \
  -DGRIFFINSIM_WPILIB_HAL_LIBRARY="$GRIFFINSIM_WPILIB_HAL_LIBRARY"
cmake --build "$BUILD_DIR"

echo "built WPILib-enabled native extension in $BUILD_DIR"
