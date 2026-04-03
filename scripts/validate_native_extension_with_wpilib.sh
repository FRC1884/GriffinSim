#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

./scripts/build_native_extension_with_wpilib.sh
./gradlew :modules:frc-bridge:test --tests '*PanamaWpilibNativeExtension*'

echo "WPILib-linked native extension validation complete"
