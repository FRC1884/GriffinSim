#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

./gradlew nativeExtensionBundle

echo "native extension scaffold bundle ready at $ROOT_DIR/build/distributions"
