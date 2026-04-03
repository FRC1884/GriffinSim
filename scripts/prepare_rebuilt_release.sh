#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

./gradlew clean test milestoneBundle
./scripts/release_smoke.sh

echo "rebuilt release preparation complete"
echo "bundle: $ROOT_DIR/build/distributions/GriffinSim-rebuilt-platform-${VERSION_NAME:-unknown}.zip"
