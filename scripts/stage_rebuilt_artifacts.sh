#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

./gradlew releaseArtifacts

echo "staged rebuilt artifacts at $ROOT_DIR/build/release-artifacts"
