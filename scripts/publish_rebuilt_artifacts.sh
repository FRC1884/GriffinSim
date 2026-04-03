#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

TARGET_DIR="${1:-$ROOT_DIR/build/publish-staging}"
case "$TARGET_DIR" in
  "$ROOT_DIR"/*) ;;
  *)
    echo "target directory must stay inside repo: $TARGET_DIR" >&2
    exit 2
    ;;
esac

./gradlew releaseArtifacts
rm -rf "$TARGET_DIR"
mkdir -p "$TARGET_DIR"
cp -R "$ROOT_DIR/build/release-artifacts/." "$TARGET_DIR/"

echo "published rebuilt artifacts to $TARGET_DIR"
