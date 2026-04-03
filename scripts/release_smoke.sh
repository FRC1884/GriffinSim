#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

OUT_DIR="$ROOT_DIR/build/release-smoke"
mkdir -p "$OUT_DIR"

SCENARIO_A="$ROOT_DIR/scenarios/examples/two-robot-head-on.properties"
SCENARIO_B="$ROOT_DIR/scenarios/examples/velocity-pose-controller.properties"
REPLAY_A="$OUT_DIR/two-robot-head-on.bin"
REPLAY_B="$OUT_DIR/velocity-pose-controller.bin"
REPLAY_B_COPY="$OUT_DIR/velocity-pose-controller-copy.bin"

./gradlew test
./gradlew :apps:scenario-runner:run --args="$SCENARIO_A 2 $REPLAY_A"
./gradlew :apps:scenario-runner:run --args="$SCENARIO_B 4 $REPLAY_B"
./gradlew :apps:scenario-runner:run --args="$SCENARIO_B 4 $REPLAY_B_COPY"
./gradlew :apps:replay-diff:run --args="$REPLAY_B $REPLAY_B_COPY"

echo "release smoke complete"
echo "artifacts in $OUT_DIR"
