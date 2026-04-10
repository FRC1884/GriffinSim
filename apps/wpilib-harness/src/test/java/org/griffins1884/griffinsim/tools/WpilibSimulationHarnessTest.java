package org.griffins1884.griffinsim.tools;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class WpilibSimulationHarnessTest {
  private static final int DISABLED_TICK = 0;
  private static final int FORWARD_START = 1;
  private static final int FORWARD_END = 50;
  private static final int ROTATE_START = 51;
  private static final int ROTATE_END = 100;
  private static final int COMBINED_START = 101;
  private static final int COMBINED_END = 150;
  private static final int STOP_START = 151;
  private static final int STOP_END = 200;
  private static final double COMMAND_X = 0.6;
  private static final double COMMAND_Y = 0.3;

  @Test
  void executesRealTimedRobotLifecycleEndToEnd() {
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> result = runFixture();

    assertTrue(result.robot().lifecycleEvents().contains("robotInit"));
    assertTrue(result.robot().lifecycleEvents().contains("driverStationConnected"));
    assertTrue(result.robot().lifecycleEvents().contains("simulationInit"));
    assertTrue(result.robot().lifecycleEvents().contains("disabledInit"));
    assertTrue(result.robot().lifecycleEvents().contains("teleopInit"));
    assertEquals(200, result.robot().teleopPeriodicCalls());
  }

  @Test
  void forwardMotionProducesForwardDisplacementWithoutRotationOrDrift() {
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> result = runFixture();
    SegmentStats forward = segment(result.trace(), FORWARD_START, FORWARD_END);

    assertTrue(forward.deltaX() > 0.35, "expected forward x displacement, got " + forward.deltaX());
    assertTrue(Math.abs(forward.deltaY()) < 0.08, "unexpected sideways drift: " + forward.deltaY());
    assertTrue(
        Math.abs(forward.deltaHeading()) < 0.15,
        "unexpected heading drift during forward motion: " + forward.deltaHeading());
  }

  @Test
  void rotationCommandProducesAngularVelocityWithoutTranslation() {
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> result = runFixture();
    SegmentStats rotation = segment(result.trace(), ROTATE_START, ROTATE_END);

    assertTrue(Math.abs(rotation.deltaHeading()) > 0.9, "insufficient rotation: " + rotation.deltaHeading());
    assertTrue(Math.abs(rotation.deltaX()) < 0.12, "rotation phase translated in x: " + rotation.deltaX());
    assertTrue(Math.abs(rotation.deltaY()) < 0.12, "rotation phase translated in y: " + rotation.deltaY());
    assertTrue(rotation.maxAbsAngularVelocity() > 0.8, "rotation phase angular velocity too low");
  }

  @Test
  void combinedMotionTracksRobotHeadingFrame() {
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> result = runFixture();
    SegmentStats combined = segment(result.trace(), COMBINED_START, COMBINED_END);

    double heading = combined.start().headingRadians();
    double expectedX = (COMMAND_X * Math.cos(heading)) - (COMMAND_Y * Math.sin(heading));
    double expectedY = (COMMAND_X * Math.sin(heading)) + (COMMAND_Y * Math.cos(heading));
    double expectedMagnitude = Math.hypot(expectedX, expectedY);
    double actualMagnitude = Math.hypot(combined.deltaX(), combined.deltaY());
    double alignment =
        ((combined.deltaX() * expectedX) + (combined.deltaY() * expectedY))
            / (expectedMagnitude * actualMagnitude);

    assertTrue(actualMagnitude > 0.15, "combined phase did not translate enough");
    assertTrue(alignment > 0.8, "combined phase misaligned with robot frame: " + alignment);
  }

  @Test
  void zeroInputSettlesToNearZeroVelocityWithoutDrift() {
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> result = runFixture();
    SegmentStats stop = segment(result.trace(), STOP_START, STOP_END);
    WpilibSimulationHarness.TraceStep finalStep = stop.end();

    assertTrue(
        Math.abs(finalStep.linearSpeedMetersPerSecond()) < 0.08,
        "linear velocity remained too high: " + finalStep.linearSpeedMetersPerSecond());
    assertTrue(
        Math.abs(finalStep.angularVelocityRadiansPerSecond()) < 0.08,
        "angular velocity remained too high: " + finalStep.angularVelocityRadiansPerSecond());
    assertTrue(Math.hypot(stop.deltaX(), stop.deltaY()) < 0.08, "stop phase drifted too far");
  }

  @Test
  void motionIsSmoothAndEnergyRemainsBounded() {
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> result = runFixture();
    List<WpilibSimulationHarness.TraceStep> trace = result.trace();

    double maxPositionJump = 0.0;
    double maxHeadingJump = 0.0;
    double maxEnergy = 0.0;
    for (int index = 0; index < trace.size(); index++) {
      WpilibSimulationHarness.TraceStep step = trace.get(index);
      assertTrue(Double.isFinite(step.x()));
      assertTrue(Double.isFinite(step.y()));
      assertTrue(Double.isFinite(step.headingRadians()));
      assertTrue(Double.isFinite(step.linearSpeedMetersPerSecond()));
      assertTrue(Double.isFinite(step.angularVelocityRadiansPerSecond()));
      double energy =
          (0.5 * 50.0 * step.linearSpeedMetersPerSecond() * step.linearSpeedMetersPerSecond())
              + (0.5 * 10.0 * step.angularVelocityRadiansPerSecond() * step.angularVelocityRadiansPerSecond());
      maxEnergy = Math.max(maxEnergy, energy);
      if (index == 0) {
        continue;
      }
      WpilibSimulationHarness.TraceStep previous = trace.get(index - 1);
      maxPositionJump =
          Math.max(maxPositionJump, Math.hypot(step.x() - previous.x(), step.y() - previous.y()));
      maxHeadingJump = Math.max(maxHeadingJump, Math.abs(step.headingRadians() - previous.headingRadians()));
    }

    assertTrue(maxPositionJump < 0.08, "position discontinuity too large: " + maxPositionJump);
    assertTrue(maxHeadingJump < 0.16, "heading discontinuity too large: " + maxHeadingJump);
    assertTrue(maxEnergy < 120.0, "energy runaway detected: " + maxEnergy);
  }

  @Test
  void producesDeterministicReplayAcrossRuns() {
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> first = runFixture();
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> second = runFixture();

    assertArrayEquals(first.replayBytes(), second.replayBytes());
  }

  private static WpilibSimulationHarness.RunResult<FixtureTimedRobot> runFixture() {
    return WpilibSimulationHarness.run(FixtureTimedRobot::new, WpilibHarnessMain.defaultSpec());
  }

  private static SegmentStats segment(
      List<WpilibSimulationHarness.TraceStep> trace, int startTickInclusive, int endTickInclusive) {
    WpilibSimulationHarness.TraceStep start =
        trace.stream()
            .filter(step -> step.controlTickIndex() == startTickInclusive)
            .findFirst()
            .orElseThrow();
    WpilibSimulationHarness.TraceStep end =
        trace.stream()
            .filter(step -> step.controlTickIndex() == endTickInclusive)
            .findFirst()
            .orElseThrow();
    double maxAbsAngularVelocity =
        trace.stream()
            .filter(
                step ->
                    step.controlTickIndex() >= startTickInclusive
                        && step.controlTickIndex() <= endTickInclusive)
            .mapToDouble(step -> Math.abs(step.angularVelocityRadiansPerSecond()))
            .max()
            .orElse(0.0);
    return new SegmentStats(start, end, maxAbsAngularVelocity);
  }

  private record SegmentStats(
      WpilibSimulationHarness.TraceStep start,
      WpilibSimulationHarness.TraceStep end,
      double maxAbsAngularVelocity) {
    double deltaX() {
      return end.x() - start.x();
    }

    double deltaY() {
      return end.y() - start.y();
    }

    double deltaHeading() {
      return end.headingRadians() - start.headingRadians();
    }
  }
}
