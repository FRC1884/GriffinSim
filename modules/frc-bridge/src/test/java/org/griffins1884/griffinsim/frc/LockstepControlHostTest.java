package org.griffins1884.griffinsim.frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.PwmOutput;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.junit.jupiter.api.Test;

class LockstepControlHostTest {
  @Test
  void lockstepAppliesSensorsBeforeSteppingAndCapturesActuators() {
    RecordingTimeController timeController = new RecordingTimeController();
    List<Integer> appliedSensorSteps = new ArrayList<>();
    List<Integer> capturedActuatorSteps = new ArrayList<>();

    LockstepControlHost host =
        new LockstepControlHost(
            ControlHostConfig.defaultLockstep(),
            timeController,
            frame -> appliedSensorSteps.add(frame.header().stepId()),
            () -> { throw new AssertionError("real-time loop should not run in lockstep mode"); },
            header -> {
              capturedActuatorSteps.add(header.stepId());
              return new ActuatorFrame(header, List.of(new PwmOutput(0, 0.5)));
            });

    assertTrue(host.enqueueSensorFrame(new SensorFrame(FrameHeader.current(20_000_000L, 1), List.of(), null)));

    ActuatorFrame actuatorFrame = host.runOneTick();

    assertEquals(List.of(1), appliedSensorSteps);
    assertEquals(List.of(0.02), timeController.stepsSeconds);
    assertEquals(List.of(1), capturedActuatorSteps);
    assertEquals(1, actuatorFrame.header().stepId());
    assertTrue(host.pollActuatorFrame().isPresent());
  }

  @Test
  void dropsStaleSensorFramesAndUsesRealtimeLoopWhenConfigured() {
    RecordingTimeController timeController = new RecordingTimeController();
    List<Integer> appliedSensorSteps = new ArrayList<>();
    List<Integer> robotLoopCalls = new ArrayList<>();

    LockstepControlHost host =
        new LockstepControlHost(
            new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 8),
            timeController,
            frame -> appliedSensorSteps.add(frame.header().stepId()),
            () -> robotLoopCalls.add(1),
            header -> new ActuatorFrame(header, List.of()));

    assertTrue(host.enqueueSensorFrame(new SensorFrame(FrameHeader.current(20_000_000L, 1), List.of(), null)));
    host.runOneTick();
    assertFalse(host.enqueueSensorFrame(new SensorFrame(FrameHeader.current(10_000_000L, 0), List.of(), null)));
    host.runOneTick();

    assertEquals(List.of(1), appliedSensorSteps);
    assertEquals(2, robotLoopCalls.size());
    assertTrue(timeController.stepsSeconds.isEmpty());
    assertEquals(1, timeController.resumeCalls);
  }

  private static final class RecordingTimeController implements SimTimeController {
    private final List<Double> stepsSeconds = new ArrayList<>();
    private int pauseCalls;
    private int resumeCalls;

    @Override
    public void pause() {
      pauseCalls++;
    }

    @Override
    public void resume() {
      resumeCalls++;
    }

    @Override
    public void stepSeconds(double seconds) {
      stepsSeconds.add(seconds);
    }
  }
}
