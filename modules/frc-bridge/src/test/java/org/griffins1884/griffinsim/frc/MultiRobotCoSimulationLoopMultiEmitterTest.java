package org.griffins1884.griffinsim.frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.ImuInput;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.physics.DeterministicPhysicsWorld;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.SingleBodyActuatorCommandMapper;
import org.griffins1884.griffinsim.runtime.ReplayLogWriter;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;
import org.junit.jupiter.api.Test;

class MultiRobotCoSimulationLoopMultiEmitterTest {
  @Test
  void endpointCanEmitMultipleDeterministicSensorFramesPerTick() throws Exception {
    RecordingValueSink valueSink = new RecordingValueSink();
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(16, valueSink);
    ControlHostConfig config = new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 16);
    LockstepControlHost host =
        new LockstepControlHost(
            config,
            new RecordingTimeController(),
            bridge,
            () -> bridge.onPwmChanged(0, 0.5),
            bridge);

    MultiRobotEndpoint endpoint =
        new MultiRobotEndpoint(
            "robot",
            host,
            List.of(
                new DeterministicSensorEmulator(SensorEmissionConfig.immediate("robot", 0), 1884L),
                new DeterministicSensorEmulator(SensorEmissionConfig.immediate("robot", 1), 1885L)),
            new SingleBodyActuatorCommandMapper("robot", 50.0, 10.0, 100.0, 0.0));

    ByteArrayOutputStream output = new ByteArrayOutputStream();
    ReplayLogWriter replayLogWriter = new ReplayLogWriter(output);
    MultiRobotCoSimulationLoop loop =
        new MultiRobotCoSimulationLoop(
            config,
            new ImmutableWorldState(
                FrameHeader.current(0L, 0),
                List.of(new RigidBodyState("robot", 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0))),
            new DeterministicPhysicsWorld(),
            List.of(endpoint),
            worldState -> List.of(),
            replayLogWriter,
            List.of());

    loop.advanceOneTick();

    assertTrue(valueSink.events.stream().anyMatch(event -> event.startsWith("enc:0:")));
    assertTrue(valueSink.events.stream().anyMatch(event -> event.startsWith("enc:1:")));
    assertTrue(loop.currentWorldState().header().stepId() > 0);
  }

  private static final class RecordingTimeController implements SimTimeController {
    @Override
    public void pause() {}

    @Override
    public void resume() {}

    @Override
    public void stepSeconds(double seconds) {}
  }

  private static final class RecordingValueSink implements HalSimValueSink {
    private final java.util.ArrayList<String> events = new java.util.ArrayList<>();

    @Override
    public void setEncoder(int channel, double position, double velocity) {
      events.add("enc:" + channel + ":" + position + ":" + velocity);
    }

    @Override
    public void setImu(ImuInput imuInput) {
      events.add("imu:" + imuInput.yaw() + ":" + imuInput.pitch() + ":" + imuInput.roll());
    }
  }
}
