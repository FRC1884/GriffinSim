package org.griffins1884.griffinsim.frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.ImuInput;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.physics.BodyCollisionProfile;
import org.griffins1884.griffinsim.physics.CompositeContactGenerator;
import org.griffins1884.griffinsim.physics.DeterministicPhysicsWorld;
import org.griffins1884.griffinsim.physics.FieldContactPresets;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.MaterialProfiles;
import org.griffins1884.griffinsim.physics.PairwiseBodyContactGenerator;
import org.griffins1884.griffinsim.physics.SingleBodyActuatorCommandMapper;
import org.griffins1884.griffinsim.runtime.ReplayLogReader;
import org.griffins1884.griffinsim.runtime.ReplayLogWriter;
import org.griffins1884.griffinsim.runtime.ReplayRecordType;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;
import org.junit.jupiter.api.Test;

class MultiRobotCoSimulationLoopTest {
  @Test
  void sharedWorldTwoRobotLoopIsDeterministic() throws Exception {
    byte[] first = runReplay();
    byte[] second = runReplay();
    assertTrue(java.util.Arrays.equals(first, second));
  }

  @Test
  void multiRobotLoopEmitsRobotRobotContactTelemetry() throws Exception {
    SimulationRun run = createRun();
    ImmutableWorldState next = run.loop.advanceOneTick();

    assertEquals(2, next.bodies().size());
    assertTrue(run.loop.lastContactTelemetryFrame().contacts().stream().anyMatch(contact -> contact.contactId().contains("robot-a:vs:robot-b") || contact.contactId().contains("robot-b:vs:robot-a")));
  }

  @Test
  void replayContainsStructuredTelemetryForMultiRobotRun() throws Exception {
    SimulationRun run = createRun();
    run.loop.advanceOneTick();
    run.replayLogWriter.close();

    var records = ReplayLogReader.readAll(new ByteArrayInputStream(run.outputStream.toByteArray()));
    assertTrue(records.stream().anyMatch(record -> record.type() == ReplayRecordType.CONTACT_TELEMETRY));
  }

  private static byte[] runReplay() throws Exception {
    SimulationRun run = createRun();
    run.loop.advanceOneTick();
    run.loop.advanceOneTick();
    run.replayLogWriter.close();
    return run.outputStream.toByteArray();
  }

  private static SimulationRun createRun() {
    ControlHostConfig config = new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 16);
    ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
    ReplayLogWriter replayLogWriter = new ReplayLogWriter(outputStream);

    MultiRobotEndpoint robotA = endpoint("robot-a", 0, 0.5, 0.0);
    MultiRobotEndpoint robotB = endpoint("robot-b", 1, -0.5, 0.6);

    ImmutableWorldState initialWorld =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(
                new RigidBodyState("robot-a", 0.0, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0),
                new RigidBodyState("robot-b", 0.6, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0)));

    MultiRobotCoSimulationLoop loop =
        new MultiRobotCoSimulationLoop(
            config,
            initialWorld,
            new DeterministicPhysicsWorld(),
            List.of(robotB, robotA),
            new CompositeContactGenerator(
                List.of(
                    FieldContactPresets.rebuilt2026Arena(),
                    new PairwiseBodyContactGenerator(
                        List.of(
                            new BodyCollisionProfile("robot-a", 0.4, 0.4, 0.4, MaterialProfiles.DEFAULT_BODY),
                            new BodyCollisionProfile("robot-b", 0.4, 0.4, 0.4, MaterialProfiles.DEFAULT_BODY))))),
            replayLogWriter);
    return new SimulationRun(loop, outputStream, replayLogWriter);
  }

  private static MultiRobotEndpoint endpoint(String bodyId, int channel, double pwmValue, double startX) {
    RecordingValueSink sink = new RecordingValueSink();
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(16, sink);
    LockstepControlHost host =
        new LockstepControlHost(
            new ControlHostConfig(ClockMode.REAL_TIME, 5_000_000L, 20_000_000L, 16),
            new RecordingTimeController(),
            bridge,
            () -> bridge.onPwmChanged(channel, pwmValue),
            bridge);
    return new MultiRobotEndpoint(
        bodyId,
        host,
        new DeterministicSensorEmulator(SensorEmissionConfig.immediate(bodyId, channel), 1884L + channel),
        new SingleBodyActuatorCommandMapper(bodyId, 50.0, 10.0, 100.0, 0.0));
  }

  private record SimulationRun(
      MultiRobotCoSimulationLoop loop,
      ByteArrayOutputStream outputStream,
      ReplayLogWriter replayLogWriter) {}

  private static final class RecordingTimeController implements SimTimeController {
    @Override
    public void pause() {}

    @Override
    public void resume() {}

    @Override
    public void stepSeconds(double seconds) {}
  }

  private static final class RecordingValueSink implements HalSimValueSink {
    private final List<String> events = new ArrayList<>();

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
