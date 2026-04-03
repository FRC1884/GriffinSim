package org.griffins1884.griffinsim.frc;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.ByteArrayOutputStream;
import java.util.ArrayList;
import java.util.List;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.ImuInput;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.physics.DeterministicPhysicsWorld;
import org.griffins1884.griffinsim.physics.AugmentedActuatorCommandMapper;
import org.griffins1884.griffinsim.physics.BodyCollisionProfile;
import org.griffins1884.griffinsim.physics.BodyMotionProfile;
import org.griffins1884.griffinsim.physics.CompositeContactGenerator;
import org.griffins1884.griffinsim.physics.FieldContactPresets;
import org.griffins1884.griffinsim.physics.MaterialProfiles;
import org.griffins1884.griffinsim.physics.PairwiseBodyContactGenerator;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.SingleBodyActuatorCommandMapper;
import org.griffins1884.griffinsim.runtime.ReplayLogWriter;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;
import org.junit.jupiter.api.Test;

class DeterministicCoSimulationLoopTest {
  @Test
  void producesDeterministicReplayBytesAcrossRuns() throws Exception {
    byte[] first = runLoopAndCaptureReplay();
    byte[] second = runLoopAndCaptureReplay();

    assertTrue(java.util.Arrays.equals(first, second));
  }

  @Test
  void advancesRobotWorldStateForward() throws Exception {
    SimulationRun run = createRun();
    ImmutableWorldState next = run.loop.advanceOneTick();

    RigidBodyState robot = next.bodies().stream().filter(body -> body.bodyId().equals("robot")).findFirst().orElseThrow();
    assertTrue(robot.x() > 0.0);
    assertEquals(List.of("enc:0:0.0:0.0", "imu:0.0:0.0:0.0"), run.valueSink.events);
  }


  @Test
  void arenaPresetPushesRobotOutOfHubObstacle() throws Exception {
    SimulationRun run = createRun(new RigidBodyState("robot", 3.88, 4.021328, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0));

    ImmutableWorldState next = run.loop.advanceOneTick();
    RigidBodyState robot = next.bodies().get(0);

    assertTrue(robot.x() < 3.88);
  }


  @Test
  void exposesLastContactEventsForTelemetry() throws Exception {
    SimulationRun run = createRun(new RigidBodyState("robot", 3.88, 4.021328, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0));

    run.loop.advanceOneTick();

    assertTrue(run.loop.lastContactEvents().stream().anyMatch(event -> event.contactId().contains("hub") || event.contactId().contains("terrain") || event.contactId().contains("box")));
    assertEquals(run.loop.lastContactEvents().size(), run.loop.lastContactTelemetryFrame().contacts().size());
    assertTrue(run.loop.lastContactTelemetryFrame().contacts().stream().anyMatch(contact -> contact.frictionCoefficient() > 0.0));
    assertTrue(run.loop.lastContactTelemetryFrame().contacts().stream().anyMatch(contact -> contact.rollingFrictionCoefficient() >= 0.0));
    assertTrue(run.loop.lastContactTelemetryFrame().contacts().stream().anyMatch(contact -> contact.torsionalFrictionCoefficient() >= 0.0));
    assertTrue(run.loop.lastContactTelemetryFrame().contacts().stream().anyMatch(contact -> contact.slipRatio() >= 0.0));
    assertTrue(run.loop.lastContactTelemetryFrame().contacts().stream().anyMatch(contact -> contact.tangentialSpeedAfter() >= 0.0));
  }


  @Test
  void replayIncludesStructuredContactTelemetryRecords() throws Exception {
    SimulationRun run = createRun(new RigidBodyState("robot", 3.88, 4.021328, 0.5, 1, 0, 0, 0, 1.5, 0, 0, 0, 0, 0));

    run.loop.advanceOneTick();
    run.replayLogWriter.close();

    var records = org.griffins1884.griffinsim.runtime.ReplayLogReader.readAll(new java.io.ByteArrayInputStream(run.outputStream.toByteArray()));
    assertTrue(records.stream().anyMatch(record -> record.type() == org.griffins1884.griffinsim.runtime.ReplayRecordType.CONTACT_TELEMETRY));
  }


  @Test
  void robotGamepieceInteractionEmitsMultiBodyTelemetryAndMovesGamepiece() throws Exception {
    SimulationRun run =
        createRun(
            List.of(
                new RigidBodyState("robot", 0.0, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0),
                new RigidBodyState("gamepiece", 0.3, 0.0, 0.0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0)),
            new AugmentedActuatorCommandMapper(
                new SingleBodyActuatorCommandMapper("robot", 50.0, 10.0, 100.0, 0.0),
                List.of(new BodyMotionProfile("gamepiece", 3.0, 1.0))),
            new CompositeContactGenerator(
                List.of(
                    FieldContactPresets.rebuilt2026Arena(),
                    new PairwiseBodyContactGenerator(
                        List.of(
                            new BodyCollisionProfile("robot", 0.4, 0.4, 0.4, MaterialProfiles.DEFAULT_BODY),
                            new BodyCollisionProfile("gamepiece", 0.2, 0.2, 0.2, MaterialProfiles.HDPE))))));

    ImmutableWorldState next = run.loop.advanceOneTick();
    RigidBodyState gamepiece = next.bodies().stream().filter(body -> body.bodyId().equals("gamepiece")).findFirst().orElseThrow();

    assertTrue(gamepiece.x() > 0.3 || gamepiece.y() != 0.0 || gamepiece.z() != 0.0);
    assertTrue(run.loop.lastContactTelemetryFrame().contacts().stream().anyMatch(contact -> contact.contactId().contains("vs:gamepiece") || contact.contactId().contains("vs:robot")));
  }

  private static byte[] runLoopAndCaptureReplay() throws Exception {
    SimulationRun run = createRun();
    run.loop.advanceOneTick();
    run.loop.advanceOneTick();
    run.replayLogWriter.close();
    return run.outputStream.toByteArray();
  }

  private static SimulationRun createRun() {
    return createRun(new RigidBodyState("robot", 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0));
  }

  private static SimulationRun createRun(RigidBodyState initialRobot) {
    return createRun(
        List.of(initialRobot),
        new SingleBodyActuatorCommandMapper("robot", 50.0, 10.0, 100.0, 0.0),
        FieldContactPresets.rebuilt2026Arena());
  }

  private static SimulationRun createRun(
      List<RigidBodyState> initialBodies,
      org.griffins1884.griffinsim.physics.ActuatorCommandMapper mapper,
      org.griffins1884.griffinsim.physics.ContactGenerator contactGenerator) {
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
    DeterministicSensorEmulator sensorEmulator =
        new DeterministicSensorEmulator(SensorEmissionConfig.immediate("robot", 0), 1884L);
    ImmutableWorldState initialWorld =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            initialBodies);
    ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
    ReplayLogWriter replayLogWriter = new ReplayLogWriter(outputStream);
    DeterministicCoSimulationLoop loop =
        new DeterministicCoSimulationLoop(
            config,
            initialWorld,
            host,
            new DeterministicPhysicsWorld(),
            sensorEmulator,
            mapper,
            contactGenerator,
            replayLogWriter);
    return new SimulationRun(loop, valueSink, outputStream, replayLogWriter);
  }

  private record SimulationRun(
      DeterministicCoSimulationLoop loop,
      RecordingValueSink valueSink,
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
