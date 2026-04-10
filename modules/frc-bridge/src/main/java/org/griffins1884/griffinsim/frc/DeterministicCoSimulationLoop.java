package org.griffins1884.griffinsim.frc;

import java.util.List;
import java.util.Objects;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.ContactTelemetry;
import org.griffins1884.griffinsim.contracts.ContactTelemetryFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.SensorFrame;
import org.griffins1884.griffinsim.contracts.WorldSnapshot;
import org.griffins1884.griffinsim.physics.ActuatorCommandMapper;
import org.griffins1884.griffinsim.physics.BodyCommand;
import org.griffins1884.griffinsim.physics.ContactGenerator;
import org.griffins1884.griffinsim.physics.DeterministicPhysicsWorld;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.PhysicsStepRequest;
import org.griffins1884.griffinsim.physics.WorldSnapshots;
import org.griffins1884.griffinsim.rendering.WorldSnapshotSubscriber;
import org.griffins1884.griffinsim.runtime.ReplayLogWriter;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;

public final class DeterministicCoSimulationLoop {
  private final ControlHostConfig config;
  private final LockstepControlHost controlHost;
  private final DeterministicPhysicsWorld physicsWorld;
  private final DeterministicSensorEmulator sensorEmulator;
  private final ActuatorCommandMapper actuatorCommandMapper;
  private final ContactGenerator contactGenerator;
  private final ReplayLogWriter replayLogWriter;
  private final List<WorldSnapshotSubscriber> snapshotSubscribers;
  private ImmutableWorldState currentWorldState;
  private List<org.griffins1884.griffinsim.physics.ContactEvent> lastContactEvents;
  private ContactTelemetryFrame lastContactTelemetryFrame;
  private ActuatorFrame lastActuatorFrame;
  private List<BodyCommand> lastAppliedCommands;

  public DeterministicCoSimulationLoop(
      ControlHostConfig config,
      ImmutableWorldState initialWorldState,
      LockstepControlHost controlHost,
      DeterministicPhysicsWorld physicsWorld,
      DeterministicSensorEmulator sensorEmulator,
      ActuatorCommandMapper actuatorCommandMapper,
      ContactGenerator contactGenerator,
      ReplayLogWriter replayLogWriter) {
    this(
        config,
        initialWorldState,
        controlHost,
        physicsWorld,
        sensorEmulator,
        actuatorCommandMapper,
        contactGenerator,
        replayLogWriter,
        List.of());
  }

  public DeterministicCoSimulationLoop(
      ControlHostConfig config,
      ImmutableWorldState initialWorldState,
      LockstepControlHost controlHost,
      DeterministicPhysicsWorld physicsWorld,
      DeterministicSensorEmulator sensorEmulator,
      ActuatorCommandMapper actuatorCommandMapper,
      ContactGenerator contactGenerator,
      ReplayLogWriter replayLogWriter,
      List<WorldSnapshotSubscriber> snapshotSubscribers) {
    this.config = Objects.requireNonNull(config);
    this.currentWorldState = Objects.requireNonNull(initialWorldState);
    this.controlHost = Objects.requireNonNull(controlHost);
    this.physicsWorld = Objects.requireNonNull(physicsWorld);
    this.sensorEmulator = Objects.requireNonNull(sensorEmulator);
    this.actuatorCommandMapper = Objects.requireNonNull(actuatorCommandMapper);
    this.contactGenerator = Objects.requireNonNull(contactGenerator);
    this.replayLogWriter = replayLogWriter;
    this.snapshotSubscribers =
        List.copyOf(snapshotSubscribers == null ? List.of() : snapshotSubscribers);
    this.lastContactEvents = List.of();
    this.lastContactTelemetryFrame = new ContactTelemetryFrame(initialWorldState.header(), List.of());
    this.lastActuatorFrame = new ActuatorFrame(initialWorldState.header(), List.of());
    this.lastAppliedCommands = List.of();
  }

  public ImmutableWorldState currentWorldState() {
    return currentWorldState;
  }

  public List<org.griffins1884.griffinsim.physics.ContactEvent> lastContactEvents() {
    return lastContactEvents;
  }

  public ContactTelemetryFrame lastContactTelemetryFrame() {
    return lastContactTelemetryFrame;
  }

  public ActuatorFrame lastActuatorFrame() {
    return lastActuatorFrame;
  }

  public List<BodyCommand> lastAppliedCommands() {
    return lastAppliedCommands;
  }

  public ImmutableWorldState advanceOneTick() {
    WorldSnapshot inputSnapshot = WorldSnapshots.fromWorldState(currentWorldState);
    for (WorldSnapshotSubscriber subscriber : snapshotSubscribers) {
      subscriber.onWorldSnapshot(inputSnapshot);
    }
    sensorEmulator.observe(inputSnapshot);
    List<SensorFrame> sensorFrames = sensorEmulator.releaseReady(inputSnapshot.header().simTimeNanos());
    for (SensorFrame sensorFrame : sensorFrames) {
      controlHost.enqueueSensorFrame(sensorFrame);
      if (replayLogWriter != null) {
        replayLogWriter.appendSensorFrame(sensorFrame);
      }
    }

    ActuatorFrame actuatorFrame = controlHost.runOneTick();
    lastActuatorFrame = actuatorFrame;
    if (replayLogWriter != null) {
      replayLogWriter.appendActuatorFrame(actuatorFrame);
    }

    List<BodyCommand> commands = actuatorCommandMapper.map(actuatorFrame, currentWorldState);
    lastAppliedCommands = List.copyOf(commands);
    int substeps = (int) (config.controlStepNanos() / config.physicsStepNanos());
    long simTimeNanos = actuatorFrame.header().simTimeNanos() - config.controlStepNanos();
    for (int substep = 1; substep <= substeps; substep++) {
      simTimeNanos += config.physicsStepNanos();
      FrameHeader stepHeader =
          new FrameHeader(
              actuatorFrame.header().protocolVersion(),
              simTimeNanos,
              (actuatorFrame.header().stepId() * 1000) + substep);
      var stepResult =
          physicsWorld.stepDetailed(
              currentWorldState,
              PhysicsStepRequest.earthLike(
                  stepHeader,
                  config.physicsStepNanos(),
                  commands,
                  contactGenerator.generateContacts(currentWorldState)));
      currentWorldState = stepResult.worldState();
      lastContactEvents = stepResult.contactEvents();
      lastContactTelemetryFrame =
          new ContactTelemetryFrame(
              stepHeader,
              lastContactEvents.stream()
                  .map(event ->
                      new ContactTelemetry(
                          event.bodyId(),
                          event.contactId(),
                          event.penetrationMeters(),
                          event.normalSpeedBefore(),
                          event.tangentialSpeedBefore(),
                          event.tangentialSpeedAfter(),
                          event.slipRatio(),
                          event.frictionCoefficient(),
                          event.restitutionCoefficient(),
                          event.rollingFrictionCoefficient(),
                          event.torsionalFrictionCoefficient()))
                  .toList());
      if (replayLogWriter != null) {
        WorldSnapshot outputSnapshot = WorldSnapshots.fromWorldState(currentWorldState);
        for (WorldSnapshotSubscriber subscriber : snapshotSubscribers) {
          subscriber.onWorldSnapshot(outputSnapshot);
        }
        replayLogWriter.appendWorldSnapshot(outputSnapshot);
        replayLogWriter.appendContactTelemetryFrame(lastContactTelemetryFrame);
      } else {
        WorldSnapshot outputSnapshot = WorldSnapshots.fromWorldState(currentWorldState);
        for (WorldSnapshotSubscriber subscriber : snapshotSubscribers) {
          subscriber.onWorldSnapshot(outputSnapshot);
        }
      }
    }
    return currentWorldState;
  }
}
