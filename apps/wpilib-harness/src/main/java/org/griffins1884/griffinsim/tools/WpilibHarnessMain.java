package org.griffins1884.griffinsim.tools;

import java.security.MessageDigest;
import java.util.HexFormat;
import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.physics.BodyCommand;
import org.griffins1884.griffinsim.physics.DeterministicPhysicsWorld;
import org.griffins1884.griffinsim.physics.FieldContactPresets;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.TriChannelActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.DeterministicSensorEmulator;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;

public final class WpilibHarnessMain {
  private WpilibHarnessMain() {}

  public static void main(String[] args) throws Exception {
    WpilibSimulationHarness.RunResult<FixtureTimedRobot> result =
        WpilibSimulationHarness.run(FixtureTimedRobot::new, defaultSpec());
    FixtureTimedRobot robot = result.robot();
    WpilibSimulationHarness.TraceStep lastTrace = result.trace().get(result.trace().size() - 1);
    RigidBodyState robotBody =
        result.finalWorldState().bodies().stream()
            .filter(body -> body.bodyId().equals("robot"))
            .findFirst()
            .orElseThrow();

    System.out.println("robot_events=" + String.join(",", robot.lifecycleEvents()));
    System.out.println("disabled_periodic_calls=" + robot.disabledPeriodicCalls());
    System.out.println("teleop_periodic_calls=" + robot.teleopPeriodicCalls());
    System.out.println("first_motor_command=" + robot.firstMotorCommand());
    System.out.println("last_motor_command=" + robot.lastMotorCommand());
    System.out.println("last_encoder_distance=" + robot.lastEncoderDistance());
    System.out.println("last_gyro_angle_deg=" + robot.lastGyroAngleDegrees());
    System.out.println("final_world_x=" + robotBody.x());
    System.out.println("final_world_y=" + robotBody.y());
    System.out.println("final_heading_rad=" + lastTrace.headingRadians());
    System.out.println("final_linear_speed=" + lastTrace.linearSpeedMetersPerSecond());
    System.out.println("final_angular_velocity=" + lastTrace.angularVelocityRadiansPerSecond());
    System.out.println("final_world_z=" + robotBody.z());
    System.out.println("last_pwm=" + lastTrace.pwmByChannel());
    System.out.println("last_applied_commands=" + lastTrace.appliedCommands().stream().map(BodyCommand::bodyId).toList());
    System.out.println("last_contact_count=" + result.lastContactTelemetryFrame().contacts().size());
    System.out.println("executed_ticks=" + result.executedTicks());
    System.out.println("replay_sha256=" + sha256(result.replayBytes()));
  }

  static WpilibSimulationHarness.HarnessSpec defaultSpec() {
    return new WpilibSimulationHarness.HarnessSpec(
        ControlHostConfig.defaultLockstep(),
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot", 0.1, 0.0, -0.01, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))),
        new DeterministicPhysicsWorld(),
        new DeterministicSensorEmulator(SensorEmissionConfig.immediate("robot", 0), 1884L),
        new TriChannelActuatorCommandMapper("robot", 0, 1, 2, 50.0, 10.0, 120.0, 40.0),
        FieldContactPresets.simpleArena(),
        "robot",
        4,
        List.of(
            WpilibSimulationHarness.DriverStationPhase.disabled("startup-disabled", 1),
            WpilibSimulationHarness.DriverStationPhase.teleop("teleop", 200)));
  }

  private static String sha256(byte[] bytes) throws Exception {
    MessageDigest digest = MessageDigest.getInstance("SHA-256");
    return HexFormat.of().formatHex(digest.digest(bytes));
  }
}
