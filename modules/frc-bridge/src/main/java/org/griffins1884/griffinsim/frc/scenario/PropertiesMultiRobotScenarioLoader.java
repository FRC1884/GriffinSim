package org.griffins1884.griffinsim.frc.scenario;

import java.io.IOException;
import java.io.InputStream;
import java.io.Reader;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.frc.ClockMode;
import org.griffins1884.griffinsim.frc.ControlHostConfig;
import org.griffins1884.griffinsim.frc.MultiRobotEndpoint;
import org.griffins1884.griffinsim.physics.AugmentedActuatorCommandMapper;
import org.griffins1884.griffinsim.physics.BodyCollisionProfile;
import org.griffins1884.griffinsim.physics.BodyMotionProfile;
import org.griffins1884.griffinsim.physics.CompositeContactGenerator;
import org.griffins1884.griffinsim.physics.ContactGenerator;
import org.griffins1884.griffinsim.physics.DualChannelActuatorCommandMapper;
import org.griffins1884.griffinsim.physics.TriChannelActuatorCommandMapper;
import org.griffins1884.griffinsim.physics.FieldContactPresets;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.MaterialProfileRegistry;
import org.griffins1884.griffinsim.physics.PairwiseBodyContactGenerator;
import org.griffins1884.griffinsim.physics.SingleBodyActuatorCommandMapper;
import org.griffins1884.griffinsim.sensors.SensorEmissionConfig;

public final class PropertiesMultiRobotScenarioLoader {
  public MultiRobotScenario load(Path path) {
    try (InputStream inputStream = Files.newInputStream(path)) {
      return load(inputStream);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  public MultiRobotScenario load(InputStream inputStream) {
    Properties properties = new Properties();
    try {
      properties.load(inputStream);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return build(properties);
  }

  public MultiRobotScenario load(Reader reader) {
    Properties properties = new Properties();
    try {
      properties.load(reader);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return build(properties);
  }

  private MultiRobotScenario build(Properties properties) {
    String name = require(properties, "scenario.name");
    ClockMode clockMode = ClockMode.valueOf(properties.getProperty("control.clock_mode", "REAL_TIME"));
    long physicsStepNanos = parseLong(properties, "control.physics_step_nanos", 5_000_000L);
    long controlStepNanos = parseLong(properties, "control.control_step_nanos", 20_000_000L);
    int queueCapacity = parseInt(properties, "control.queue_capacity", 16);
    ControlHostConfig config = new ControlHostConfig(clockMode, physicsStepNanos, controlStepNanos, queueCapacity);

    List<RigidBodyState> bodies = loadBodies(properties);
    List<BodyCollisionProfile> collisionProfiles = loadCollisionProfiles(properties);
    ContactGenerator contactGenerator = buildContactGenerator(properties, collisionProfiles);
    List<MultiRobotEndpoint> endpoints = loadEndpoints(properties, config);

    return new MultiRobotScenario(
        name,
        config,
        new ImmutableWorldState(FrameHeader.current(0L, 0), bodies),
        endpoints,
        contactGenerator);
  }

  private static List<RigidBodyState> loadBodies(Properties properties) {
    List<RigidBodyState> bodies = new ArrayList<>();
    for (String bodyId : splitList(require(properties, "world.bodies"))) {
      bodies.add(
          new RigidBodyState(
              bodyId,
              parseDouble(properties, "world.body." + bodyId + ".x", 0.0),
              parseDouble(properties, "world.body." + bodyId + ".y", 0.0),
              parseDouble(properties, "world.body." + bodyId + ".z", 0.0),
              1.0,
              0.0,
              0.0,
              0.0,
              parseDouble(properties, "world.body." + bodyId + ".vx", 0.0),
              parseDouble(properties, "world.body." + bodyId + ".vy", 0.0),
              parseDouble(properties, "world.body." + bodyId + ".vz", 0.0),
              0.0,
              0.0,
              0.0));
    }
    return List.copyOf(bodies);
  }

  private static List<BodyCollisionProfile> loadCollisionProfiles(Properties properties) {
    List<BodyCollisionProfile> profiles = new ArrayList<>();
    for (String bodyId : splitList(require(properties, "world.bodies"))) {
      double hx = parseDouble(properties, "collision." + bodyId + ".half_extent_x", 0.4);
      double hy = parseDouble(properties, "collision." + bodyId + ".half_extent_y", 0.4);
      double hz = parseDouble(properties, "collision." + bodyId + ".half_extent_z", 0.4);
      String materialName = properties.getProperty("collision." + bodyId + ".material", "default-body");
      profiles.add(new BodyCollisionProfile(bodyId, hx, hy, hz, MaterialProfileRegistry.byName(materialName)));
    }
    return List.copyOf(profiles);
  }

  private static ContactGenerator buildContactGenerator(Properties properties, List<BodyCollisionProfile> collisionProfiles) {
    String fieldPreset = properties.getProperty("contact.field_preset", "rebuilt2026Arena");
    ContactGenerator fieldGenerator =
        switch (fieldPreset) {
          case "simpleArena" -> FieldContactPresets.simpleArena();
          case "rebuilt2026Arena" -> FieldContactPresets.rebuilt2026Arena();
          default -> throw new IllegalArgumentException("Unknown field preset: " + fieldPreset);
        };
    boolean pairwise = Boolean.parseBoolean(properties.getProperty("contact.enable_pairwise", "true"));
    if (!pairwise) {
      return fieldGenerator;
    }
    return new CompositeContactGenerator(List.of(fieldGenerator, new PairwiseBodyContactGenerator(collisionProfiles)));
  }

  private static List<MultiRobotEndpoint> loadEndpoints(Properties properties, ControlHostConfig config) {
    List<MultiRobotEndpoint> endpoints = new ArrayList<>();
    for (String bodyId : splitList(require(properties, "scenario.robots"))) {
      int pwmChannel = parseInt(properties, "robot." + bodyId + ".pwm_channel", 0);
      double pwmValue = parseDouble(properties, "robot." + bodyId + ".pwm_value", 0.0);
      long sensorSeed = parseLong(properties, "robot." + bodyId + ".sensor_seed", 1884L + pwmChannel);
      double massKg = parseDouble(properties, "robot." + bodyId + ".mass_kg", 50.0);
      double inertia = parseDouble(properties, "robot." + bodyId + ".inertia", 10.0);
      int sensorChannel = parseInt(properties, "robot." + bodyId + ".sensor_channel", pwmChannel);
      long latencyNanos = parseLong(properties, "robot." + bodyId + ".sensor_latency_nanos", 0L);
      boolean passive = Boolean.parseBoolean(properties.getProperty("robot." + bodyId + ".passive", "false"));
      boolean poseVelocityController = properties.containsKey("robot." + bodyId + ".xytheta_velocity_waypoints");
      boolean poseController = properties.containsKey("robot." + bodyId + ".xytheta_controller_waypoints");
      boolean holonomicController = properties.containsKey("robot." + bodyId + ".xy_controller_waypoints");
      var mapper =
          passive
              ? new AugmentedActuatorCommandMapper(
                  new SingleBodyActuatorCommandMapper(bodyId, massKg, inertia, 0.0, 0.0),
                  List.of(new BodyMotionProfile(bodyId, massKg, inertia)))
              : (poseVelocityController || poseController)
                  ? new TriChannelActuatorCommandMapper(
                      bodyId,
                      parseInt(properties, "robot." + bodyId + ".pwm_channel_x", pwmChannel),
                      parseInt(properties, "robot." + bodyId + ".pwm_channel_y", pwmChannel + 1),
                      parseInt(properties, "robot." + bodyId + ".pwm_channel_theta", pwmChannel + 2),
                      massKg,
                      inertia,
                      100.0,
                      20.0)
                  : holonomicController
                      ? new DualChannelActuatorCommandMapper(
                          bodyId,
                          parseInt(properties, "robot." + bodyId + ".pwm_channel_x", pwmChannel),
                          parseInt(properties, "robot." + bodyId + ".pwm_channel_y", pwmChannel + 1),
                          massKg,
                          inertia,
                          100.0,
                          0.0)
                      : new SingleBodyActuatorCommandMapper(bodyId, massKg, inertia, 100.0, 0.0);
      if (poseVelocityController) {
        HolonomicPosePwmCommandPlan plan =
            HolonomicVelocityPoseFollowerCompiler.compile(
                new HolonomicVelocityPoseFollowerSpec(
                    parsePoseSetpointList(properties.getProperty("robot." + bodyId + ".xytheta_velocity_waypoints")),
                    parseDouble(properties, "world.body." + bodyId + ".x", 0.0),
                    parseDouble(properties, "world.body." + bodyId + ".y", 0.0),
                    parseDouble(properties, "world.body." + bodyId + ".theta", 0.0),
                    parseDouble(properties, "robot." + bodyId + ".controller_kp", 1.0),
                    parseDouble(properties, "robot." + bodyId + ".controller_kp_theta", parseDouble(properties, "robot." + bodyId + ".controller_kp", 1.0)),
                    parseDouble(properties, "robot." + bodyId + ".controller_kv", 0.5),
                    parseDouble(properties, "robot." + bodyId + ".controller_kv_theta", 0.5),
                    parseDouble(properties, "robot." + bodyId + ".controller_max_pwm", Math.max(Math.abs(pwmValue), 1.0)),
                    parseDouble(properties, "robot." + bodyId + ".controller_meters_per_tick", 0.1),
                    parseDouble(properties, "robot." + bodyId + ".controller_radians_per_tick", 0.1),
                    parseDouble(properties, "robot." + bodyId + ".controller_tolerance_meters", 0.01),
                    parseDouble(properties, "robot." + bodyId + ".controller_tolerance_theta", 0.01)));
        endpoints.add(
            ScenarioEndpointFactory.scheduledPoseEndpoint(
                config,
                new ScheduledPoseRobotSpec(
                    bodyId,
                    parseInt(properties, "robot." + bodyId + ".pwm_channel_x", pwmChannel),
                    parseInt(properties, "robot." + bodyId + ".pwm_channel_y", pwmChannel + 1),
                    parseInt(properties, "robot." + bodyId + ".pwm_channel_theta", pwmChannel + 2),
                    plan,
                    sensorSeed,
                    new SensorEmissionConfig(bodyId, sensorChannel, latencyNanos, 1.0, 1.0, 0.0, 0.0),
                    mapper)));
      } else if (holonomicController) {
        HolonomicPwmCommandPlan plan =
            PathFollower2dCompiler.compile(
                new PathFollower2dSpec(
                    parseWaypoint2dList(properties.getProperty("robot." + bodyId + ".xy_controller_waypoints")),
                    parseDouble(properties, "world.body." + bodyId + ".x", 0.0),
                    parseDouble(properties, "world.body." + bodyId + ".y", 0.0),
                    parseDouble(properties, "robot." + bodyId + ".controller_kp", 1.0),
                    parseDouble(properties, "robot." + bodyId + ".controller_max_pwm", Math.max(Math.abs(pwmValue), 1.0)),
                    parseDouble(properties, "robot." + bodyId + ".controller_meters_per_tick", 0.1),
                    parseDouble(properties, "robot." + bodyId + ".controller_tolerance_meters", 0.01)));
        endpoints.add(
            ScenarioEndpointFactory.scheduledHolonomicEndpoint(
                config,
                new ScheduledHolonomicRobotSpec(
                    bodyId,
                    parseInt(properties, "robot." + bodyId + ".pwm_channel_x", pwmChannel),
                    parseInt(properties, "robot." + bodyId + ".pwm_channel_y", pwmChannel + 1),
                    plan,
                    sensorSeed,
                    new SensorEmissionConfig(bodyId, sensorChannel, latencyNanos, 1.0, 1.0, 0.0, 0.0),
                    mapper)));
      } else {
        PwmCommandTimeline timeline =
            properties.containsKey("robot." + bodyId + ".x_controller_waypoints")
                ? PathFollower1dCompiler.compile(
                    new PathFollower1dSpec(
                        parseWaypointList(properties.getProperty("robot." + bodyId + ".x_controller_waypoints")),
                        parseDouble(properties, "world.body." + bodyId + ".x", 0.0),
                        parseDouble(properties, "robot." + bodyId + ".controller_kp", 1.0),
                        parseDouble(properties, "robot." + bodyId + ".controller_max_pwm", Math.max(Math.abs(pwmValue), 1.0)),
                        parseDouble(properties, "robot." + bodyId + ".controller_meters_per_tick", 0.1),
                        parseDouble(properties, "robot." + bodyId + ".controller_tolerance_meters", 0.01)))
                : properties.containsKey("robot." + bodyId + ".x_waypoints")
                    ? parseWaypoints(properties.getProperty("robot." + bodyId + ".x_waypoints"))
                    : properties.containsKey("robot." + bodyId + ".pwm_schedule")
                        ? parseTimeline(properties.getProperty("robot." + bodyId + ".pwm_schedule"))
                        : PwmCommandTimeline.constant(pwmValue);
        endpoints.add(
            ScenarioEndpointFactory.scheduledPwmEndpoint(
                config,
                new ScheduledPwmRobotSpec(
                    bodyId,
                    pwmChannel,
                    timeline,
                    sensorSeed,
                    new SensorEmissionConfig(bodyId, sensorChannel, latencyNanos, 1.0, 1.0, 0.0, 0.0),
                    mapper)));
      }
    }
    return List.copyOf(endpoints);
  }

  private static PwmCommandTimeline parseTimeline(String spec) {
    List<ScheduledPwmCommand> commands = new ArrayList<>();
    for (String token : splitList(spec)) {
      String[] parts = token.split(":");
      if (parts.length != 3) {
        throw new IllegalArgumentException("Invalid pwm_schedule token: " + token);
      }
      commands.add(
          new ScheduledPwmCommand(
              Integer.parseInt(parts[0].trim()),
              Integer.parseInt(parts[1].trim()),
              Double.parseDouble(parts[2].trim())));
    }
    return new PwmCommandTimeline(commands, 0.0);
  }

  private static List<PoseSetpoint2d> parsePoseSetpointList(String spec) {
    List<PoseSetpoint2d> setpoints = new ArrayList<>();
    for (String token : splitList(spec)) {
      String[] parts = token.split(":");
      if (parts.length < 7 || parts.length > 8) {
        throw new IllegalArgumentException("Invalid pose setpoint token: " + token);
      }
      int tick = Integer.parseInt(parts[0].trim());
      double xMeters = Double.parseDouble(parts[1].trim());
      double yMeters = Double.parseDouble(parts[2].trim());
      double thetaRadians = Double.parseDouble(parts[3].trim());
      double vx = Double.parseDouble(parts[4].trim());
      double vy = Double.parseDouble(parts[5].trim());
      double omega = Double.parseDouble(parts[6].trim());
      double pwmMagnitude = parts.length == 8 ? Double.parseDouble(parts[7].trim()) : 1.0;
      setpoints.add(new PoseSetpoint2d(tick, xMeters, yMeters, thetaRadians, vx, vy, omega, pwmMagnitude));
    }
    return List.copyOf(setpoints);
  }

  private static List<PoseWaypoint2d> parsePoseWaypointList(String spec) {
    List<PoseWaypoint2d> waypoints = new ArrayList<>();
    for (String token : splitList(spec)) {
      String[] parts = token.split(":");
      if (parts.length < 4 || parts.length > 5) {
        throw new IllegalArgumentException("Invalid pose waypoint token: " + token);
      }
      int tick = Integer.parseInt(parts[0].trim());
      double xMeters = Double.parseDouble(parts[1].trim());
      double yMeters = Double.parseDouble(parts[2].trim());
      double thetaRadians = Double.parseDouble(parts[3].trim());
      double pwmMagnitude = parts.length == 5 ? Double.parseDouble(parts[4].trim()) : 0.5;
      waypoints.add(new PoseWaypoint2d(tick, xMeters, yMeters, thetaRadians, pwmMagnitude));
    }
    return List.copyOf(waypoints);
  }

  private static List<Waypoint2d> parseWaypoint2dList(String spec) {
    List<Waypoint2d> waypoints = new ArrayList<>();
    for (String token : splitList(spec)) {
      String[] parts = token.split(":");
      if (parts.length < 3 || parts.length > 4) {
        throw new IllegalArgumentException("Invalid 2d waypoint token: " + token);
      }
      int tick = Integer.parseInt(parts[0].trim());
      double xMeters = Double.parseDouble(parts[1].trim());
      double yMeters = Double.parseDouble(parts[2].trim());
      double pwmMagnitude = parts.length == 4 ? Double.parseDouble(parts[3].trim()) : 0.5;
      waypoints.add(new Waypoint2d(tick, xMeters, yMeters, pwmMagnitude));
    }
    return List.copyOf(waypoints);
  }

  private static List<Waypoint1d> parseWaypointList(String spec) {
    List<Waypoint1d> waypoints = new ArrayList<>();
    for (String token : splitList(spec)) {
      String[] parts = token.split(":");
      if (parts.length < 2 || parts.length > 3) {
        throw new IllegalArgumentException("Invalid waypoint token: " + token);
      }
      int tick = Integer.parseInt(parts[0].trim());
      double xMeters = Double.parseDouble(parts[1].trim());
      double pwmMagnitude = parts.length == 3 ? Double.parseDouble(parts[2].trim()) : 0.5;
      waypoints.add(new Waypoint1d(tick, xMeters, pwmMagnitude));
    }
    return List.copyOf(waypoints);
  }

  private static PwmCommandTimeline parseWaypoints(String spec) {
    return WaypointTimelineCompiler.compile1d(parseWaypointList(spec));
  }

  private static String require(Properties properties, String key) {
    String value = properties.getProperty(key);
    if (value == null || value.isBlank()) {
      throw new IllegalArgumentException("Missing required property: " + key);
    }
    return value;
  }

  private static List<String> splitList(String csv) {
    return java.util.Arrays.stream(csv.split(","))
        .map(String::trim)
        .filter(part -> !part.isEmpty())
        .toList();
  }

  private static int parseInt(Properties properties, String key, int fallback) {
    String value = properties.getProperty(key);
    return value == null ? fallback : Integer.parseInt(value.trim());
  }

  private static long parseLong(Properties properties, String key, long fallback) {
    String value = properties.getProperty(key);
    return value == null ? fallback : Long.parseLong(value.trim());
  }

  private static double parseDouble(Properties properties, String key, double fallback) {
    String value = properties.getProperty(key);
    return value == null ? fallback : Double.parseDouble(value.trim());
  }
}
