package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.StringReader;
import org.junit.jupiter.api.Test;

class PropertiesMultiRobotScenarioLoaderTest {
  @Test
  void loadsExternalPropertiesScenarioAndRunsDeterministically() {
    String spec =
        String.join(
            "\n",
            "scenario.name=spec-head-on",
            "scenario.robots=robot-a,robot-b",
            "control.clock_mode=REAL_TIME",
            "control.physics_step_nanos=5000000",
            "control.control_step_nanos=20000000",
            "control.queue_capacity=16",
            "world.bodies=robot-a,robot-b",
            "world.body.robot-a.x=0.0",
            "world.body.robot-b.x=0.6",
            "collision.robot-a.half_extent_x=0.4",
            "collision.robot-a.half_extent_y=0.4",
            "collision.robot-a.half_extent_z=0.4",
            "collision.robot-b.half_extent_x=0.4",
            "collision.robot-b.half_extent_y=0.4",
            "collision.robot-b.half_extent_z=0.4",
            "contact.field_preset=rebuilt2026Arena",
            "contact.enable_pairwise=true",
            "robot.robot-a.pwm_channel=0",
            "robot.robot-a.pwm_value=0.5",
            "robot.robot-a.sensor_channel=0",
            "robot.robot-a.mass_kg=50.0",
            "robot.robot-a.inertia=10.0",
            "robot.robot-b.pwm_channel=1",
            "robot.robot-b.pwm_value=-0.5",
            "robot.robot-b.sensor_channel=1",
            "robot.robot-b.mass_kg=50.0",
            "robot.robot-b.inertia=10.0");

    PropertiesMultiRobotScenarioLoader loader = new PropertiesMultiRobotScenarioLoader();
    MultiRobotScenario scenario = loader.load(new StringReader(spec));
    MultiRobotRunResult first = new MultiRobotScenarioRunner().run(loader.load(new StringReader(spec)), 2);
    MultiRobotRunResult second = new MultiRobotScenarioRunner().run(loader.load(new StringReader(spec)), 2);

    assertEquals("spec-head-on", scenario.name());
    assertEquals(2, scenario.endpoints().size());
    assertEquals(first.finalWorldState(), second.finalWorldState());
    assertTrue(java.util.Arrays.equals(first.replayBytes(), second.replayBytes()));
  }

  @Test
  void loadsScheduledPwmSequenceFromProperties() {
    String spec =
        String.join(
            "\n",
            "scenario.name=scheduled-pwm",
            "scenario.robots=robot-a",
            "control.clock_mode=REAL_TIME",
            "control.physics_step_nanos=5000000",
            "control.control_step_nanos=20000000",
            "control.queue_capacity=16",
            "world.bodies=robot-a",
            "world.body.robot-a.x=0.0",
            "collision.robot-a.half_extent_x=0.4",
            "collision.robot-a.half_extent_y=0.4",
            "collision.robot-a.half_extent_z=0.4",
            "contact.field_preset=simpleArena",
            "contact.enable_pairwise=false",
            "robot.robot-a.pwm_channel=0",
            "robot.robot-a.pwm_schedule=0:0:0.1,1:2:0.9,3:4:-0.3",
            "robot.robot-a.sensor_channel=0",
            "robot.robot-a.mass_kg=50.0",
            "robot.robot-a.inertia=10.0");

    MultiRobotScenario scenario = new PropertiesMultiRobotScenarioLoader().load(new StringReader(spec));
    MultiRobotRunResult result = new MultiRobotScenarioRunner().run(scenario, 3);

    assertEquals("scheduled-pwm", scenario.name());
    assertEquals(3, result.contactTelemetryFrames().size());
  }


  @Test
  void loadsWaypointBasedSequenceFromProperties() {
    String spec =
        String.join(
            "\n",
            "scenario.name=waypoint-sequence",
            "scenario.robots=robot-a",
            "control.clock_mode=REAL_TIME",
            "control.physics_step_nanos=5000000",
            "control.control_step_nanos=20000000",
            "control.queue_capacity=16",
            "world.bodies=robot-a",
            "world.body.robot-a.x=0.0",
            "collision.robot-a.half_extent_x=0.4",
            "collision.robot-a.half_extent_y=0.4",
            "collision.robot-a.half_extent_z=0.4",
            "contact.field_preset=simpleArena",
            "contact.enable_pairwise=false",
            "robot.robot-a.pwm_channel=0",
            "robot.robot-a.x_waypoints=0:0.0:0.2,1:1.0:0.8,3:-0.5:0.4",
            "robot.robot-a.sensor_channel=0",
            "robot.robot-a.mass_kg=50.0",
            "robot.robot-a.inertia=10.0");

    MultiRobotScenario scenario = new PropertiesMultiRobotScenarioLoader().load(new StringReader(spec));
    MultiRobotRunResult result = new MultiRobotScenarioRunner().run(scenario, 4);

    assertEquals("waypoint-sequence", scenario.name());
    assertTrue(result.finalWorldState().bodies().get(0).x() != 0.0);
  }


  @Test
  void loadsControllerWaypointSequenceFromProperties() {
    String spec =
        String.join(
            "\n",
            "scenario.name=controller-waypoint-sequence",
            "scenario.robots=robot-a",
            "control.clock_mode=REAL_TIME",
            "control.physics_step_nanos=5000000",
            "control.control_step_nanos=20000000",
            "control.queue_capacity=16",
            "world.bodies=robot-a",
            "world.body.robot-a.x=0.0",
            "collision.robot-a.half_extent_x=0.4",
            "collision.robot-a.half_extent_y=0.4",
            "collision.robot-a.half_extent_z=0.4",
            "contact.field_preset=simpleArena",
            "contact.enable_pairwise=false",
            "robot.robot-a.pwm_channel=0",
            "robot.robot-a.x_controller_waypoints=0:0.0:0.2,1:1.0:0.8,3:-0.5:0.4",
            "robot.robot-a.controller_kp=1.2",
            "robot.robot-a.controller_max_pwm=0.9",
            "robot.robot-a.controller_meters_per_tick=0.2",
            "robot.robot-a.controller_tolerance_meters=0.01",
            "robot.robot-a.sensor_channel=0",
            "robot.robot-a.mass_kg=50.0",
            "robot.robot-a.inertia=10.0");

    MultiRobotScenario scenario = new PropertiesMultiRobotScenarioLoader().load(new StringReader(spec));
    MultiRobotRunResult result = new MultiRobotScenarioRunner().run(scenario, 4);

    assertEquals("controller-waypoint-sequence", scenario.name());
    assertTrue(result.finalWorldState().bodies().get(0).x() != 0.0);
  }


  @Test
  void loadsHolonomicControllerWaypointSequenceFromProperties() {
    String spec =
        String.join(
            "\n",
            "scenario.name=holonomic-controller-sequence",
            "scenario.robots=robot-a",
            "control.clock_mode=REAL_TIME",
            "control.physics_step_nanos=5000000",
            "control.control_step_nanos=20000000",
            "control.queue_capacity=16",
            "world.bodies=robot-a",
            "world.body.robot-a.x=0.0",
            "world.body.robot-a.y=0.0",
            "collision.robot-a.half_extent_x=0.4",
            "collision.robot-a.half_extent_y=0.4",
            "collision.robot-a.half_extent_z=0.4",
            "contact.field_preset=simpleArena",
            "contact.enable_pairwise=false",
            "robot.robot-a.pwm_channel=0",
            "robot.robot-a.pwm_channel_x=0",
            "robot.robot-a.pwm_channel_y=1",
            "robot.robot-a.xy_controller_waypoints=0:0.0:0.0:0.2,1:1.0:0.5:0.8,3:-0.5:-0.25:0.4",
            "robot.robot-a.controller_kp=1.2",
            "robot.robot-a.controller_max_pwm=0.9",
            "robot.robot-a.controller_meters_per_tick=0.2",
            "robot.robot-a.controller_tolerance_meters=0.01",
            "robot.robot-a.sensor_channel=0",
            "robot.robot-a.mass_kg=50.0",
            "robot.robot-a.inertia=10.0");

    MultiRobotScenario scenario = new PropertiesMultiRobotScenarioLoader().load(new StringReader(spec));
    MultiRobotRunResult result = new MultiRobotScenarioRunner().run(scenario, 4);

    assertEquals("holonomic-controller-sequence", scenario.name());
    assertTrue(result.finalWorldState().bodies().get(0).x() != 0.0 || result.finalWorldState().bodies().get(0).y() != 0.0);
  }


  @Test
  void loadsPoseControllerWaypointSequenceFromProperties() {
    String spec =
        String.join(
            "\n",
            "scenario.name=pose-controller-sequence",
            "scenario.robots=robot-a",
            "control.clock_mode=REAL_TIME",
            "control.physics_step_nanos=5000000",
            "control.control_step_nanos=20000000",
            "control.queue_capacity=16",
            "world.bodies=robot-a",
            "world.body.robot-a.x=0.0",
            "world.body.robot-a.y=0.0",
            "world.body.robot-a.theta=0.0",
            "collision.robot-a.half_extent_x=0.4",
            "collision.robot-a.half_extent_y=0.4",
            "collision.robot-a.half_extent_z=0.4",
            "contact.field_preset=simpleArena",
            "contact.enable_pairwise=false",
            "robot.robot-a.pwm_channel=0",
            "robot.robot-a.pwm_channel_x=0",
            "robot.robot-a.pwm_channel_y=1",
            "robot.robot-a.pwm_channel_theta=2",
            "robot.robot-a.xytheta_controller_waypoints=0:0.0:0.0:0.0:0.2,1:1.0:0.5:1.0:0.8,3:-0.5:-0.25:-0.5:0.4",
            "robot.robot-a.controller_kp=1.2",
            "robot.robot-a.controller_kp_theta=0.8",
            "robot.robot-a.controller_max_pwm=0.9",
            "robot.robot-a.controller_meters_per_tick=0.2",
            "robot.robot-a.controller_radians_per_tick=0.1",
            "robot.robot-a.controller_tolerance_meters=0.01",
            "robot.robot-a.controller_tolerance_theta=0.01",
            "robot.robot-a.sensor_channel=0",
            "robot.robot-a.mass_kg=50.0",
            "robot.robot-a.inertia=10.0");

    MultiRobotScenario scenario = new PropertiesMultiRobotScenarioLoader().load(new StringReader(spec));
    MultiRobotRunResult result = new MultiRobotScenarioRunner().run(scenario, 4);

    assertEquals("pose-controller-sequence", scenario.name());
    assertEquals(4, result.contactTelemetryFrames().size());
  }


  @Test
  void loadsVelocityAwarePoseControllerSequenceFromProperties() {
    String spec =
        String.join(
            "\n",
            "scenario.name=pose-velocity-controller-sequence",
            "scenario.robots=robot-a",
            "control.clock_mode=REAL_TIME",
            "control.physics_step_nanos=5000000",
            "control.control_step_nanos=20000000",
            "control.queue_capacity=16",
            "world.bodies=robot-a",
            "world.body.robot-a.x=0.0",
            "world.body.robot-a.y=0.0",
            "world.body.robot-a.theta=0.0",
            "collision.robot-a.half_extent_x=0.4",
            "collision.robot-a.half_extent_y=0.4",
            "collision.robot-a.half_extent_z=0.4",
            "contact.field_preset=simpleArena",
            "contact.enable_pairwise=false",
            "robot.robot-a.pwm_channel=0",
            "robot.robot-a.pwm_channel_x=0",
            "robot.robot-a.pwm_channel_y=1",
            "robot.robot-a.pwm_channel_theta=2",
            "robot.robot-a.xytheta_velocity_waypoints=0:0.0:0.0:0.0:0.0:0.0:0.0:0.2,1:1.0:0.5:1.0:0.3:0.2:0.1:0.8,3:-0.5:-0.25:-0.5:-0.2:-0.1:-0.05:0.4",
            "robot.robot-a.controller_kp=1.2",
            "robot.robot-a.controller_kp_theta=0.8",
            "robot.robot-a.controller_kv=0.5",
            "robot.robot-a.controller_kv_theta=0.3",
            "robot.robot-a.controller_max_pwm=0.9",
            "robot.robot-a.controller_meters_per_tick=0.2",
            "robot.robot-a.controller_radians_per_tick=0.1",
            "robot.robot-a.controller_tolerance_meters=0.01",
            "robot.robot-a.controller_tolerance_theta=0.01",
            "robot.robot-a.sensor_channel=0",
            "robot.robot-a.mass_kg=50.0",
            "robot.robot-a.inertia=10.0");

    MultiRobotScenario scenario = new PropertiesMultiRobotScenarioLoader().load(new StringReader(spec));
    MultiRobotRunResult result = new MultiRobotScenarioRunner().run(scenario, 4);

    assertEquals("pose-velocity-controller-sequence", scenario.name());
    assertTrue(result.finalWorldState().bodies().get(0).x() != 0.0 || result.finalWorldState().bodies().get(0).y() != 0.0 || result.finalWorldState().bodies().get(0).wz() != 0.0);
  }

}
