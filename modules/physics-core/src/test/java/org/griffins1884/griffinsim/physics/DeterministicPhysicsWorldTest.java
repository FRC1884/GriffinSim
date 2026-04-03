package org.griffins1884.griffinsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.junit.jupiter.api.Test;

class DeterministicPhysicsWorldTest {
  @Test
  void steppingIsStableAcrossUnsortedInputs() {
    DeterministicPhysicsWorld world = new DeterministicPhysicsWorld();
    ImmutableWorldState initial =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(
                new RigidBodyState("robot", 0, 0, 0.1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                new RigidBodyState("piece", 1, 0, 0.2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));

    PhysicsStepRequest leftRequest =
        PhysicsStepRequest.earthLike(
            FrameHeader.current(20_000_000L, 1),
            20_000_000L,
            List.of(
                new BodyCommand("robot", 50, 10, 10, 0, 490.3325, 0, 0, 0),
                new BodyCommand("piece", 1, 1, 0, 0, 9.80665, 0, 0, 0)),
            List.of(new ContactConstraint("robot", "floor-b", 0, 0, 1, 0.02), new ContactConstraint("robot", "floor-a", 0, 0, 1, 0.01)));
    PhysicsStepRequest rightRequest =
        PhysicsStepRequest.earthLike(
            FrameHeader.current(20_000_000L, 1),
            20_000_000L,
            List.of(
                new BodyCommand("piece", 1, 1, 0, 0, 9.80665, 0, 0, 0),
                new BodyCommand("robot", 50, 10, 10, 0, 490.3325, 0, 0, 0)),
            List.of(new ContactConstraint("robot", "floor-a", 0, 0, 1, 0.01), new ContactConstraint("robot", "floor-b", 0, 0, 1, 0.02)));

    ImmutableWorldState left = world.step(initial, leftRequest);
    ImmutableWorldState right = world.step(initial, rightRequest);

    assertEquals(left, right);
    assertEquals(List.of("piece", "robot"), left.bodies().stream().map(RigidBodyState::bodyId).toList());
  }

  @Test
  void contactCorrectionRemovesNegativeVelocityIntoSurface() {
    DeterministicPhysicsWorld world = new DeterministicPhysicsWorld();
    ImmutableWorldState initial =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot", 0, 0, -0.01, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0)));

    ImmutableWorldState next =
        world.step(
            initial,
            PhysicsStepRequest.earthLike(
                FrameHeader.current(5_000_000L, 1),
                5_000_000L,
                List.of(new BodyCommand("robot", 50, 10, 0, 0, 0, 0, 0, 0)),
                List.of(new ContactConstraint("robot", "floor", 0, 0, 1, 0.05))));

    RigidBodyState robot = next.bodies().get(0);
    assertEquals(0.0, robot.vz(), 1e-9);
    assertEquals(0.03475483375, robot.z(), 1e-9);
  }


  @Test
  void frictionAndRestitutionModifyContactResponseAndEmitTelemetry() {
    DeterministicPhysicsWorld world = new DeterministicPhysicsWorld();
    ImmutableWorldState initial =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot", 0, 0, -0.01, 1, 0, 0, 0, 2.0, 0, -1.0, 0, 0, 0)));

    PhysicsStepResult result =
        world.stepDetailed(
            initial,
            PhysicsStepRequest.earthLike(
                FrameHeader.current(5_000_000L, 1),
                5_000_000L,
                List.of(new BodyCommand("robot", 50, 10, 0, 0, 0, 0, 0, 0)),
                List.of(new ContactConstraint("robot", "floor", 0, 0, 1, 0.05, 0.5, 0.25, 0.2, 0.4))));

    RigidBodyState robot = result.worldState().bodies().get(0);
    assertEquals(0.2622583125, robot.vz(), 1e-9);
    assertEquals(1.0, robot.vx(), 1e-9);
    assertEquals(1, result.contactEvents().size());
    assertEquals(-1.04903325, result.contactEvents().get(0).normalSpeedBefore(), 1e-8);
    assertEquals(2.0, result.contactEvents().get(0).tangentialSpeedBefore(), 1e-9);
    assertEquals(0.5, result.contactEvents().get(0).frictionCoefficient(), 1e-9);
    assertEquals(0.25, result.contactEvents().get(0).restitutionCoefficient(), 1e-9);
    assertEquals(0.2, result.contactEvents().get(0).rollingFrictionCoefficient(), 1e-9);
    assertEquals(0.4, result.contactEvents().get(0).torsionalFrictionCoefficient(), 1e-9);
    assertEquals(1.0, result.contactEvents().get(0).tangentialSpeedAfter(), 1e-9);
    assertEquals(0.5, result.contactEvents().get(0).slipRatio(), 1e-9);
  }

}
