package org.griffins1884.griffinsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.junit.jupiter.api.Test;

class CompositeContactGeneratorTest {
  @Test
  void mergesAndSortsContactsFromMultipleGenerators() {
    ImmutableWorldState world =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot", -0.01, 0.0, -0.02, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));

    CompositeContactGenerator generator =
        new CompositeContactGenerator(
            List.of(
                new AxisAlignedObstacleContactGenerator(
                    List.of(new AxisAlignedBoxObstacle("wall", -0.1, 0.0, -1.0, 1.0, -1.0, 1.0))),
                new FlatFloorContactGenerator(0.0)));

    List<ContactConstraint> contacts = generator.generateContacts(world);

    assertEquals(List.of("robot", "robot"), contacts.stream().map(ContactConstraint::bodyId).toList());
    assertEquals(List.of("robot:floor", "wall:box"), contacts.stream().map(ContactConstraint::contactId).toList());
  }
}
