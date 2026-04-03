package org.griffins1884.griffinsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.junit.jupiter.api.Test;

class FlatFloorContactGeneratorTest {
  @Test
  void generatesDeterministicFloorContactsForPenetratingBodies() {
    ImmutableWorldState world =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(
                new RigidBodyState("robot", 0, 0, -0.02, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                new RigidBodyState("piece", 0, 0, 0.01, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));

    List<ContactConstraint> contacts = new FlatFloorContactGenerator(0.0).generateContacts(world);

    assertEquals(1, contacts.size());
    assertEquals("robot", contacts.get(0).bodyId());
    assertEquals(0.02, contacts.get(0).penetrationMeters(), 1e-9);
  }
}
