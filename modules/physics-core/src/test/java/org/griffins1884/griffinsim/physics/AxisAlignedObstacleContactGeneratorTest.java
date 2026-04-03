package org.griffins1884.griffinsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.junit.jupiter.api.Test;

class AxisAlignedObstacleContactGeneratorTest {
  @Test
  void generatesOutwardNormalForSmallestPenetrationAxis() {
    ImmutableWorldState world =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot", 1.95, 1.5, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));
    AxisAlignedObstacleContactGenerator generator =
        new AxisAlignedObstacleContactGenerator(
            List.of(new AxisAlignedBoxObstacle("tower", 1.0, 2.0, 1.0, 2.0, 0.0, 1.0)));

    List<ContactConstraint> contacts = generator.generateContacts(world);

    assertEquals(1, contacts.size());
    ContactConstraint contact = contacts.get(0);
    assertEquals("tower:box", contact.contactId());
    assertEquals(1.0, contact.normalX(), 1e-9);
    assertEquals(0.05, contact.penetrationMeters(), 1e-9);
  }
}
