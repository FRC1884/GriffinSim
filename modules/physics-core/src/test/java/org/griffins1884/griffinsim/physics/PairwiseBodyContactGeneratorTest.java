package org.griffins1884.griffinsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.junit.jupiter.api.Test;

class PairwiseBodyContactGeneratorTest {
  @Test
  void generatesReciprocalContactsForOverlappingBodies() {
    PairwiseBodyContactGenerator generator =
        new PairwiseBodyContactGenerator(
            List.of(
                new BodyCollisionProfile("robot", 0.4, 0.4, 0.4, MaterialProfiles.DEFAULT_BODY),
                new BodyCollisionProfile("gamepiece", 0.2, 0.2, 0.2, MaterialProfiles.HDPE)));
    ImmutableWorldState world =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(
                new RigidBodyState("robot", 0.0, 0.0, 0.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                new RigidBodyState("gamepiece", 0.3, 0.0, 0.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));

    List<ContactConstraint> contacts = generator.generateContacts(world);

    assertEquals(2, contacts.size());
    assertTrue(contacts.stream().anyMatch(contact -> contact.bodyId().equals("robot")));
    assertTrue(contacts.stream().anyMatch(contact -> contact.bodyId().equals("gamepiece")));
    assertTrue(contacts.stream().allMatch(contact -> contact.rollingFrictionCoefficient() > 0.0));
  }
}
