package org.griffins1884.griffinsim.physics.seasons.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.physics.ContactConstraint;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.junit.jupiter.api.Test;

class Rebuilt2026TerrainContactGeneratorTest {
  @Test
  void bumpRegionsProduceRaisedTerrainContacts() {
    Rebuilt2026TerrainContactGenerator generator =
        new Rebuilt2026TerrainContactGenerator(List.of(new BodyHeightProfile("robot", 0.9)));
    ImmutableWorldState world =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState(
                "robot",
                Rebuilt2026FieldConstants.HUB_CENTER_X_BLUE_METERS,
                Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS + Rebuilt2026FieldConstants.inchesToMeters(60.0),
                0.0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0)));

    List<ContactConstraint> contacts = generator.generateContacts(world);

    assertTrue(contacts.stream().anyMatch(contact -> contact.contactId().contains("terrain")));
  }

  @Test
  void trenchOpeningsEnforceOverheadClearance() {
    Rebuilt2026TerrainContactGenerator generator =
        new Rebuilt2026TerrainContactGenerator(List.of(new BodyHeightProfile("robot", 1.0)));
    ImmutableWorldState world =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState(
                "robot",
                Rebuilt2026FieldConstants.TRENCH_CENTER_X_BLUE_METERS,
                Rebuilt2026FieldConstants.TRENCH_UPPER_CENTER_Y_METERS,
                Rebuilt2026FieldConstants.TRENCH_OPENING_HEIGHT_METERS - 0.2,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0)));

    List<ContactConstraint> contacts = generator.generateContacts(world);

    ContactConstraint ceiling = contacts.stream().filter(contact -> contact.contactId().contains("trench-ceiling")).findFirst().orElseThrow();
    assertEquals(-1.0, ceiling.normalZ(), 1e-9);
    assertTrue(ceiling.penetrationMeters() > 0.0);
  }
}
