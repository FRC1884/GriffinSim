package org.griffins1884.griffinsim.physics.seasons.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.physics.ContactConstraint;
import org.griffins1884.griffinsim.physics.ContactGenerator;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.junit.jupiter.api.Test;

class Rebuilt2026FieldContactPresetTest {
  @Test
  void blocksHubAndTowerGeometry() {
    ContactGenerator generator = Rebuilt2026FieldContactPreset.create();

    ImmutableWorldState hubWorld =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot", Rebuilt2026FieldConstants.HUB_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));
    ImmutableWorldState towerWorld =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState(
                "robot",
                Rebuilt2026FieldConstants.TOWER_FRONT_FACE_X_BLUE_METERS - (Rebuilt2026FieldConstants.TOWER_DEPTH_METERS * 0.5),
                Rebuilt2026FieldConstants.TOWER_CENTER_Y_BLUE_METERS
                    + (Rebuilt2026FieldConstants.TOWER_OPENING_WIDTH_METERS * 0.5)
                    + ((Rebuilt2026FieldConstants.TOWER_WIDTH_METERS - Rebuilt2026FieldConstants.TOWER_OPENING_WIDTH_METERS) * 0.25),
                0.5,
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

    List<ContactConstraint> hubContacts = generator.generateContacts(hubWorld);
    List<ContactConstraint> towerContacts = generator.generateContacts(towerWorld);

    assertTrue(hubContacts.stream().anyMatch(contact -> contact.contactId().startsWith("blue-hub")));
    assertTrue(towerContacts.stream().anyMatch(contact -> contact.contactId().contains("tower")));
  }

  @Test
  void trenchEdgesAreRepresentedAsOverheadBarriers() {
    ContactGenerator generator = Rebuilt2026FieldContactPreset.create();
    ImmutableWorldState trenchEdgeWorld =
        new ImmutableWorldState(
            FrameHeader.current(0L, 0),
            List.of(new RigidBodyState("robot", Rebuilt2026FieldConstants.TRENCH_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.TRENCH_UPPER_CENTER_Y_METERS + (Rebuilt2026FieldConstants.TRENCH_FOOTPRINT_WIDTH_METERS * 0.5) - 0.01, 0.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));

    List<ContactConstraint> contacts = generator.generateContacts(trenchEdgeWorld);

    assertTrue(contacts.stream().anyMatch(contact -> contact.contactId().contains("edge")));
  }
}
