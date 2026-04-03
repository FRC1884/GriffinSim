package org.griffins1884.griffinsim.physics.seasons.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class Rebuilt2026FieldConstantsTest {
  @Test
  void preservesLegacyFieldReferenceValues() {
    assertEquals(16.513048, Rebuilt2026FieldConstants.FIELD_LENGTH_METERS, 1e-6);
    assertEquals(8.042656, Rebuilt2026FieldConstants.FIELD_WIDTH_METERS, 1e-6);
    assertEquals(4.611624, Rebuilt2026FieldConstants.HUB_CENTER_X_BLUE_METERS, 1e-6);
    assertEquals(11.901424, Rebuilt2026FieldConstants.HUB_CENTER_X_RED_METERS, 1e-6);
    assertEquals(4.021328, Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS, 1e-6);
    assertTrue(Rebuilt2026FieldConstants.TOWER_OPENING_WIDTH_METERS > 0.8);
    assertTrue(Rebuilt2026FieldConstants.TRENCH_OPENING_HEIGHT_METERS > 0.5);
  }
}
