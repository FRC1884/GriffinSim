package org.griffins1884.sim3d.seasonspecific.rebuilt2026;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.griffins1884.sim3d.integration.FieldMarkerSample;
import org.griffins1884.sim3d.integration.SimulationScenario;
import org.junit.jupiter.api.Test;

class Rebuilt2026IntegrationAssetsTest {
  @Test
  void exposesStaticFieldMarkersForVisualization() {
    FieldMarkerSample[] markers = Rebuilt2026FieldContactModel.INSTANCE.getFieldMarkers();

    assertTrue(markers.length >= 12);
    assertEquals("blue-left-bump", markers[0].id());
    assertTrue(markers[0].pose().getZ() > 0.0);
  }

  @Test
  void exposesRepeatableAutonomousScenariosInsideFieldBounds() {
    for (SimulationScenario scenario : Rebuilt2026AutoScenarios.all()) {
      assertFalse(scenario.name().isBlank());
      assertTrue(scenario.startPose().getX() >= 0.0);
      assertTrue(scenario.targetPose().getX() <= Rebuilt2026FieldContactModel.fieldLengthMeters());
      assertTrue(scenario.startPose().getY() >= 0.0);
      assertTrue(scenario.targetPose().getY() <= Rebuilt2026FieldContactModel.fieldWidthMeters());
    }
  }
}
