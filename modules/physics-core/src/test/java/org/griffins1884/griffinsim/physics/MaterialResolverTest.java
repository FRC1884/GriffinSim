package org.griffins1884.griffinsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class MaterialResolverTest {
  @Test
  void combinesMaterialCoefficientsDeterministically() {
    double friction = MaterialResolver.combinedFriction(MaterialProfiles.DEFAULT_BODY, MaterialProfiles.HDPE);
    double restitution = MaterialResolver.combinedRestitution(MaterialProfiles.DEFAULT_BODY, MaterialProfiles.HDPE);
    double rolling = MaterialResolver.combinedRollingFriction(MaterialProfiles.DEFAULT_BODY, MaterialProfiles.HDPE);
    double torsional = MaterialResolver.combinedTorsionalFriction(MaterialProfiles.DEFAULT_BODY, MaterialProfiles.HDPE);

    assertEquals(Math.sqrt(0.8 * 0.35), friction, 1e-12);
    assertEquals(0.1, restitution, 1e-12);
    assertEquals(Math.sqrt(0.05 * 0.02), rolling, 1e-12);
    assertEquals(Math.sqrt(0.03 * 0.015), torsional, 1e-12);
  }
}
