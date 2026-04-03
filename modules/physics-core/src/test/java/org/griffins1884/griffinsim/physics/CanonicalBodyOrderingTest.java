package org.griffins1884.griffinsim.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import org.griffins1884.griffinsim.contracts.FrameHeader;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.junit.jupiter.api.Test;

class CanonicalBodyOrderingTest {
  @Test
  void worldStateNormalizesBodyOrder() {
    ImmutableWorldState state =
        new ImmutableWorldState(
            FrameHeader.current(5_000_000L, 1),
            List.of(
                new RigidBodyState("zeta", 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                new RigidBodyState("alpha", 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)));

    assertEquals(List.of("alpha", "zeta"), state.bodies().stream().map(RigidBodyState::bodyId).toList());
  }
}
