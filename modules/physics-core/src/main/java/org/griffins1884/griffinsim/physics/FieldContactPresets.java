package org.griffins1884.griffinsim.physics;

import java.util.List;
import org.griffins1884.griffinsim.physics.seasons.rebuilt2026.Rebuilt2026FieldContactPreset;

public final class FieldContactPresets {
  private FieldContactPresets() {}

  public static ContactGenerator simpleArena() {
    return new CompositeContactGenerator(
        List.of(
            new FlatFloorContactGenerator(0.0),
            new AxisAlignedObstacleContactGenerator(
                List.of(
                    new AxisAlignedBoxObstacle("blue-hub", 2.8, 3.4, -0.5, 0.5, 0.0, 1.2),
                    new AxisAlignedBoxObstacle("red-hub", 12.8, 13.4, -0.5, 0.5, 0.0, 1.2),
                    new AxisAlignedBoxObstacle("left-wall", -0.2, 0.0, -8.0, 8.0, 0.0, 2.0),
                    new AxisAlignedBoxObstacle("right-wall", 16.0, 16.2, -8.0, 8.0, 0.0, 2.0)))));
  }

  public static ContactGenerator rebuilt2026Arena() {
    return Rebuilt2026FieldContactPreset.create();
  }
}
