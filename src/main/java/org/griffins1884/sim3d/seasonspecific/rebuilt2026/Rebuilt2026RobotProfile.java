package org.griffins1884.sim3d.seasonspecific.rebuilt2026;

import edu.wpi.first.math.util.Units;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.ChassisMassProperties;

/** Default chassis envelope and mass properties for 2026 rebuilt field simulation work. */
public final class Rebuilt2026RobotProfile {
  public static final ChassisFootprint DEFAULT_FOOTPRINT =
      new ChassisFootprint(
          Units.inchesToMeters(34.0),
          Units.inchesToMeters(34.0),
          Units.inchesToMeters(21.75),
          Units.inchesToMeters(1.5));

  public static final ChassisMassProperties DEFAULT_CHASSIS_MASS_PROPERTIES =
      new ChassisMassProperties(
          45.0,
          Units.inchesToMeters(11.5),
          Units.inchesToMeters(27.5),
          Units.inchesToMeters(27.5),
          1.2);

  private Rebuilt2026RobotProfile() {}
}
