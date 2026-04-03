package org.griffins1884.sim3d;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Planar drivetrain backend with swerve module access for encoder-style simulation consumers. */
public interface SwerveDriveBackend extends PlanarDriveBackend {
  SwerveModuleSimulation[] getModuleSimulations();
}
