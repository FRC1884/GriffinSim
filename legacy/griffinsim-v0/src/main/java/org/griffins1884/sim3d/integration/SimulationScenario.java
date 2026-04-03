package org.griffins1884.sim3d.integration;

import edu.wpi.first.math.geometry.Pose2d;

/** Repeatable simulation scenario definition for autonomous and regression checks. */
public record SimulationScenario(String name, String description, Pose2d startPose, Pose2d targetPose) {}
