package org.griffins1884.sim3d.integration;

import edu.wpi.first.math.geometry.Pose3d;
import org.griffins1884.sim3d.ChassisState3d;
import org.griffins1884.sim3d.SimImuSample;
import org.griffins1884.sim3d.SwerveTractionState;
import org.griffins1884.sim3d.TerrainContactSample;

/** Snapshot for visualization and telemetry publishers. */
public record VisualizationFrame(
    Pose3d robotPose,
    ChassisState3d chassisState3d,
    SimImuSample imuSample,
    TerrainContactSample terrainContactSample,
    SwerveTractionState tractionState,
    FieldMarkerSample[] fieldMarkers) {}
