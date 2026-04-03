package org.griffins1884.griffinsim.frc.scenario;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class HolonomicPoseFollowerCompilerTest {
  @Test
  void compilesPoseWaypointsIntoXYZThetaPwmPlan() {
    HolonomicPosePwmCommandPlan plan =
        HolonomicPoseFollowerCompiler.compile(
            new HolonomicPoseFollowerSpec(
                List.of(
                    new PoseWaypoint2d(0, 0.0, 0.0, 0.0, 0.2),
                    new PoseWaypoint2d(1, 1.0, 0.5, 1.0, 0.8),
                    new PoseWaypoint2d(3, -0.5, -0.25, -0.5, 0.4)),
                0.0,
                0.0,
                0.0,
                1.2,
                0.8,
                0.9,
                0.2,
                0.1,
                0.01,
                0.01));

    assertTrue(plan.xTimeline().valueAtTick(1) > 0.0);
    assertTrue(plan.yTimeline().valueAtTick(1) > 0.0);
    assertTrue(plan.thetaTimeline().valueAtTick(1) > 0.0);
    assertEquals(0.0, plan.thetaTimeline().valueAtTick(4));
  }
}
