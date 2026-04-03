package org.griffins1884.griffinsim.frc.scenario;

public record HolonomicPosePwmCommandPlan(
    PwmCommandTimeline xTimeline,
    PwmCommandTimeline yTimeline,
    PwmCommandTimeline thetaTimeline) {
  public HolonomicPosePwmCommandPlan {
    if (xTimeline == null || yTimeline == null || thetaTimeline == null) {
      throw new NullPointerException("timelines must be present");
    }
  }
}
