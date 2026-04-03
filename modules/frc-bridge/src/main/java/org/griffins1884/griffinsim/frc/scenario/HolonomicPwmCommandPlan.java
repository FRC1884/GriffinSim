package org.griffins1884.griffinsim.frc.scenario;

public record HolonomicPwmCommandPlan(PwmCommandTimeline xTimeline, PwmCommandTimeline yTimeline) {
  public HolonomicPwmCommandPlan {
    if (xTimeline == null || yTimeline == null) {
      throw new NullPointerException("timelines must be present");
    }
  }
}
