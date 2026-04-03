package org.griffins1884.griffinsim.frc;

import edu.wpi.first.wpilibj.simulation.SimHooks;

public final class WpiSimHooksTimeController implements SimTimeController {
  @Override
  public void pause() {
    SimHooks.pauseTiming();
  }

  @Override
  public void resume() {
    SimHooks.resumeTiming();
  }

  @Override
  public void stepSeconds(double seconds) {
    SimHooks.stepTiming(seconds);
  }
}
