package org.griffins1884.griffinsim.frc;

public final class NoOpSimTimeController implements SimTimeController {
  @Override
  public void pause() {}

  @Override
  public void resume() {}

  @Override
  public void stepSeconds(double seconds) {}
}
