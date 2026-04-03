package org.griffins1884.griffinsim.frc;

public interface SimTimeController {
  void pause();

  void resume();

  void stepSeconds(double seconds);
}
