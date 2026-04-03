package org.griffins1884.griffinsim.frc;

import edu.wpi.first.hal.simulation.NotifyCallback;

public interface PwmCallbackSource {
  int registerSpeedCallback(int channel, NotifyCallback callback, boolean initialNotify);

  void cancelSpeedCallback(int channel, int uid);
}
