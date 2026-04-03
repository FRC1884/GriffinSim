package org.griffins1884.griffinsim.frc;

import edu.wpi.first.hal.simulation.NotifyCallback;
import edu.wpi.first.hal.simulation.PWMDataJNI;

public final class JniPwmCallbackSource implements PwmCallbackSource {
  @Override
  public int registerSpeedCallback(int channel, NotifyCallback callback, boolean initialNotify) {
    return PWMDataJNI.registerSpeedCallback(channel, callback, initialNotify);
  }

  @Override
  public void cancelSpeedCallback(int channel, int uid) {
    PWMDataJNI.cancelSpeedCallback(channel, uid);
  }
}
