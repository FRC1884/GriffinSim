package org.griffins1884.griffinsim.frc;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public final class HalSimPwmCallbackRegistrar implements AutoCloseable {
  private final PwmCallbackSource callbackSource;
  private final QueuedHalSimBridge bridge;
  private final List<RegisteredCallback> registrations = new ArrayList<>();

  public HalSimPwmCallbackRegistrar(PwmCallbackSource callbackSource, QueuedHalSimBridge bridge) {
    this.callbackSource = Objects.requireNonNull(callbackSource);
    this.bridge = Objects.requireNonNull(bridge);
  }

  public void registerChannels(int channelCount) {
    for (int channel = 0; channel < channelCount; channel++) {
      final int currentChannel = channel;
      NotifyCallback callback = (String name, HALValue value) -> bridge.onPwmChanged(currentChannel, value.getDouble());
      int uid = callbackSource.registerSpeedCallback(channel, callback, true);
      registrations.add(new RegisteredCallback(channel, uid));
    }
  }

  @Override
  public void close() {
    for (RegisteredCallback registration : registrations) {
      callbackSource.cancelSpeedCallback(registration.channel(), registration.uid());
    }
    registrations.clear();
  }

  private record RegisteredCallback(int channel, int uid) {}
}
