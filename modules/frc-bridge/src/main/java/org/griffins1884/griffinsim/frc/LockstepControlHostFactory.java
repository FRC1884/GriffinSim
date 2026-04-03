package org.griffins1884.griffinsim.frc;

import java.util.Objects;

public final class LockstepControlHostFactory {
  private LockstepControlHostFactory() {}

  public static LockstepControlHost createLockstepHost(
      ControlHostConfig config,
      HalSimValueSink valueSink,
      RobotProgramLoop robotProgramLoop) {
    Objects.requireNonNull(config);
    QueuedHalSimBridge bridge = new QueuedHalSimBridge(config.queueCapacity(), valueSink);
    return new LockstepControlHost(config, new WpiSimHooksTimeController(), bridge, robotProgramLoop, bridge);
  }
}
