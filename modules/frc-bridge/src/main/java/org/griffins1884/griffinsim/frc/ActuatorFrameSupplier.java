package org.griffins1884.griffinsim.frc;

import org.griffins1884.griffinsim.contracts.ActuatorFrame;
import org.griffins1884.griffinsim.contracts.FrameHeader;

public interface ActuatorFrameSupplier {
  ActuatorFrame capture(FrameHeader header);
}
