package org.griffins1884.griffinsim.physics;

import java.util.List;
import org.griffins1884.griffinsim.contracts.ActuatorFrame;

public interface ActuatorCommandMapper {
  List<BodyCommand> map(ActuatorFrame frame, ImmutableWorldState currentWorld);
}
