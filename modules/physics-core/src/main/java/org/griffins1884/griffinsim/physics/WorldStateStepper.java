package org.griffins1884.griffinsim.physics;

public interface WorldStateStepper {
  ImmutableWorldState step(ImmutableWorldState current, long stepNanos);
}
