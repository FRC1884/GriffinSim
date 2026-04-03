package org.griffins1884.griffinsim.physics;

import java.util.List;

public interface ContactGenerator {
  List<ContactConstraint> generateContacts(ImmutableWorldState worldState);
}
