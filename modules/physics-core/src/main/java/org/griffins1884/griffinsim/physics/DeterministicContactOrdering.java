package org.griffins1884.griffinsim.physics;

import java.util.Comparator;
import java.util.List;

public final class DeterministicContactOrdering {
  private static final Comparator<ContactConstraint> ORDER =
      Comparator.comparing(ContactConstraint::bodyId).thenComparing(ContactConstraint::contactId);

  private DeterministicContactOrdering() {}

  public static List<ContactConstraint> sort(List<ContactConstraint> contacts) {
    return List.copyOf((contacts == null ? List.<ContactConstraint>of() : contacts).stream().sorted(ORDER).toList());
  }
}
