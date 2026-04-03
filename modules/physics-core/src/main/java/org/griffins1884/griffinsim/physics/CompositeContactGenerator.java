package org.griffins1884.griffinsim.physics;

import java.util.List;

public final class CompositeContactGenerator implements ContactGenerator {
  private final List<ContactGenerator> generators;

  public CompositeContactGenerator(List<ContactGenerator> generators) {
    this.generators = List.copyOf(generators == null ? List.of() : generators);
  }

  @Override
  public List<ContactConstraint> generateContacts(ImmutableWorldState worldState) {
    return generators.stream()
        .flatMap(generator -> generator.generateContacts(worldState).stream())
        .sorted(java.util.Comparator.comparing(ContactConstraint::bodyId).thenComparing(ContactConstraint::contactId))
        .toList();
  }
}
