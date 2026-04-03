package org.griffins1884.griffinsim.physics;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import org.griffins1884.griffinsim.contracts.RigidBodyState;

public final class PairwiseBodyContactGenerator implements ContactGenerator {
  private final Map<String, BodyCollisionProfile> profiles;

  public PairwiseBodyContactGenerator(List<BodyCollisionProfile> profiles) {
    this.profiles =
        (profiles == null ? List.<BodyCollisionProfile>of() : profiles).stream()
            .collect(java.util.stream.Collectors.toUnmodifiableMap(BodyCollisionProfile::bodyId, profile -> profile));
  }

  @Override
  public List<ContactConstraint> generateContacts(ImmutableWorldState worldState) {
    List<RigidBodyState> bodies = worldState.bodies();
    List<ContactConstraint> contacts = new ArrayList<>();
    for (int i = 0; i < bodies.size(); i++) {
      for (int j = i + 1; j < bodies.size(); j++) {
        RigidBodyState a = bodies.get(i);
        RigidBodyState b = bodies.get(j);
        BodyCollisionProfile profileA = profiles.get(a.bodyId());
        BodyCollisionProfile profileB = profiles.get(b.bodyId());
        if (profileA == null || profileB == null) {
          continue;
        }
        addPairContacts(contacts, a, profileA, b, profileB);
      }
    }
    return contacts.stream()
        .sorted(Comparator.comparing(ContactConstraint::bodyId).thenComparing(ContactConstraint::contactId))
        .toList();
  }

  private static void addPairContacts(
      List<ContactConstraint> contacts,
      RigidBodyState a,
      BodyCollisionProfile profileA,
      RigidBodyState b,
      BodyCollisionProfile profileB) {
    double overlapX =
        (profileA.halfExtentX() + profileB.halfExtentX()) - Math.abs(a.x() - b.x());
    double overlapY =
        (profileA.halfExtentY() + profileB.halfExtentY()) - Math.abs(a.y() - b.y());
    double overlapZ =
        (profileA.halfExtentZ() + profileB.halfExtentZ()) - Math.abs(a.z() - b.z());
    if (overlapX <= 0.0 || overlapY <= 0.0 || overlapZ <= 0.0) {
      return;
    }

    double penetration = overlapX;
    double nx = a.x() <= b.x() ? -1.0 : 1.0;
    double ny = 0.0;
    double nz = 0.0;
    if (overlapY < penetration) {
      penetration = overlapY;
      nx = 0.0;
      ny = a.y() <= b.y() ? -1.0 : 1.0;
      nz = 0.0;
    }
    if (overlapZ < penetration) {
      penetration = overlapZ;
      nx = 0.0;
      ny = 0.0;
      nz = a.z() <= b.z() ? -1.0 : 1.0;
    }

    double friction = MaterialResolver.combinedFriction(profileA.materialProfile(), profileB.materialProfile());
    double restitution = MaterialResolver.combinedRestitution(profileA.materialProfile(), profileB.materialProfile());
    double rolling = MaterialResolver.combinedRollingFriction(profileA.materialProfile(), profileB.materialProfile());
    double torsional = MaterialResolver.combinedTorsionalFriction(profileA.materialProfile(), profileB.materialProfile());

    contacts.add(new ContactConstraint(a.bodyId(), a.bodyId() + ":vs:" + b.bodyId(), nx, ny, nz, penetration * 0.5, friction, restitution, rolling, torsional));
    contacts.add(new ContactConstraint(b.bodyId(), b.bodyId() + ":vs:" + a.bodyId(), -nx, -ny, -nz, penetration * 0.5, friction, restitution, rolling, torsional));
  }
}
