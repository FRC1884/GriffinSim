package org.griffins1884.griffinsim.physics;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import org.griffins1884.griffinsim.contracts.RigidBodyState;

public final class AxisAlignedObstacleContactGenerator implements ContactGenerator {
  private final List<AxisAlignedBoxObstacle> obstacles;
  private final Map<String, MaterialProfile> bodyMaterials;

  public AxisAlignedObstacleContactGenerator(List<AxisAlignedBoxObstacle> obstacles) {
    this(obstacles, Map.of());
  }

  public AxisAlignedObstacleContactGenerator(List<AxisAlignedBoxObstacle> obstacles, Map<String, MaterialProfile> bodyMaterials) {
    this.obstacles = List.copyOf(obstacles == null ? List.of() : obstacles);
    this.bodyMaterials = Map.copyOf(bodyMaterials == null ? Map.of() : bodyMaterials);
  }

  @Override
  public List<ContactConstraint> generateContacts(ImmutableWorldState worldState) {
    List<ContactConstraint> contacts = new ArrayList<>();
    for (RigidBodyState body : worldState.bodies()) {
      for (AxisAlignedBoxObstacle obstacle : obstacles) {
        ContactConstraint contact = contactFor(body, obstacle, bodyMaterials.getOrDefault(body.bodyId(), MaterialProfiles.DEFAULT_BODY));
        if (contact != null) {
          contacts.add(contact);
        }
      }
    }
    return contacts.stream()
        .sorted(Comparator.comparing(ContactConstraint::bodyId).thenComparing(ContactConstraint::contactId))
        .toList();
  }

  private static ContactConstraint contactFor(RigidBodyState body, AxisAlignedBoxObstacle obstacle, MaterialProfile bodyMaterial) {
    if (body.x() < obstacle.minX() || body.x() > obstacle.maxX()) {
      return null;
    }
    if (body.y() < obstacle.minY() || body.y() > obstacle.maxY()) {
      return null;
    }
    if (body.z() < obstacle.minZ() || body.z() > obstacle.maxZ()) {
      return null;
    }

    double leftPen = body.x() - obstacle.minX();
    double rightPen = obstacle.maxX() - body.x();
    double backPen = body.y() - obstacle.minY();
    double frontPen = obstacle.maxY() - body.y();
    double downPen = body.z() - obstacle.minZ();
    double upPen = obstacle.maxZ() - body.z();

    double minPen = leftPen;
    double nx = -1.0;
    double ny = 0.0;
    double nz = 0.0;
    if (rightPen < minPen) { minPen = rightPen; nx = 1.0; ny = 0.0; nz = 0.0; }
    if (backPen < minPen) { minPen = backPen; nx = 0.0; ny = -1.0; nz = 0.0; }
    if (frontPen < minPen) { minPen = frontPen; nx = 0.0; ny = 1.0; nz = 0.0; }
    if (downPen < minPen) { minPen = downPen; nx = 0.0; ny = 0.0; nz = -1.0; }
    if (upPen < minPen) { minPen = upPen; nx = 0.0; ny = 0.0; nz = 1.0; }

    return new ContactConstraint(
        body.bodyId(),
        obstacle.obstacleId() + ":box",
        nx,
        ny,
        nz,
        minPen,
        MaterialResolver.combinedFriction(bodyMaterial, obstacle.materialProfile()),
        MaterialResolver.combinedRestitution(bodyMaterial, obstacle.materialProfile()));
  }
}
