package org.griffins1884.griffinsim.physics.seasons.rebuilt2026;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.griffins1884.griffinsim.contracts.RigidBodyState;
import org.griffins1884.griffinsim.physics.ContactConstraint;
import org.griffins1884.griffinsim.physics.ContactGenerator;
import org.griffins1884.griffinsim.physics.ImmutableWorldState;
import org.griffins1884.griffinsim.physics.MaterialProfile;
import org.griffins1884.griffinsim.physics.MaterialProfiles;
import org.griffins1884.griffinsim.physics.MaterialResolver;

public final class Rebuilt2026TerrainContactGenerator implements ContactGenerator {
  private static final double BUMP_PROFILE_DEPTH_METERS = Rebuilt2026FieldConstants.inchesToMeters(48.93);
  private static final double BUMP_SPAN_METERS = Rebuilt2026FieldConstants.inchesToMeters(73.00);
  private static final double BUMP_CENTER_OFFSET_Y_METERS = Rebuilt2026FieldConstants.inchesToMeters(60.00);
  private static final double BUMP_HEIGHT_METERS = Rebuilt2026FieldConstants.inchesToMeters(6.513);
  private static final double BUMP_RAMP_FRACTION = 24.47 / 48.93;

  private final Map<String, Double> bodyHeights;
  private final Map<String, MaterialProfile> bodyMaterials;

  public Rebuilt2026TerrainContactGenerator(List<BodyHeightProfile> bodyHeights) {
    this(bodyHeights, Map.of());
  }

  public Rebuilt2026TerrainContactGenerator(List<BodyHeightProfile> bodyHeights, Map<String, MaterialProfile> bodyMaterials) {
    this.bodyHeights =
        (bodyHeights == null ? List.<BodyHeightProfile>of() : bodyHeights).stream()
            .collect(Collectors.toUnmodifiableMap(BodyHeightProfile::bodyId, BodyHeightProfile::heightMeters));
    this.bodyMaterials = Map.copyOf(bodyMaterials == null ? Map.of() : bodyMaterials);
  }

  @Override
  public List<ContactConstraint> generateContacts(ImmutableWorldState worldState) {
    return worldState.bodies().stream()
        .flatMap(body -> contactsFor(body).stream())
        .sorted(java.util.Comparator.comparing(ContactConstraint::bodyId).thenComparing(ContactConstraint::contactId))
        .toList();
  }

  private List<ContactConstraint> contactsFor(RigidBodyState body) {
    java.util.ArrayList<ContactConstraint> contacts = new java.util.ArrayList<>();
    MaterialProfile bodyMaterial = bodyMaterials.getOrDefault(body.bodyId(), MaterialProfiles.DEFAULT_BODY);
    double terrainHeight = terrainHeightAt(body.x(), body.y());
    if (body.z() < terrainHeight) {
      contacts.add(new ContactConstraint(body.bodyId(), body.bodyId() + ":terrain", 0.0, 0.0, 1.0, terrainHeight - body.z(), MaterialResolver.combinedFriction(bodyMaterial, MaterialProfiles.FOAM_BUMP), MaterialResolver.combinedRestitution(bodyMaterial, MaterialProfiles.FOAM_BUMP)));
    }
    double clearanceHeight = trenchClearanceAt(body.x(), body.y());
    double bodyHeight = bodyHeights.getOrDefault(body.bodyId(), 0.9);
    if (clearanceHeight > 0.0 && body.z() + bodyHeight > clearanceHeight) {
      contacts.add(new ContactConstraint(body.bodyId(), body.bodyId() + ":trench-ceiling", 0.0, 0.0, -1.0, (body.z() + bodyHeight) - clearanceHeight, MaterialResolver.combinedFriction(bodyMaterial, MaterialProfiles.HDPE), MaterialResolver.combinedRestitution(bodyMaterial, MaterialProfiles.HDPE)));
    }
    return List.copyOf(contacts);
  }

  double terrainHeightAt(double x, double y) {
    return Math.max(
        Math.max(profileHeight(x, y, Rebuilt2026FieldConstants.HUB_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS + BUMP_CENTER_OFFSET_Y_METERS),
                 profileHeight(x, y, Rebuilt2026FieldConstants.HUB_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS - BUMP_CENTER_OFFSET_Y_METERS)),
        Math.max(profileHeight(x, y, Rebuilt2026FieldConstants.HUB_CENTER_X_RED_METERS, Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS + BUMP_CENTER_OFFSET_Y_METERS),
                 profileHeight(x, y, Rebuilt2026FieldConstants.HUB_CENTER_X_RED_METERS, Rebuilt2026FieldConstants.HUB_CENTER_Y_METERS - BUMP_CENTER_OFFSET_Y_METERS)));
  }

  double trenchClearanceAt(double x, double y) {
    if (inTrenchOpen(x, y, Rebuilt2026FieldConstants.TRENCH_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.TRENCH_UPPER_CENTER_Y_METERS)
        || inTrenchOpen(x, y, Rebuilt2026FieldConstants.TRENCH_CENTER_X_BLUE_METERS, Rebuilt2026FieldConstants.TRENCH_LOWER_CENTER_Y_METERS)
        || inTrenchOpen(x, y, Rebuilt2026FieldConstants.TRENCH_CENTER_X_RED_METERS, Rebuilt2026FieldConstants.TRENCH_UPPER_CENTER_Y_METERS)
        || inTrenchOpen(x, y, Rebuilt2026FieldConstants.TRENCH_CENTER_X_RED_METERS, Rebuilt2026FieldConstants.TRENCH_LOWER_CENTER_Y_METERS)) {
      return Rebuilt2026FieldConstants.TRENCH_OPENING_HEIGHT_METERS;
    }
    return -1.0;
  }

  private static boolean inTrenchOpen(double x, double y, double centerX, double centerY) {
    double halfDepth = Rebuilt2026FieldConstants.TRENCH_FOOTPRINT_DEPTH_METERS * 0.5;
    double halfWidth = Rebuilt2026FieldConstants.TRENCH_FOOTPRINT_WIDTH_METERS * 0.5;
    return x >= centerX - halfDepth && x <= centerX + halfDepth && y >= centerY - halfWidth && y <= centerY + halfWidth;
  }

  private static double profileHeight(double x, double y, double centerX, double centerY) {
    double halfDepth = BUMP_PROFILE_DEPTH_METERS * 0.5;
    double halfWidth = BUMP_SPAN_METERS * 0.5;
    if (x < centerX - halfDepth || x > centerX + halfDepth || y < centerY - halfWidth || y > centerY + halfWidth) {
      return 0.0;
    }
    double progress = (x - (centerX - halfDepth)) / BUMP_PROFILE_DEPTH_METERS;
    double rampFraction = Math.min(BUMP_RAMP_FRACTION, 0.5);
    if (progress < rampFraction) {
      return BUMP_HEIGHT_METERS * (progress / rampFraction);
    }
    if (progress > 1.0 - rampFraction) {
      return BUMP_HEIGHT_METERS * ((1.0 - progress) / rampFraction);
    }
    return BUMP_HEIGHT_METERS;
  }
}
