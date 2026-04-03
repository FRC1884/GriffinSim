package org.griffins1884.griffinsim.physics;

public final class MaterialProfileRegistry {
  private MaterialProfileRegistry() {}

  public static MaterialProfile byName(String name) {
    if (name == null || name.isBlank()) {
      return MaterialProfiles.DEFAULT_BODY;
    }
    return switch (name) {
      case "default-body" -> MaterialProfiles.DEFAULT_BODY;
      case "default-field" -> MaterialProfiles.DEFAULT_FIELD;
      case "painted-steel" -> MaterialProfiles.PAINTED_STEEL;
      case "hdpe" -> MaterialProfiles.HDPE;
      case "carpet" -> MaterialProfiles.CARPET;
      case "foam-bump" -> MaterialProfiles.FOAM_BUMP;
      default -> throw new IllegalArgumentException("Unknown material profile: " + name);
    };
  }
}
