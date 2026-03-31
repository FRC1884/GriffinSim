# Versioning

GriffinSim uses a simple staged versioning model:

- `0.x.y-SNAPSHOT`: active development on unstable APIs
- `0.x.y`: tested milestone suitable for direct season-repo consumption
- `1.x.y`: stable public API for long-lived use across multiple robot projects

## Release Rules

1. Increment the patch version for backwards-compatible fixes and validation-only changes.
2. Increment the minor version for new public simulation APIs, new field/contact models, or new tooling hooks.
3. Increment the major version when public integration contracts change incompatibly.
4. Keep `-SNAPSHOT` on `main` until a milestone is intentionally cut.

## Current Guidance

- Until the 6DOF backend is native and no longer Maple-dependent at the core, releases should stay in the `0.x` range.
- Robot repos should pin a concrete GriffinSim version for their season branch instead of floating on `main`.
- Season repos should treat GriffinSim as a desktop simulation dependency only.
