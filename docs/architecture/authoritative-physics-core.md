# Authoritative Physics Core Notes

## Current implemented foundation

- immutable world state snapshots (`ImmutableWorldState`)
- canonical body ordering before every step
- canonical contact ordering before contact resolution
- deterministic step request contract with explicit next header and fixed `stepNanos`
- concrete rigid-body stepping scaffold in `DeterministicPhysicsWorld`
- simple contact correction that removes velocity into the contact normal

## What is still missing

- full 6DOF multibody joints
- broadphase/narrowphase collision generation
- rolling/torsional friction
- season/world geometry rewrite into the active physics pipeline
