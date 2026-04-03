# Release Checklist

## Determinism gate

- [ ] `./gradlew test` passes
- [ ] example scenario files load and run deterministically
- [ ] replay outputs are stable across repeated scenario-runner executions

## Documentation gate

- [ ] README reflects current controller/scenario capabilities
- [ ] `docs/architecture/scenario-spec.md` is updated for supported keys
- [ ] `docs/architecture/controller-layers.md` matches implemented controller layers
- [ ] milestone notes summarize what is and is not production-ready

## Packaging gate

- [ ] scenario-runner CLI builds and runs
- [ ] replay-diff CLI builds and runs
- [ ] no active build depends on `legacy/griffinsim-v0/`

## Known limitations to disclose

- simplified contact solver
- no packaged native HALSIM extension artifact
- controller layers compile intent into PWM schedules rather than full chassis controllers
