# Compatibility Test Plan

This repository should treat host compatibility as a first-class milestone before engine complexity increases.

## Automated rounds

### Round 1: Build contract

- build both bundles
- confirm each target file exists
- confirm generated bundles are valid JavaScript

### Round 2: TurboWarp contract

- bundle throws if loaded without `unsandboxed`
- bundle calls `Scratch.extensions.register(...)`
- registered instance exposes the shared block contract
- blocks mutate shared scene state as expected

### Round 3: Gandi contract

- normal remote bundle calls `Scratch.extensions.register(...)`
- approved bundle populates `window.tempExt`
- `tempExt.Extension` can be instantiated with a runtime
- instance exposes the same shared block contract
- blocks mutate shared scene state as expected

## Manual editor checks

These should happen before rendering or physics work gets deep.

### TurboWarp

1. Run `node .\scripts\dev-server.mjs --watch`
2. Open TurboWarp editor
3. Load `http://localhost:8000/turbowarp-unsandboxed.js` as an extension
4. Confirm the extension category appears
5. Run:
   - `reset 3D scene`
   - `set camera position`
   - `add cube`
   - `render debug frame`
6. Verify:
   - `scene summary` changes
   - `last frame summary` changes
   - no immediate runtime crash

### Gandi

1. Run `node .\scripts\dev-server.mjs --watch`
2. Load `http://localhost:8000/gandi-normal-remote.js` into Gandi's custom-extension URL input
3. Confirm the extension category appears
4. Run the same minimal block sequence
5. Verify:
   - `scene summary` changes
   - `last frame summary` changes
   - no immediate runtime crash
6. If you are specifically testing the approved workflow, use `http://localhost:8000/gandi-approved.js` through the `?gext=` route documented by the Gandi custom-extension repository

## Exit criteria before real engine work

- both adapters pass automated tests
- both editors can load the test extension manually
- both editors can execute the same probe blocks without diverging behavior
- only after that should renderer integration become the next active milestone
