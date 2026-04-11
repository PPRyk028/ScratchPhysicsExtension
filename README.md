# Scratch Physics Extension Kit

A long-term scaffold for building one shared 3D physics core with multiple host adapters:

- `Gandi` normal remote extension entry
- `Gandi` approved extension entry
- `TurboWarp` unsandboxed extension entry

The project is dependency-free on purpose. You can build and serve it with plain Node.js while the engine API is still evolving.

The project direction is now:

- physics-first
- debug-render-only
- rigid convex bodies first
- cloth and soft bodies later

## Structure

```text
src/
  core/                 Temporary compatibility-probe core and math helpers
  physics/              PhysicsWorld, registries, math, and debug primitives
  shared/               Shared extension metadata and base block implementation
  platform/
    gandi/              Gandi remote/approved entries and host bridge
    turbowarp/          TurboWarp unsandboxed entry and host bridge
scripts/
  build.mjs             Tiny ESM bundler that emits browser-ready single-file builds
  dev-server.mjs        Local HTTP server for TurboWarp/Gandi testing
docs/
  architecture.md       Maintenance notes and extension-boundary rules
  physics-engine-design-v0.1.md
dist/                   Generated bundles
```

## Build

```powershell
node .\scripts\build.mjs
```

Outputs:

- `dist/turbowarp-unsandboxed.js`
- `dist/gandi-normal-remote.js`
- `dist/gandi-approved.js`

For GitHub Pages hosting:

```powershell
node .\scripts\build-pages.mjs
```

This copies the Gandi public test bundle to `docs/gandi-normal-remote.js` and writes a simple Pages landing page at `docs/index.html`.

Local load targets:

- TurboWarp unsandboxed: `http://localhost:8000/turbowarp-unsandboxed.js`
- Gandi custom-extension URL input: `http://localhost:8000/gandi-normal-remote.js`
- Gandi approved bundle: `http://localhost:8000/gandi-approved.js`

## Local development server

```powershell
node .\scripts\dev-server.mjs --watch
```

The server defaults to `http://localhost:8000/`, which matches TurboWarp's official unsandboxed local-development requirement.

## Bundler conventions

The custom bundler in `scripts/build.mjs` is intentionally tiny. For now, keep source modules within these rules:

- use relative imports only
- use named imports and named exports only
- keep import statements on one line
- avoid external npm dependencies until you intentionally replace the bundler

## Current world-layer block surface

The scaffold now exposes the first physics-oriented world API:

- `reset physics world`
- `set gravity`
- `create material`
- `set debug camera position`
- `create box rigid body`
- `create static box collider`
- `step physics world by ... seconds`
- `physics world summary`
- `rigid body ... summary`
- `collider ... summary`
- `material ... summary`
- `bodies at point ...`
- `colliders at point ...`
- `bodies in box ...`
- `colliders in box ...`
- `render debug frame`
- `debug frame summary`

The older scene-style blocks are still present as hidden compatibility aliases so the host-loading tests remain stable while the API migrates.

## Recommended next steps

The rigid pipeline now includes:

- persistent manifolds for box-box stacks
- angular velocity and local diagonal inertia data
- angular contact response in the rigid solver
- first-pass `GJK -> EPA -> feature-based manifold generation` for convex support-mapped pairs

Recommended next steps:

1. Improve the generic convex manifold reducer and clipping heuristics beyond the current first-pass feature sampling.
2. Add raycast, shape-cast, and sweep query entry points that reuse the support-mapped narrowphase.
3. Add early CCD/speculative contacts for fast rigid bodies.
4. Add joints on top of the thicker rigid pipeline.
5. Revisit box-box narrowphase so rotated box contacts can leave the AABB-era path.
6. Add cloth and soft-body systems only after the rigid pipeline is stable.
