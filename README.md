# Scratch 3D Extension Kit

A long-term scaffold for building one shared 3D engine core with multiple host adapters:

- `Gandi` normal remote extension entry
- `Gandi` approved extension entry
- `TurboWarp` unsandboxed extension entry

The project is dependency-free on purpose. You can build and serve it with plain Node.js while the engine API is still evolving.

## Structure

```text
src/
  core/                 Host-agnostic 3D engine state and math helpers
  shared/               Shared extension metadata and base block implementation
  platform/
    gandi/              Gandi remote/approved entries and host bridge
    turbowarp/          TurboWarp unsandboxed entry and host bridge
scripts/
  build.mjs             Tiny ESM bundler that emits browser-ready single-file builds
  dev-server.mjs        Local HTTP server for TurboWarp/Gandi testing
docs/
  architecture.md       Maintenance notes and extension-boundary rules
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

## Current block surface

The scaffold intentionally keeps the first API small and stable:

- `reset 3D scene`
- `set camera position x: y: z:`
- `add cube [ID] at x: y: z: size:`
- `render debug frame`
- `scene summary`
- `last frame summary`
- `host summary`

These blocks already share one engine core across both adapters. The rendering layer is stubbed behind a host bridge so you can later add renderer integration without rewriting the block API.

## Recommended next steps

1. Extend `src/core/engine3d.js` with camera rotation, materials, mesh loading, and draw-command generation.
2. Add renderer-backed drawing inside `src/platform/turbowarp/host.js`.
3. Port the same rendering contract into `src/platform/gandi/host.js`.
4. Keep opcodes stable once projects start depending on them.
