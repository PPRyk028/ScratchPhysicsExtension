# Architecture Notes

## Goals

- Keep the 3D engine state independent from Scratch host APIs.
- Share block opcodes and argument IDs across Gandi and TurboWarp.
- Allow deeper renderer integration later without rewriting the public block surface.

## Layers

### `src/core`

Pure logic only.

- vector/matrix helpers
- scene state
- primitive creation
- debug frame planning

This layer must not read `window`, `Scratch`, `runtime`, or renderer internals.

### `src/shared`

Cross-host extension code.

- stable extension id
- block metadata
- argument coercion
- base extension class that maps Scratch block calls to the shared engine

If a change affects opcodes, argument IDs, or menu IDs, treat it as a compatibility change.

### `src/platform/gandi`

Gandi-specific integration.

- approved-extension bootstrap via `tempExt`
- runtime bridge
- future renderer hooks

### `src/platform/turbowarp`

TurboWarp-specific integration.

- unsandboxed bootstrap via `Scratch.extensions.register(...)`
- `Scratch.vm` / runtime / renderer bridge
- future stage-layer integration

## Build-system rule

The current build step uses a tiny custom ESM bundler to stay dependency-free during early engine work.

- keep imports relative
- keep imports/exports named
- if you outgrow these constraints, replace the bundler intentionally instead of working around it piecemeal

## Compatibility rules

- Keep `engine3d` as the stable extension id unless you intentionally break the ecosystem.
- Never rename opcodes after release without keeping backward-compat shims.
- Prefer adding new blocks over changing existing argument meaning.
- Keep host-only behavior behind adapter methods, not inside `src/core`.

## Rendering roadmap

The scaffold currently emits debug frame plans instead of pixels. A practical next milestone is:

1. Convert meshes into host-agnostic draw commands in `src/core/engine3d.js`.
2. Implement a renderer contract in both host bridges.
3. Add texture/material loading once both hosts share the same draw-command schema.
