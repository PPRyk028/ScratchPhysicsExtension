# Engine Roadmap

## Phase 0: Compatibility probe

Goal: prove that a shared extension surface can survive on both hosts.

- keep blocks simple
- verify registration models
- verify state mutation
- avoid renderer-heavy code until both hosts are stable

## Phase 1: Visual prototype

Goal: render one primitive with one camera.

- line-frame cube only
- one scene
- one camera
- one render path
- no physics yet

## Phase 2: Rendering foundation

Goal: establish a renderer contract that both hosts can implement.

- draw-command schema
- projection and clipping
- mesh buffers
- materials/colors
- lifecycle hooks for resize/reset/dispose

## Phase 3: Scene system

Goal: support larger projects without rewriting the public API.

- transforms
- parent/child hierarchy
- cameras
- lights or fake lighting
- mesh asset loading

## Phase 4: Physics prototype

Goal: decide what "physics engine" means in this ecosystem before promising full modern physics.

- define fixed timestep
- define collision scope: AABB only, SAT, or rigid body
- define required determinism level
- define performance budget on Scratch-like runtimes

## Phase 5: Physics engine implementation

Goal: add only the physics level that the host environments can actually sustain.

- broad phase
- narrow phase
- contact resolution
- gravity and constraints
- scratch-facing block API for forces, velocity, collision queries

## Phase 6: Performance and tooling

Goal: keep projects usable for end users.

- object pooling
- frame diagnostics
- debug overlay
- asset import helpers
- compatibility regression tests

## Non-goals for early phases

- full Unity-like editor
- arbitrary shader pipeline
- complete modern rigid-body stack from day one
- promising cross-host feature parity before the compatibility probe is complete
