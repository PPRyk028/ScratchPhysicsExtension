# Physics Engine Design v0.1

Status: Draft  
Date: 2026-04-10

## Purpose

This project is a cross-host 3D physics extension for:

- Gandi normal remote extensions
- Gandi approved extensions
- TurboWarp unsandboxed extensions

It is **not** a general-purpose 3D rendering engine. Rendering exists only to support physics debugging:

- wireframe bodies
- AABBs
- contact points
- contact normals
- joint frames
- cloth and soft-body diagnostic meshes

The long-term product goal is:

- arbitrary convex polyhedron rigid bodies
- cloth
- soft bodies

The engine should feel modern in capability, but it must remain realistic for a JavaScript CPU-only runtime inside Scratch-like hosts.

## Product Definition

### What this engine is

- a deterministic-enough simulation core shared by Gandi and TurboWarp
- a physics-first runtime with debug visualization
- an extension-oriented API rather than a scene editor
- a host-agnostic core with thin platform adapters

### What this engine is not

- not a full game engine
- not a polished renderer
- not a GPU-accelerated PhysX clone
- not a FEM-first scientific simulator
- not a promise of feature parity with Unity, Unreal, or Omniverse

## Design Constraints

### Runtime constraints

- JavaScript only in early phases
- CPU simulation only
- must run inside browser-hosted extension environments
- must share one physics core across Gandi and TurboWarp
- must tolerate different host privilege levels

### Product constraints

- keep public block opcodes stable once released
- avoid host-specific physics behavior
- support debug-first workflows before visual polish
- prioritize simulation correctness over broad feature count

### Performance assumptions

- default simulation target: 60 Hz fixed step
- support lower-rate stepping through substep controls
- accept that cloth and soft bodies will need stricter object-count budgets than rigid bodies

## Core Scope

### In scope

- rigid body simulation for arbitrary convex polyhedra
- static colliders for planes, boxes, and convex hulls
- broadphase, narrowphase, manifolds, and impulse solving
- joints and constraints
- cloth based on triangle meshes
- soft bodies based on tetrahedral volume meshes
- debug rendering data generation
- serialization-ready world state

### Out of scope for early phases

- dynamic non-convex rigid bodies
- fluids
- destruction/fracture
- GPU pipelines
- high-order FEM
- photorealistic rendering

### Deferred features

- convex decomposition tooling
- robust self-collision for cloth at production scale
- advanced vehicle/character systems
- fully deterministic lockstep across every browser and platform

## Architecture Overview

The engine should keep simulation code independent from extension-host code.

### Layer 1: Host adapters

Location target:

- `src/platform/turbowarp`
- `src/platform/gandi`

Responsibilities:

- extension registration
- argument coercion at the block boundary
- host capability detection
- debug draw presentation hooks
- lifecycle integration with the editor/runtime

These adapters must not contain physics logic.

### Layer 2: Shared extension surface

Location target:

- `src/shared`

Responsibilities:

- stable block metadata
- block-to-core command mapping
- versioned API surface
- shared diagnostics/reporters

This layer should expose the physics engine without leaking host internals.

### Layer 3: Physics core

Recommended location target:

- `src/physics`

Submodules:

- `world/`
- `math/`
- `geometry/`
- `collision/`
- `manifold/`
- `rigid/`
- `constraints/`
- `deformable/cloth/`
- `deformable/softbody/`
- `debug/`
- `serialization/`

This layer owns all simulation behavior.

## Simulation Architecture

### World Object

The central runtime object should be `PhysicsWorld`.

Responsibilities:

- own bodies, shapes, colliders, joints, cloth, and soft bodies
- maintain fixed-step timing
- maintain broadphase structures
- manage persistent pair caches and manifolds
- expose debug draw data for the host

### World stepping model

The engine should use a fixed-step accumulator:

1. accumulate real time
2. consume `dt_fixed` slices
3. run a bounded number of substeps
4. expose interpolation-friendly state if needed later

Default policy:

- `dt_fixed = 1 / 60`
- configurable max substeps
- deterministic stepping order for stable behavior

## Rigid Body Pipeline

The rigid-body path is the foundation for the entire project. Cloth and soft-body collision should build on this collision stack rather than inventing a separate world representation.

### Shape representation

Dynamic rigid bodies should initially support:

- convex hulls
- boxes
- spheres
- capsules

Boxes, spheres, and capsules can be specialized convex shapes. Arbitrary convex polyhedra should use:

- explicit face/edge/vertex data for manifold generation
- support mapping for GJK/EPA
- precomputed mass properties where possible

Recommended internal representation:

- local vertices
- plane equations per face
- adjacency information
- support function cache hints
- mass and inertia tensors in local space

### Broadphase

Chosen baseline:

- Dynamic AABB Tree

Reason:

- good fit for sparse dynamic scenes
- widely used in real-time physics
- works for rigid bodies, cloth particles, and soft-body proxy nodes

Possible later optimization:

- sweep-and-prune for large, mostly axis-stable scenes

### Narrowphase

Chosen baseline for convex rigid bodies:

- GJK for distance/intersection
- EPA for penetration depth and normal when intersecting
- feature-based clipping/contact reduction for manifold generation

Rationale:

- scales naturally to arbitrary convex polyhedra
- keeps the rigid-body story centered on convex support-mapped shapes
- aligns with the engine's long-term target better than a box-only SAT architecture

### Contact manifold

This project **will use persistent contact manifolds** for rigid bodies.

Manifold goals:

- stable resting contact
- better friction behavior
- warm starting for the solver
- reduced contact jitter between frames

Recommended per-pair cache:

- pair key
- contact normal
- up to 4 contact points
- local anchor on body A
- local anchor on body B
- accumulated normal impulse
- accumulated tangent impulses
- separation or penetration estimate
- feature id
- lifetime

Policy:

- keep at most 4 points per manifold
- refresh from current transforms each step
- prune stale points
- preserve impulse history when feature matching succeeds

### Solver

Chosen rigid solver family:

- Sequential Impulses / PGS

Details:

- velocity-level iterative solver
- warm starting enabled
- accumulated impulse clamping
- 2 tangent friction directions
- solver iterations configurable per world

Why this choice:

- proven in real-time game physics
- good engineering fit for rigid-body contact and joint stacks
- practical in JavaScript
- easier to debug incrementally than more ambitious global solvers

### Stabilization strategy

Initial plan:

- Baumgarte-style bias for penetration correction
- warm starting
- optional split impulse path after the rigid MVP if penetration correction injects too much energy

This should be treated as tunable policy, not hard-coded forever.

### Continuous collision detection

CCD should not block the first rigid-body MVP.

Plan:

- MVP: discrete collision only
- next phase: speculative contacts for high-speed bodies
- later phase: conservative advancement / shape cast for selected convex bodies

### Sleeping

Rigid bodies should support sleeping after the manifold and solver are stable enough.

Requirements:

- linear and angular thresholds
- time under threshold
- wake on contact/joint/force changes

## Joint and Constraint System

Rigid constraints should be solved in the same Sequential Impulse framework as contacts.

Initial target joints:

- fixed
- point-to-point / ball socket
- hinge
- distance

Deferred joints:

- slider
- 6DoF
- motors and limits beyond the basics

## Cloth Architecture

Cloth should not reuse the rigid-body impulse solver as its primary simulation method.

Chosen cloth solver family:

- XPBD

Representation:

- particle positions
- particle inverse masses
- triangle mesh topology
- edge and bend constraints

Constraint set:

- stretch
- shear
- bend
- tether/attachment
- optional area preservation later

Collision:

- cloth particle vs convex colliders in early phases
- optional edge/triangle refinements later
- self-collision deferred until the non-self-colliding path is stable

Why XPBD:

- robust for cloth-like constraints
- better fit for position-based deformables than rigid-body impulse solving
- predictable iteration and compliance controls

## Soft-Body Architecture

Soft bodies should also follow XPBD rather than a pure rigid-body solver.

Representation:

- tetrahedral volume mesh
- particle/node masses
- edge constraints
- volume constraints
- optional shape-matching clusters later

Constraint set:

- edge length preservation
- tetra volume preservation
- boundary attachments
- damping

Collision:

- node/feature interaction with rigid convex colliders first
- full self-collision deferred

Important scope note:

- the initial soft-body target is game-style deformables
- it is not a high-precision FEM target

## Rigid and Deformable Coupling

Final architecture target:

- rigid bodies solved with Sequential Impulses
- cloth and soft bodies solved with XPBD
- collision coupling shared through the same collider world

Practical rollout:

1. one-way collisions from rigid world to deformables
2. stable contact constraints against convex colliders
3. two-way momentum transfer only after the one-way path is stable

This sequencing reduces risk and keeps the rigid-body core from being delayed by deformable coupling complexity.

## Debug Rendering Contract

Rendering is subordinate to physics. The physics core should emit debug primitives instead of drawing directly.

Recommended debug primitive types:

- line list
- point list
- triangle wireframe list
- text labels only if the host can support them cheaply

Primary overlays:

- collider wireframes
- AABBs
- contact points
- contact normals
- center of mass
- velocity vectors
- joint frames
- cloth edges
- soft-body tetra or surface edges

Host adapters may visualize these primitives differently, but the debug schema should be shared.

## Data Model

The engine should assign stable ids to all simulation objects.

Core object categories:

- `Shape`
- `Collider`
- `RigidBody`
- `Joint`
- `Cloth`
- `SoftBody`
- `Material`
- `ContactPair`
- `Manifold`

Recommended world state partitions:

- immutable shape definitions
- mutable body state
- solver scratch buffers
- debug frame output

## Block API Strategy

The extension API should be layered and versioned.

### Guiding rules

- blocks should describe physics intent, not host behavior
- block ids and argument ids must stay stable once released
- avoid exposing low-level solver internals too early
- support simple presets before advanced expert tuning

### Early block groups

- world setup
- shape creation
- rigid body creation
- force/impulse application
- simulation stepping
- debug draw toggles
- queries: raycasts, overlaps, contacts

### Deferred block groups

- cloth authoring
- soft-body authoring
- detailed constraint tuning
- serialization/import helpers

Separate API drafting should follow after the rigid pipeline design is accepted.

## Determinism Policy

The project should aim for **stable gameplay determinism within practical limits**, not bit-perfect cross-browser replay at all costs.

Policy:

- fixed simulation step
- deterministic iteration order
- stable object ids
- minimized unordered-map dependence
- avoid hidden host timing dependence

Non-goal:

- exact floating-point identity across every browser engine and device

## Milestones

### M0: Host compatibility probe

Status:

- completed

Exit criteria:

- TurboWarp loads the unsandboxed bundle
- Gandi loads the normal remote bundle
- shared block surface is identical across adapters

### M1: Physics core skeleton

Goals:

- introduce `src/physics`
- define `PhysicsWorld`
- define body, collider, and shape registries
- define debug primitive output format

Exit criteria:

- no real collision yet
- world creation and stepping API exists
- adapters can query debug state from the physics core

### M2: Convex rigid-body MVP

Goals:

- boxes, spheres, capsules, convex hulls
- gravity
- integration
- broadphase
- discrete narrowphase
- single-contact solving sufficient for simple demos

Exit criteria:

- simple falling and bouncing scenes work
- rigid bodies are controllable through blocks
- debug wireframes are visible

### M3: Persistent manifold and stable stacks

Goals:

- persistent manifold cache
- warm starting
- multi-point frictional contact
- sleeping

Exit criteria:

- box stacks are acceptably stable
- convex hulls rest without severe jitter
- contact debug overlays are meaningful

### M4: Joint system and scene queries

Goals:

- fixed, hinge, ball-socket, distance joints
- raycasts and overlap queries
- early speculative CCD path

Exit criteria:

- common rigid-body gameplay structures are possible
- joints and contacts coexist stably

### M5: Cloth MVP

Goals:

- XPBD cloth
- attachments
- rigid-body collision
- cloth debug rendering

Exit criteria:

- hanging cloth and flag-like scenes work
- collision against rigid convex bodies is stable enough for demos

### M6: Soft-body MVP

Goals:

- tetrahedral soft body representation
- edge and volume constraints
- rigid-body collision

Exit criteria:

- basic deformable volume scenes run
- diagnostics expose constraint and collision state

### M7: Coupling, optimization, and tooling

Goals:

- improve rigid/deformable coupling
- profiler counters
- replay/debug tools
- serialization and import/export helpers
- regression scenes for both hosts

Exit criteria:

- project can support sustained iteration without architecture rewrites

## Risks

### Highest technical risks

- robust manifold generation for arbitrary convex polyhedra
- performance cost of deformable collision in JavaScript
- stable tetrahedral soft-body authoring pipeline
- block API complexity for advanced physics features

### Mitigations

- rigid body pipeline first
- debug-first rendering
- XPBD for deformables instead of forcing one solver for everything
- explicit milestone gates before adding new simulation families

## Immediate Next Step

The project now has the first rigid-world scaffold in code:

- `PhysicsWorld`
- shape registry
- rigid body registry
- collider registry
- material registry
- shared debug primitive schema
- broadphase proxy generation
- sweep-style broadphase pair generation
- box-box multi-point manifold path
- angular velocity and local inertia integration
- angular contact response in the solver
- first-pass `GJK -> EPA -> feature-based manifold generation` for convex support-mapped pairs

The next implementation phase should continue the rigid collision stack, not return to rendering polish.

Immediate coding tasks:

- improve the generic convex manifold reducer and clipping heuristics beyond the current first-pass feature sampling
- add raycast, shape-cast, and sweep query entry points that can share the same support-mapped narrowphase
- add early CCD/speculative contacts for fast rigid bodies
- revisit rotated box-box contact generation so it can leave the AABB-era path
