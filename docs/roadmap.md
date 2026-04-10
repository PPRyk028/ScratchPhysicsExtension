# Physics Roadmap

## Phase 0: Host compatibility probe

Goal: prove that one extension surface can load on both hosts.

- verify TurboWarp unsandboxed loading
- verify Gandi normal remote loading
- verify shared block registration and state updates

## Phase 1: Physics core skeleton

Goal: replace the temporary scene-only core with a real physics world.

- `PhysicsWorld`
- shape and body registries
- fixed-step timing
- debug primitive output contract

## Phase 2: Convex rigid-body MVP

Goal: establish the rigid-body foundation.

- boxes, spheres, capsules, convex hulls
- broadphase
- GJK/EPA-based narrowphase
- gravity and integration
- simple debug wireframes

## Phase 3: Stable rigid contact

Goal: make rigid bodies rest, stack, and slide reliably.

- persistent manifold
- warm starting
- Sequential Impulses / PGS
- friction and restitution
- sleeping

## Phase 4: Joints and queries

Goal: support useful gameplay structures and inspection tools.

- hinge, ball-socket, fixed, distance joints
- raycasts
- overlap tests
- early CCD path

## Phase 5: Cloth MVP

Goal: add deformables without derailing the rigid-body architecture.

- XPBD cloth
- stretch, shear, bend, tether constraints
- collision against rigid convex bodies
- cloth debug visualization

## Phase 6: Soft-body MVP

Goal: introduce volumetric deformables.

- tetrahedral soft bodies
- edge and volume constraints
- rigid/deformable collision
- soft-body diagnostics

## Phase 7: Coupling, performance, and tooling

Goal: make the project maintainable and scalable.

- stronger rigid/deformable coupling
- profiling
- replay and diagnostics
- serialization helpers
- compatibility regression tests

## Non-goals for early phases

- dynamic non-convex rigid bodies
- fluid simulation
- GPU-first physics
- photorealistic rendering
- full editor tooling before the core solver is stable
