import test from 'node:test';
import assert from 'node:assert/strict';
import { DEBUG_FRAME_SCHEMA_VERSION, DEBUG_PRIMITIVE_TYPES, PhysicsWorld, rotateVec3ByQuat } from '../src/physics/index.js';

test('PhysicsWorld creates box bodies with matching shape, body, collider, and default material records', () => {
  const world = new PhysicsWorld();
  const created = world.createBoxBody({
    id: 'crate',
    position: { x: 1, y: 2, z: 3 },
    size: 4
  });

  assert.equal(created.shape.id, 'crate:shape');
  assert.equal(created.body.id, 'crate');
  assert.equal(created.body.shapeId, 'crate:shape');
  assert.equal(created.collider.id, 'crate:collider');
  assert.equal(created.collider.bodyId, 'crate');
  assert.equal(created.collider.materialId, 'material-default');

  const snapshot = world.getSnapshot();
  assert.equal(snapshot.shapeCount, 1);
  assert.equal(snapshot.bodyCount, 1);
  assert.equal(snapshot.colliderCount, 1);
  assert.equal(snapshot.materialCount, 1);
  assert.equal(snapshot.shapes[0].geometry.halfExtents.x, 2);
  assert.equal(snapshot.bodies[0].position.z, 3);
  assert.equal(snapshot.colliders[0].shapeId, 'crate:shape');
  assert.equal(snapshot.materials[0].id, 'material-default');
});

test('PhysicsWorld supports custom materials and static box colliders', () => {
  const world = new PhysicsWorld();
  world.createMaterial({
    id: 'ice',
    friction: 0.05,
    restitution: 0.2,
    density: 0.9
  });

  const created = world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -5, z: 0 },
    size: 20,
    materialId: 'ice'
  });

  assert.equal(created.shape.id, 'floor:shape');
  assert.equal(created.collider.id, 'floor:collider');
  assert.equal(created.collider.bodyId, null);
  assert.equal(created.collider.materialId, 'ice');

  const material = world.getEffectiveMaterialForCollider('floor:collider');
  const pose = world.getColliderWorldPose('floor:collider');

  assert.equal(material.id, 'ice');
  assert.equal(material.friction, 0.05);
  assert.equal(pose.position.y, -5);
});

test('PhysicsWorld fixed-step integration advances dynamic bodies', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 0.5,
    maxSubsteps: 4,
    gravity: { x: 0, y: -10, z: 0 }
  });

  world.createBoxBody({
    id: 'falling-box',
    position: { x: 0, y: 5, z: 0 },
    size: 2
  });

  const stepStats = world.step(1);
  const body = world.getBody('falling-box');

  assert.equal(stepStats.performedSubsteps, 2);
  assert.equal(stepStats.simulationTick, 2);
  assert.equal(body.linearVelocity.y, -10);
  assert.equal(body.position.y, -2.5);
});

test('PhysicsWorld angular integration advances orientation from angular velocity', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 0.5,
    maxSubsteps: 2,
    gravity: { x: 0, y: 0, z: 0 }
  });

  world.createBoxBody({
    id: 'spinner',
    position: { x: 0, y: 0, z: 0 },
    size: 2,
    angularVelocity: { x: 0, y: 0, z: Math.PI }
  });

  world.step(0.5);
  const body = world.getBody('spinner');
  const rotatedXAxis = rotateVec3ByQuat(body.rotation, { x: 1, y: 0, z: 0 });

  assert.ok(Math.abs(body.rotation.z) > 0.5, `expected non-trivial z quaternion component, got ${body.rotation.z}`);
  assert.ok(rotatedXAxis.y > 0.8, `expected x axis to rotate toward +y, got ${JSON.stringify(rotatedXAxis)}`);
});

test('PhysicsWorld debug frames expose the shared primitive schema', () => {
  const world = new PhysicsWorld();
  world.createBoxBody({
    id: 'debug-box',
    position: { x: 4, y: 5, z: 6 },
    size: 2
  });

  const frame = world.buildDebugFrame();

  assert.equal(frame.schemaVersion, DEBUG_FRAME_SCHEMA_VERSION);
  assert.equal(frame.frameNumber, 1);
  assert.equal(frame.stats.bodyCount, 1);
  assert.equal(frame.stats.shapeCount, 1);
  assert.equal(frame.stats.colliderCount, 1);
  assert.equal(frame.stats.materialCount, 1);
  assert.equal(frame.stats.broadphasePairCount, 0);
  assert.equal(frame.stats.contactPairCount, 0);
  assert.equal(frame.stats.byType[DEBUG_PRIMITIVE_TYPES.WIRE_BOX], 2);
  assert.equal(frame.stats.byType[DEBUG_PRIMITIVE_TYPES.POINT], 1);
  assert.equal(frame.primitives[0].source.bodyId, 'debug-box');
  assert.equal(frame.primitives[0].source.colliderId, 'debug-box:collider');
  assert.equal(frame.primitives[0].source.materialId, 'material-default');
});

test('PhysicsWorld generates broadphase pairs and box-box narrowphase contact pairs', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -2, z: 0 },
    size: 4
  });
  world.createBoxBody({
    id: 'falling-box',
    position: { x: 0, y: 0, z: 0 },
    size: 4
  });

  const collisionState = world.getCollisionState();

  assert.equal(collisionState.summary.proxyCount, 2);
  assert.equal(collisionState.summary.pairCount, 1);
  assert.equal(collisionState.summary.contactCount, 1);
  assert.equal(collisionState.summary.manifoldCount, 1);
  assert.equal(collisionState.summary.pairKinds['dynamic-static'], 1);
  assert.equal(collisionState.summary.algorithms['box-box-aabb-v2'], 1);
  assert.equal(collisionState.broadphasePairs[0].pairKind, 'dynamic-static');
  assert.equal(collisionState.contactPairs[0].contactCount, 4);
  assert.equal(collisionState.manifolds[0].contactCount, 4);
  assert.equal(collisionState.manifolds[0].contacts[0].accumulatedNormalImpulse, 0);
  assert.equal(collisionState.contactPairs[0].contacts[0].featureId.startsWith('axis:y:corner:'), true);
  assert.equal(collisionState.contactPairs[0].normal.y, 1);
  assert.equal(collisionState.contactPairs[0].penetration, 2);
});

test('PhysicsWorld sphere restitution produces a bounce against a floor box', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 10
  });
  world.createMaterial({
    id: 'bouncy',
    friction: 0,
    restitution: 0.8
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2,
    materialId: 'bouncy'
  });
  world.createSphereBody({
    id: 'ball',
    position: { x: 0, y: 3, z: 0 },
    radius: 0.5,
    mass: 1,
    materialId: 'bouncy'
  });

  let observedBounce = false;
  for (let stepIndex = 0; stepIndex < 240; stepIndex += 1) {
    world.step(1 / 120);
    if (world.getBody('ball').linearVelocity.y > 0.5) {
      observedBounce = true;
      break;
    }
  }

  assert.equal(observedBounce, true);
});

test('PhysicsWorld sphere-box overlaps use the GJK/EPA manifold path', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });
  world.createSphereBody({
    id: 'ball',
    position: { x: 0, y: 0.2, z: 0 },
    radius: 0.5,
    mass: 1
  });

  const collisionState = world.getCollisionState();

  assert.equal(collisionState.summary.algorithms['gjk-epa-manifold-v1'], 1);
  assert.ok(collisionState.contactPairs[0].contactCount >= 1);
});

test('PhysicsWorld friction reduces tangential velocity on a resting box', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12
  });
  world.createMaterial({
    id: 'grippy',
    friction: 1.2,
    restitution: 0
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2,
    materialId: 'grippy'
  });
  world.createBoxBody({
    id: 'slider',
    position: { x: 0, y: 1, z: 0 },
    size: 2,
    mass: 1,
    linearVelocity: { x: 4, y: 0, z: 0 },
    materialId: 'grippy'
  });

  let maxTangentSolved = 0;
  for (let stepIndex = 0; stepIndex < 24; stepIndex += 1) {
    world.step(1 / 120);
    maxTangentSolved = Math.max(maxTangentSolved, world.getSnapshot().lastSolverStats.solvedTangentContactCount);
  }

  const slider = world.getBody('slider');

  assert.ok(Math.abs(slider.linearVelocity.x) < 4, `expected friction to reduce x velocity, got ${slider.linearVelocity.x}`);
  assert.ok(maxTangentSolved > 0);
});

test('PhysicsWorld friction transfers tangential motion into sphere spin', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12,
    gravity: { x: 0, y: -9.81, z: 0 }
  });
  world.createMaterial({
    id: 'rolling',
    friction: 1.1,
    restitution: 0
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2,
    materialId: 'rolling'
  });
  world.createSphereBody({
    id: 'ball',
    position: { x: 0, y: 0.55, z: 0 },
    radius: 0.5,
    mass: 1,
    linearVelocity: { x: 4, y: 0, z: 0 },
    materialId: 'rolling'
  });

  for (let stepIndex = 0; stepIndex < 48; stepIndex += 1) {
    world.step(1 / 120);
  }

  const ball = world.getBody('ball');
  assert.ok(Math.abs(ball.angularVelocity.z) > 0.2, `expected rolling spin around z, got ${ball.angularVelocity.z}`);
  assert.ok(Math.abs(ball.linearVelocity.x) < 4, `expected friction to reduce sphere x velocity, got ${ball.linearVelocity.x}`);
});

test('PhysicsWorld capsule-sphere narrowphase reports GJK/EPA manifold contacts', () => {
  const world = new PhysicsWorld();
  world.createCapsuleBody({
    id: 'capsule',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    radius: 0.5,
    halfHeight: 1
  });
  world.createSphereBody({
    id: 'sphere',
    position: { x: 0.8, y: 0, z: 0 },
    radius: 0.5,
    mass: 1
  });

  const collisionState = world.getCollisionState();

  assert.equal(collisionState.summary.algorithms['gjk-epa-manifold-v1'], 1);
  assert.ok(collisionState.contactPairs[0].contactCount >= 1);
});

test('PhysicsWorld convex hulls use multi-point GJK/EPA manifold generation', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });
  world.createConvexHullBody({
    id: 'hull',
    position: { x: 0, y: 0.3, z: 0 },
    vertices: [
      { x: -0.6, y: -0.4, z: -0.6 },
      { x: 0.6, y: -0.4, z: -0.6 },
      { x: 0.6, y: -0.4, z: 0.6 },
      { x: -0.6, y: -0.4, z: 0.6 },
      { x: 0, y: 0.8, z: 0 }
    ],
    mass: 1
  });

  const collisionState = world.getCollisionState();

  const featureIds = new Set(collisionState.contactPairs[0].contacts.map((contact) => contact.featureId));

  assert.equal(collisionState.summary.algorithms['gjk-epa-manifold-v1'], 1);
  assert.ok(collisionState.contactPairs[0].contactCount >= 3, `expected multi-point hull contact, got ${collisionState.contactPairs[0].contactCount}`);
  assert.ok(featureIds.size >= 3, `expected distinct hull feature ids, got ${Array.from(featureIds).join(', ')}`);
});

test('PhysicsWorld capsule-box pairs use the GJK/EPA manifold path', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });
  world.createCapsuleBody({
    id: 'capsule',
    position: { x: 0, y: 0.4, z: 0 },
    radius: 0.5,
    halfHeight: 0.8,
    mass: 1
  });

  const collisionState = world.getCollisionState();

  assert.equal(collisionState.summary.algorithms['gjk-epa-manifold-v1'], 1);
  assert.ok(collisionState.contactPairs[0].contactCount >= 1);
});

test('PhysicsWorld manifold cache and normal solver keep a falling box resting on a floor', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 60,
    solverIterations: 10
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'rest-box',
    position: { x: 0, y: 4, z: 0 },
    size: 2,
    mass: 1
  });

  for (let stepIndex = 0; stepIndex < 180; stepIndex += 1) {
    world.step(1 / 60);
  }

  const box = world.getBody('rest-box');
  const collisionState = world.getCollisionState();
  const restManifold = collisionState.manifolds.find((manifold) => manifold.pairKey === 'floor:collider|rest-box:collider');

  assert.ok(restManifold);
  assert.equal(collisionState.summary.manifoldCount, 1);
  assert.ok(restManifold.contacts[0].accumulatedNormalImpulse > 0);
  assert.ok(Math.abs(restManifold.contacts[0].accumulatedTangentImpulseA) < 0.05);
  assert.ok(Math.abs(box.position.y - 1) < 0.1, `expected resting height near 1, got ${box.position.y}`);
  assert.ok(Math.abs(box.linearVelocity.y) < 0.2, `expected resting velocity near 0, got ${box.linearVelocity.y}`);
  assert.ok(Math.abs(box.angularVelocity.x) < 0.05, `expected resting angular x velocity near 0, got ${box.angularVelocity.x}`);
});

test('PhysicsWorld normal solver supports a simple two-box stack', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'bottom-box',
    position: { x: 0, y: 1.2, z: 0 },
    size: 2,
    mass: 1
  });
  world.createBoxBody({
    id: 'top-box',
    position: { x: 0, y: 3.4, z: 0 },
    size: 2,
    mass: 1
  });

  for (let stepIndex = 0; stepIndex < 360; stepIndex += 1) {
    world.step(1 / 120);
  }

  const bottomBox = world.getBody('bottom-box');
  const topBox = world.getBody('top-box');
  const collisionState = world.getCollisionState();

  assert.ok(bottomBox.position.y > 0.8 && bottomBox.position.y < 1.2, `unexpected bottom box height ${bottomBox.position.y}`);
  assert.ok(topBox.position.y > 2.75 && topBox.position.y < 3.25, `unexpected top box height ${topBox.position.y}`);
  assert.ok(topBox.position.y > bottomBox.position.y + 1.7, 'top box should remain above bottom box');
  assert.ok(collisionState.summary.manifoldCount >= 2);
  assert.ok(collisionState.summary.contactCount >= 2);
});

test('PhysicsWorld off-center box impact generates angular velocity', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createStaticBoxCollider({
    id: 'wall',
    position: { x: 3, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'impact-box',
    position: { x: 0, y: 1.25, z: 0 },
    size: 2,
    mass: 1,
    linearVelocity: { x: 8, y: 0, z: 0 }
  });

  let observedSpin = false;
  for (let stepIndex = 0; stepIndex < 80; stepIndex += 1) {
    world.step(1 / 120);
    const body = world.getBody('impact-box');
    if (Math.abs(body.angularVelocity.z) > 0.2) {
      observedSpin = true;
      break;
    }
  }

  const body = world.getBody('impact-box');
  assert.equal(observedSpin, true);
  assert.ok(Math.abs(body.angularVelocity.z) > 0.2, `expected off-center impact spin, got ${body.angularVelocity.z}`);
});

test('PhysicsWorld point and AABB queries return overlapping bodies and colliders', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 2, y: -4, z: 0 },
    size: 8
  });
  world.createBoxBody({
    id: 'query-box',
    position: { x: 2, y: 0, z: 0 },
    size: 4
  });

  const pointResult = world.queryPoint({ x: 2, y: 0, z: 0 });
  const aabbResult = world.queryAabb({
    center: { x: 2, y: 0, z: 0 },
    halfExtents: { x: 5, y: 5, z: 5 }
  });

  assert.equal(pointResult.count.bodies, 1);
  assert.equal(pointResult.count.colliders, 2);
  assert.equal(pointResult.bodies[0].id, 'query-box');
  assert.equal(aabbResult.count.bodies, 1);
  assert.equal(aabbResult.count.colliders, 2);
});

test('PhysicsWorld exposes collider world poses, body collider lookup, and world summary', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: -3, z: 0 }
  });
  world.createMaterial({
    id: 'rubber',
    friction: 1,
    restitution: 0.8
  });
  world.createBoxBody({
    id: 'pose-box',
    position: { x: 7, y: 8, z: 9 },
    size: 2,
    materialId: 'rubber'
  });

  const pose = world.getColliderWorldPose('pose-box:collider');
  const colliders = world.getBodyColliders('pose-box');
  const summary = world.getWorldSummary();

  assert.equal(pose.position.x, 7);
  assert.equal(colliders.length, 1);
  assert.equal(colliders[0].materialId, 'rubber');
  assert.equal(summary.bodyCount, 1);
  assert.equal(summary.colliderCount, 1);
  assert.equal(summary.materialCount, 2);
  assert.equal(summary.broadphaseProxyCount, 1);
  assert.equal(summary.broadphasePairCount, 0);
  assert.equal(summary.contactPairCount, 0);
  assert.equal(summary.manifoldCount, 0);
  assert.equal(summary.gravity.y, -3);
});

test('PhysicsWorld broadphase filters static-static pairs and same-body collider pairs', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'static-a',
    position: { x: 0, y: 0, z: 0 },
    size: 4
  });
  world.createStaticBoxCollider({
    id: 'static-b',
    position: { x: 1, y: 0, z: 0 },
    size: 4
  });
  const body = world.createRigidBody({
    id: 'compound-body',
    position: { x: 10, y: 0, z: 0 }
  });
  const shapeA = world.createBoxShape({
    id: 'compound-a:shape',
    halfExtents: { x: 1, y: 1, z: 1 }
  });
  const shapeB = world.createBoxShape({
    id: 'compound-b:shape',
    halfExtents: { x: 1, y: 1, z: 1 }
  });
  world.createCollider({
    id: 'compound-a:collider',
    bodyId: body.id,
    shapeId: shapeA.id,
    localPose: { position: { x: -0.5, y: 0, z: 0 } }
  });
  world.createCollider({
    id: 'compound-b:collider',
    bodyId: body.id,
    shapeId: shapeB.id,
    localPose: { position: { x: 0.5, y: 0, z: 0 } }
  });

  const collisionState = world.getCollisionState();

  assert.equal(collisionState.summary.proxyCount, 4);
  assert.equal(collisionState.summary.pairCount, 0);
  assert.equal(collisionState.summary.contactCount, 0);
  assert.equal(collisionState.summary.manifoldCount, 0);
});

test('PhysicsWorld reset clears registries and restores the default material', () => {
  const world = new PhysicsWorld();
  world.createMaterial({
    id: 'wood',
    friction: 0.6
  });
  world.createBoxBody({
    id: 'to-reset',
    position: { x: 0, y: 0, z: 0 },
    size: 1
  });
  world.buildDebugFrame();

  world.reset();

  const snapshot = world.getSnapshot();
  const frame = world.buildDebugFrame();

  assert.equal(snapshot.bodyCount, 0);
  assert.equal(snapshot.shapeCount, 0);
  assert.equal(snapshot.colliderCount, 0);
  assert.equal(snapshot.materialCount, 1);
  assert.equal(snapshot.materials[0].id, 'material-default');
  assert.equal(snapshot.renderFrameCount, 0);
  assert.equal(frame.frameNumber, 1);
  assert.equal(frame.primitives.length, 0);
});
