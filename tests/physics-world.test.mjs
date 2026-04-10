import test from 'node:test';
import assert from 'node:assert/strict';
import { DEBUG_FRAME_SCHEMA_VERSION, DEBUG_PRIMITIVE_TYPES, PhysicsWorld } from '../src/physics/index.js';

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
  assert.equal(collisionState.summary.pairKinds['dynamic-static'], 1);
  assert.equal(collisionState.summary.algorithms['box-box-aabb-v1'], 1);
  assert.equal(collisionState.broadphasePairs[0].pairKind, 'dynamic-static');
  assert.equal(collisionState.contactPairs[0].contactCount, 1);
  assert.equal(collisionState.contactPairs[0].contacts[0].featureId, 'axis:y');
  assert.equal(collisionState.contactPairs[0].normal.y, 1);
  assert.equal(collisionState.contactPairs[0].penetration, 2);
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
