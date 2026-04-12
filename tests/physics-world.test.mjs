import test from 'node:test';
import assert from 'node:assert/strict';
import { DEBUG_FRAME_SCHEMA_VERSION, DEBUG_PRIMITIVE_TYPES, PhysicsWorld, createQuatFromAxisAngle, dotVec3, rotateVec3ByQuat } from '../src/physics/index.js';
import { getShapeSupportPolygon } from '../src/physics/collision/support.js';

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

test('PhysicsWorld draws convex hull debug geometry as per-face line segments', () => {
  const world = new PhysicsWorld();
  world.createConvexHullBody({
    id: 'debug-hull',
    position: { x: 0, y: 0, z: 0 },
    vertices: [
      { x: -1, y: -1, z: -1 },
      { x: 1, y: -1, z: -1 },
      { x: 1, y: -1, z: 1 },
      { x: -1, y: -1, z: 1 },
      { x: 0, y: 1, z: 0 }
    ]
  });

  const frame = world.buildDebugFrame();

  assert.ok((frame.stats.byType[DEBUG_PRIMITIVE_TYPES.LINE] ?? 0) >= 8);
  assert.equal(frame.stats.byType[DEBUG_PRIMITIVE_TYPES.WIRE_BOX], 1);
  assert.ok(frame.primitives.some((primitive) => primitive.id === 'debug-hull:collider:wire-edge:0'));
  assert.ok(frame.primitives.every((primitive) => primitive.id !== 'debug-hull:collider:wire-box'));
});

test('PhysicsWorld debug camera preserves independent angle state', () => {
  const world = new PhysicsWorld();
  world.setDebugCameraPosition({ x: 120, y: 60, z: 320 });
  world.setDebugCameraTarget({ x: 10, y: 20, z: 30 });

  const snapshot = world.getSnapshot();
  const frame = world.buildDebugFrame();

  assert.deepEqual(snapshot.debugCamera.position, { x: 120, y: 60, z: 320 });
  assert.deepEqual(snapshot.debugCamera.target, { x: 10, y: 20, z: 30 });
  assert.deepEqual(frame.camera.position, { x: 120, y: 60, z: 320 });
  assert.deepEqual(frame.camera.target, { x: 10, y: 20, z: 30 });
});

test('PhysicsWorld exports and imports scene definitions with stable ids and settings', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 90,
    maxSubsteps: 3,
    solverIterations: 12
  });
  world.setGravity({ x: 0, y: -12, z: 0 });
  world.setDebugCameraPosition({ x: 200, y: 120, z: 500 });
  world.setDebugCameraTarget({ x: -8, y: 18, z: 0 });
  world.createMaterial({
    id: 'ice',
    friction: 0.05,
    restitution: 0.1,
    density: 0.8
  });
  world.createStaticConvexHullCollider({
    id: 'ramp',
    position: { x: 0, y: -20, z: 0 },
    vertices: [
      { x: -40, y: -20, z: -20 },
      { x: 40, y: -20, z: -20 },
      { x: 40, y: -20, z: 20 },
      { x: -40, y: -20, z: 20 },
      { x: -40, y: 10, z: -20 },
      { x: -40, y: 10, z: 20 }
    ],
    materialId: 'ice'
  });
  world.createBoxBody({
    id: 'crate',
    position: { x: 0, y: 20, z: 0 },
    size: 10,
    mass: 2,
    materialId: 'ice'
  });
  world.createConvexHullBody({
    id: 'hull',
    position: { x: 20, y: 35, z: 0 },
    mass: 1,
    materialId: 'ice',
    vertices: [
      { x: -5, y: -5, z: -5 },
      { x: 5, y: -5, z: -5 },
      { x: 5, y: -5, z: 5 },
      { x: -5, y: -5, z: 5 },
      { x: 0, y: 7, z: 0 }
    ]
  });
  world.createDistanceJoint({
    id: 'link',
    bodyAId: 'crate',
    bodyBId: 'hull',
    distance: 24
  });

  const scene = world.exportSceneDefinition();
  const restoredWorld = new PhysicsWorld();
  const imported = restoredWorld.importSceneDefinition(scene, {
    reset: true
  });

  assert.equal(scene.schemaVersion, 'physics-scene@1');
  assert.equal(imported.bodyCount, 2);
  assert.equal(imported.colliderCount, 3);
  assert.equal(imported.jointCount, 1);
  assert.equal(restoredWorld.getBody('crate').mass, 2);
  assert.equal(restoredWorld.getCollider('ramp:collider').materialId, 'ice');
  assert.equal(restoredWorld.getJoint('link').type, 'distance-joint');
  assert.deepEqual(restoredWorld.getSnapshot().gravity, { x: 0, y: -12, z: 0 });
  assert.deepEqual(restoredWorld.getSnapshot().debugCamera.position, { x: 200, y: 120, z: 500 });
  assert.deepEqual(restoredWorld.getSnapshot().debugCamera.target, { x: -8, y: 18, z: 0 });
  assert.equal(restoredWorld.fixedDeltaTime, 1 / 90);
  assert.equal(restoredWorld.solverIterations, 12);
});

test('PhysicsWorld generates broadphase pairs and box-box convex manifold contact pairs', () => {
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
  assert.equal(collisionState.summary.algorithms['gjk-epa-manifold-v1'], 1);
  assert.equal(collisionState.broadphasePairs[0].pairKind, 'dynamic-static');
  assert.equal(collisionState.contactPairs[0].contactCount, 4);
  assert.equal(collisionState.manifolds[0].contactCount, 4);
  assert.equal(collisionState.manifolds[0].contacts[0].accumulatedNormalImpulse, 0);
  assert.equal(collisionState.contactPairs[0].contacts[0].featureId.includes('|'), true);
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
  for (let stepIndex = 0; stepIndex < 480; stepIndex += 1) {
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

test('Support polygons only keep true extreme box features for face, edge, and corner directions', () => {
  const world = new PhysicsWorld();
  const boxShape = world.createBoxShape({
    id: 'box-shape',
    halfExtents: { x: 1, y: 1, z: 1 }
  });
  const pose = {
    position: { x: 0, y: 0, z: 0 },
    rotation: { x: 0, y: 0, z: 0, w: 1 }
  };

  const facePolygon = getShapeSupportPolygon(boxShape, pose, { x: 0, y: 1, z: 0 });
  const edgePolygon = getShapeSupportPolygon(boxShape, pose, { x: 1, y: 1, z: 0 });
  const cornerPolygon = getShapeSupportPolygon(boxShape, pose, { x: 1, y: 0.2, z: 1 });

  assert.equal(facePolygon.points.length, 4);
  assert.equal(edgePolygon.points.length, 2);
  assert.equal(cornerPolygon.points.length, 1);
});

test('Support polygons do not expand a convex hull vertex contact into a full face', () => {
  const world = new PhysicsWorld();
  const hullShape = world.createConvexHullShape({
    id: 'pyramid',
    vertices: [
      { x: -30, y: -30, z: -30 },
      { x: 30, y: -30, z: -30 },
      { x: 30, y: -30, z: 30 },
      { x: -30, y: -30, z: 30 },
      { x: 0, y: 45, z: 0 }
    ]
  });
  const pose = {
    position: { x: 0, y: 0, z: 0 },
    rotation: { x: 0, y: 0, z: 0, w: 1 }
  };

  const supportPolygon = getShapeSupportPolygon(hullShape, pose, {
    x: 0.28734788556634544,
    y: 0.9578262852211515,
    z: 0
  });

  assert.equal(supportPolygon.points.length, 1);
  assert.equal(supportPolygon.points[0].featureId, 'vertex:4');
});

test('PhysicsWorld unsupported box on a static convex ledge tips and keeps contacts inside the support region', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 14
  });
  world.createStaticConvexHullCollider({
    id: 'ledge',
    position: { x: 0, y: 0, z: 0 },
    vertices: [
      { x: -40, y: -10, z: -40 },
      { x: 0, y: -10, z: -40 },
      { x: 0, y: -10, z: 40 },
      { x: -40, y: -10, z: 40 },
      { x: -40, y: 10, z: -40 },
      { x: 0, y: 10, z: -40 },
      { x: 0, y: 10, z: 40 },
      { x: -40, y: 10, z: 40 }
    ]
  });
  world.createBoxBody({
    id: 'box',
    position: { x: 10, y: 30, z: 0 },
    size: 40,
    mass: 1
  });

  let firstContactPair = null;
  let maxTilt = 0;
  let maxPenetration = 0;
  let maxUpwardVelocity = 0;
  for (let stepIndex = 0; stepIndex < 480; stepIndex += 1) {
    world.step(1 / 120);
    const body = world.getBody('box');
    maxTilt = Math.max(maxTilt, Math.abs(body.rotation.x), Math.abs(body.rotation.z));
    maxPenetration = Math.max(maxPenetration, world.getCollisionState().solverStats.maxPenetration);
    maxUpwardVelocity = Math.max(maxUpwardVelocity, body.linearVelocity.y);
    const contactPair = world.getCollisionState().contactPairs.find((pair) => pair.pairKey === 'box:collider|ledge:collider' || pair.pairKey === 'ledge:collider|box:collider');
    if (contactPair && !firstContactPair) {
      firstContactPair = contactPair;
    }
  }

  assert.ok(firstContactPair, 'expected the box to contact the ledge');
  for (const contact of firstContactPair.contacts) {
    assert.ok(contact.position.x >= -10.1 && contact.position.x <= 0.1, `expected clipped contact x to stay on the ledge support region, got ${contact.position.x}`);
  }
  assert.ok(maxTilt > 0.2, `expected unsupported box to tip, got max tilt ${maxTilt}`);
  assert.ok(maxPenetration < 1, `expected ledge tip to stay shallow, got max penetration ${maxPenetration}`);
  assert.ok(maxUpwardVelocity < 1, `expected ledge tip to avoid catapulting upward, got max upward velocity ${maxUpwardVelocity}`);
});

test('PhysicsWorld convex hull on a static ramp keeps first manifold contacts on the actual support plane', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 14
  });
  world.createStaticConvexHullCollider({
    id: 'ramp',
    position: { x: 0, y: -30, z: 0 },
    vertices: [
      { x: -100, y: -20, z: -60 },
      { x: 100, y: -20, z: -60 },
      { x: 100, y: -20, z: 60 },
      { x: -100, y: -20, z: 60 },
      { x: -100, y: 40, z: -60 },
      { x: -100, y: 40, z: 60 }
    ]
  });
  world.createConvexHullBody({
    id: 'hull',
    position: { x: -20, y: 120, z: 0 },
    vertices: [
      { x: -30, y: -30, z: -30 },
      { x: 30, y: -30, z: -30 },
      { x: 30, y: -30, z: 30 },
      { x: -30, y: -30, z: 30 },
      { x: 0, y: 45, z: 0 }
    ],
    mass: 1
  });

  let firstContactPair = null;
  let maxPenetration = 0;
  let maxUpwardVelocity = 0;
  for (let stepIndex = 0; stepIndex < 720; stepIndex += 1) {
    world.step(1 / 120);
    const body = world.getBody('hull');
    maxPenetration = Math.max(maxPenetration, world.getCollisionState().solverStats.maxPenetration);
    maxUpwardVelocity = Math.max(maxUpwardVelocity, body.linearVelocity.y);
    const contactPair = world.getCollisionState().contactPairs.find((pair) => pair.colliderAId === 'ramp:collider' && pair.colliderBId === 'hull:collider');
    if (contactPair) {
      firstContactPair = contactPair;
      break;
    }
  }

  assert.ok(firstContactPair, 'expected hull to contact the ramp');
  assert.ok(firstContactPair.contactCount <= 2, `expected vertex/edge ramp contact, got ${firstContactPair.contactCount}`);

  const rampNormal = { x: 0.28734788556634544, y: 0.9578262852211515, z: 0 };
  const rampPlaneOffset = -19.15652570442303;
  for (const contact of firstContactPair.contacts) {
    const signedDistance = contact.position.x * rampNormal.x + contact.position.y * rampNormal.y + contact.position.z * rampNormal.z - rampPlaneOffset;
    assert.ok(
      Math.abs(signedDistance + contact.penetration * 0.5) < 0.1,
      `expected ramp contact to stay close to the support plane, got signed distance ${signedDistance} for penetration ${contact.penetration}`
    );
  }
  assert.ok(maxPenetration < 1, `expected ramp contact to avoid deep false penetration, got ${maxPenetration}`);
  assert.ok(maxUpwardVelocity < 1, `expected ramp tip to avoid catapulting upward, got ${maxUpwardVelocity}`);
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

test('PhysicsWorld culls shallow separating contacts instead of keeping stale manifolds alive', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'lifting-box',
    position: { x: 0, y: 0.995, z: 0 },
    size: 2,
    mass: 1,
    linearVelocity: { x: 0, y: 1, z: 0 }
  });

  const collisionState = world.getCollisionState();

  assert.equal(collisionState.summary.contactCount, 0);
  assert.equal(collisionState.summary.manifoldCount, 0);
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

test('PhysicsWorld offset box stacking stays on the convex manifold path', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 60,
    solverIterations: 12
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -50, z: 0 },
    size: 100
  });
  world.createBoxBody({
    id: 'bottom',
    position: { x: 0, y: 40, z: 0 },
    size: 40,
    mass: 1
  });
  world.createBoxBody({
    id: 'top',
    position: { x: 30, y: 120, z: 0 },
    size: 40,
    mass: 1
  });

  let observedAabbFallback = false;
  let observedConvexStackContact = false;
  for (let stepIndex = 0; stepIndex < 240; stepIndex += 1) {
    world.step(1 / 60);
    const collisionState = world.getCollisionState();
    if ((collisionState.summary.algorithms['support-mapped-aabb-v1'] ?? 0) > 0) {
      observedAabbFallback = true;
      break;
    }

    const stackedPair = collisionState.contactPairs.find((pair) => pair.pairKey === 'bottom:collider|top:collider');
    if (stackedPair) {
      observedConvexStackContact = true;
      assert.equal(stackedPair.algorithm, 'gjk-epa-manifold-v1');
    }
  }

  assert.equal(observedAabbFallback, false);
  assert.equal(observedConvexStackContact, true);
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
  let maxObservedSpin = 0;
  for (let stepIndex = 0; stepIndex < 80; stepIndex += 1) {
    world.step(1 / 120);
    const body = world.getBody('impact-box');
    maxObservedSpin = Math.max(maxObservedSpin, Math.abs(body.angularVelocity.z));
    if (Math.abs(body.angularVelocity.z) > 0.1) {
      observedSpin = true;
      break;
    }
  }

  const body = world.getBody('impact-box');
  assert.equal(observedSpin, true);
  assert.ok(maxObservedSpin > 0.1, `expected off-center impact spin, got ${maxObservedSpin}`);
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

test('PhysicsWorld contact queries list touching bodies and colliders', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'box-a',
    position: { x: 0, y: 0, z: 0 },
    size: 4,
    mass: 1
  });
  world.createBoxBody({
    id: 'box-b',
    position: { x: 2.5, y: 0, z: 0 },
    size: 4,
    mass: 1
  });

  const touchingBodies = world.getBodiesTouchingBody('box-a');
  const touchingColliders = world.getCollidersTouchingCollider('box-a:collider');

  assert.equal(touchingBodies.count, 1);
  assert.equal(touchingBodies.bodies[0].id, 'box-b');
  assert.equal(touchingBodies.pairs.length, 1);
  assert.equal(touchingColliders.count, 1);
  assert.equal(touchingColliders.colliders[0].id, 'box-b:collider');
  assert.equal(touchingColliders.pairs[0].pairKey, 'box-a:collider|box-b:collider');
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

test('PhysicsWorld raycast returns the nearest hit collider and stores the query snapshot', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'near-floor',
    position: { x: 0, y: -5, z: 0 },
    size: 2
  });
  world.createStaticBoxCollider({
    id: 'far-floor',
    position: { x: 0, y: -15, z: 0 },
    size: 2
  });

  const result = world.raycast({
    origin: { x: 0, y: 10, z: 0 },
    direction: { x: 0, y: -1, z: 0 },
    maxDistance: 40
  });
  const snapshot = world.getSnapshot();

  assert.equal(result.hit, true);
  assert.equal(result.colliderId, 'near-floor:collider');
  assert.equal(result.bodyId, null);
  assert.equal(result.shapeType, 'box');
  assert.equal(result.algorithm, 'box-raycast-v1');
  assert.ok(Math.abs(result.distance - 14) < 1e-6, `expected hit distance near 14, got ${result.distance}`);
  assert.equal(result.normal.y, 1);
  assert.equal(result.candidateCount, 2);
  assert.equal(snapshot.lastRaycast.hit, true);
  assert.equal(snapshot.lastRaycast.colliderId, 'near-floor:collider');
});

test('PhysicsWorld convex hull raycast uses the support-mapped query path', () => {
  const world = new PhysicsWorld();
  world.createConvexHullBody({
    id: 'hull',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    vertices: [
      { x: -1, y: -1, z: -1 },
      { x: 1, y: -1, z: -1 },
      { x: 1, y: -1, z: 1 },
      { x: -1, y: -1, z: 1 },
      { x: 0, y: 1, z: 0 }
    ]
  });

  const result = world.raycast({
    origin: { x: 0, y: 5, z: 0 },
    direction: { x: 0, y: -1, z: 0 },
    maxDistance: 10
  });
  const snapshot = world.getSnapshot();

  assert.equal(result.hit, true);
  assert.equal(result.colliderId, 'hull:collider');
  assert.equal(result.shapeType, 'convex-hull');
  assert.equal(result.algorithm, 'convex-raycast-v1');
  assert.ok(Math.abs(result.distance - 4) < 1e-3, `expected convex hull hit distance near 4, got ${result.distance}`);
  assert.ok(Math.abs(result.point.y - 1) < 1e-3, `expected convex hull hit point y near 1, got ${result.point.y}`);
  assert.equal(snapshot.lastRaycast.algorithm, 'convex-raycast-v1');
});

test('PhysicsWorld debug frame visualizes the last successful raycast', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });

  world.raycast({
    origin: { x: 0, y: 5, z: 0 },
    direction: { x: 0, y: -1, z: 0 },
    maxDistance: 10
  });

  const frame = world.buildDebugFrame();
  const categories = new Set(frame.primitives.map((primitive) => primitive.category));

  assert.equal(frame.stats.raycastHit, true);
  assert.ok(categories.has('raycast-line'));
  assert.ok(categories.has('raycast-hit-point'));
  assert.ok(categories.has('raycast-hit-normal'));
  assert.ok(frame.stats.byType[DEBUG_PRIMITIVE_TYPES.LINE] >= 2);
  assert.ok(frame.stats.byType[DEBUG_PRIMITIVE_TYPES.POINT] >= 2);
});

test('PhysicsWorld raycast miss keeps only the query line in the debug frame', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });

  const result = world.raycast({
    origin: { x: 0, y: 5, z: 0 },
    direction: { x: 1, y: 0, z: 0 },
    maxDistance: 10
  });
  const frame = world.buildDebugFrame();
  const categories = frame.primitives.map((primitive) => primitive.category);

  assert.equal(result.hit, false);
  assert.equal(frame.stats.raycastHit, false);
  assert.ok(categories.includes('raycast-line'));
  assert.equal(categories.includes('raycast-hit-point'), false);
  assert.equal(categories.includes('raycast-hit-normal'), false);
});

test('PhysicsWorld sphere cast hits the nearest collider and records the sweep pose', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -5, z: 0 },
    size: 2
  });

  const result = world.sphereCast({
    origin: { x: 0, y: 10, z: 0 },
    radius: 1,
    direction: { x: 0, y: -1, z: 0 },
    maxDistance: 30
  });

  assert.equal(result.hit, true);
  assert.equal(result.castType, 'sphere');
  assert.equal(result.colliderId, 'floor:collider');
  assert.equal(result.algorithm, 'convex-toi-v1');
  assert.ok(Math.abs(result.distance - 13) < 0.01, `expected sphere cast distance near 13, got ${result.distance}`);
  assert.ok(Math.abs(result.sweepPosition.y - (-3)) < 0.01, `expected sweep center near -3, got ${result.sweepPosition.y}`);
  assert.equal(result.sampleOrigins.length, 1);
});

test('PhysicsWorld capsule cast returns a hit and exposes sample sweep lines', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'wall',
    position: { x: 6, y: 0, z: 0 },
    size: 2
  });

  const result = world.capsuleCast({
    origin: { x: 0, y: 0, z: 0 },
    radius: 0.5,
    halfHeight: 1,
    direction: { x: 1, y: 0, z: 0 },
    maxDistance: 10
  });
  const frame = world.buildDebugFrame();
  const categories = frame.primitives.map((primitive) => primitive.category);

  assert.equal(result.hit, true);
  assert.equal(result.castType, 'capsule');
  assert.equal(result.algorithm, 'convex-toi-v1');
  assert.equal(result.sampleOrigins.length, 3);
  assert.ok(categories.includes('shape-cast-line'));
  assert.ok(categories.includes('shape-cast-hit-point'));
  assert.ok(categories.includes('shape-cast-hit-normal'));
});

test('PhysicsWorld sphere cast uses convex TOI against a convex hull target', () => {
  const world = new PhysicsWorld();
  world.createConvexHullBody({
    id: 'hull',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    vertices: [
      { x: -1, y: -1, z: -1 },
      { x: 1, y: -1, z: -1 },
      { x: 1, y: -1, z: 1 },
      { x: -1, y: -1, z: 1 },
      { x: 0, y: 1, z: 0 }
    ]
  });

  const result = world.sphereCast({
    origin: { x: 0, y: 4, z: 0 },
    radius: 0.5,
    direction: { x: 0, y: -1, z: 0 },
    maxDistance: 10
  });

  assert.equal(result.hit, true);
  assert.equal(result.algorithm, 'convex-toi-v1');
  assert.equal(result.colliderId, 'hull:collider');
});

test('PhysicsWorld capsule cast uses convex TOI against a convex hull target', () => {
  const world = new PhysicsWorld();
  world.createConvexHullBody({
    id: 'hull',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    vertices: [
      { x: -1, y: -1, z: -1 },
      { x: 1, y: -1, z: -1 },
      { x: 1, y: -1, z: 1 },
      { x: -1, y: -1, z: 1 },
      { x: 0, y: 1, z: 0 }
    ]
  });

  const result = world.capsuleCast({
    origin: { x: 0, y: 4, z: 0 },
    radius: 0.5,
    halfHeight: 1,
    direction: { x: 0, y: -1, z: 0 },
    maxDistance: 10
  });

  assert.equal(result.hit, true);
  assert.equal(result.algorithm, 'convex-toi-v1');
  assert.equal(result.colliderId, 'hull:collider');
});

test('PhysicsWorld first-pass CCD prevents a fast sphere from tunneling through a thin wall', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 0.1,
    gravity: { x: 0, y: 0, z: 0 },
    ccdMotionThreshold: 0.5
  });
  world.createStaticBoxCollider({
    id: 'wall',
    position: { x: 0, y: 0, z: 0 },
    size: 1
  });
  world.createSphereBody({
    id: 'fast-ball',
    position: { x: -5, y: 0, z: 0 },
    radius: 0.5,
    mass: 1,
    linearVelocity: { x: 100, y: 0, z: 0 }
  });

  world.step(0.1);
  const ball = world.getBody('fast-ball');
  const ccdEvents = world.getLastCcdEvents();
  const frame = world.buildDebugFrame();
  const categories = frame.primitives.map((primitive) => primitive.category);

  assert.ok(ball.position.x < 0.1, `expected CCD to stop sphere before wall, got ${ball.position.x}`);
  assert.equal(ccdEvents.length, 1);
  assert.equal(ccdEvents[0].bodyId, 'fast-ball');
  assert.equal(ccdEvents[0].targetColliderId, 'wall:collider');
  assert.ok(categories.includes('ccd-path'));
  assert.ok(categories.includes('ccd-hit-point'));
  assert.ok(categories.includes('ccd-hit-normal'));
});

test('PhysicsWorld CCD uses convex TOI for a fast box against a thin wall', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 0.1,
    gravity: { x: 0, y: 0, z: 0 },
    ccdMotionThreshold: 0.5
  });
  world.createStaticBoxCollider({
    id: 'wall',
    position: { x: 0, y: 0, z: 0 },
    size: 1
  });
  world.createBoxBody({
    id: 'fast-box',
    position: { x: -5, y: 0, z: 0 },
    size: 1,
    mass: 1,
    linearVelocity: { x: 100, y: 0, z: 0 }
  });

  world.step(0.1);
  const body = world.getBody('fast-box');
  const ccdEvents = world.getLastCcdEvents();

  assert.ok(body.position.x < 0.1, `expected CCD to stop box before wall, got ${body.position.x}`);
  assert.equal(ccdEvents.length, 1);
  assert.equal(ccdEvents[0].algorithm, 'convex-toi-v1');
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

test('PhysicsWorld builds contact islands for touching stacks and isolated bodies', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'bottom',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'top',
    position: { x: 0, y: 1.9, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'lone',
    position: { x: 8, y: 0, z: 0 },
    size: 2
  });

  const collisionState = world.getCollisionState();
  const stackIsland = collisionState.islands.find((island) => island.bodyIds.includes('bottom') && island.bodyIds.includes('top'));
  const loneIsland = collisionState.islands.find((island) => island.bodyIds.length === 1 && island.bodyIds[0] === 'lone');

  assert.equal(collisionState.summary.islandCount, 2);
  assert.equal(collisionState.summary.sleepingBodyCount, 0);
  assert.equal(collisionState.summary.awakeBodyCount, 3);
  assert.ok(stackIsland);
  assert.equal(stackIsland.bodyIds.length, 2);
  assert.ok(stackIsland.manifoldCount >= 1);
  assert.ok(loneIsland);
});

test('PhysicsWorld puts a quiet resting body to sleep and exposes sleep stats', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12,
    sleepTimeThreshold: 0.25
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -1, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'sleep-box',
    position: { x: 0, y: 3, z: 0 },
    size: 2,
    mass: 1
  });

  for (let stepIndex = 0; stepIndex < 360; stepIndex += 1) {
    world.step(1 / 120);
  }

  const body = world.getBody('sleep-box');
  const summary = world.getWorldSummary();
  const collisionState = world.getCollisionState();
  const frame = world.buildDebugFrame();

  assert.equal(body.sleeping, true);
  assert.ok(body.sleepTimer >= 0.25);
  assert.ok(Math.abs(body.linearVelocity.y) < 1e-8, `expected zeroed linear velocity, got ${body.linearVelocity.y}`);
  assert.ok(Math.abs(body.angularVelocity.x) < 1e-8, `expected zeroed angular velocity, got ${body.angularVelocity.x}`);
  assert.equal(summary.islandCount, 1);
  assert.equal(summary.sleepingBodyCount, 1);
  assert.equal(summary.awakeBodyCount, 0);
  assert.equal(collisionState.islands[0].sleepingBodyCount, 1);
  assert.equal(frame.stats.sleepingBodyCount, 1);
});

test('PhysicsWorld sleeps an entire resting stack as one island', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12,
    sleepTimeThreshold: 0.25
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

  for (let stepIndex = 0; stepIndex < 600; stepIndex += 1) {
    world.step(1 / 120);
  }

  const bottomBox = world.getBody('bottom-box');
  const topBox = world.getBody('top-box');
  const collisionState = world.getCollisionState();
  const stackIsland = collisionState.islands.find((island) => island.bodyIds.includes('bottom-box') && island.bodyIds.includes('top-box'));

  assert.equal(bottomBox.sleeping, true);
  assert.equal(topBox.sleeping, true);
  assert.equal(collisionState.summary.islandCount, 1);
  assert.equal(collisionState.summary.sleepingBodyCount, 2);
  assert.equal(collisionState.summary.awakeBodyCount, 0);
  assert.ok(stackIsland);
  assert.equal(stackIsland.bodyIds.length, 2);
  assert.equal(stackIsland.sleepingBodyCount, 2);
});

test('PhysicsWorld wakes a sleeping body island when hit by an active body', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12,
    gravity: { x: 0, y: 0, z: 0 },
    sleepTimeThreshold: 0.2
  });
  world.createBoxBody({
    id: 'target',
    position: { x: 0, y: 0, z: 0 },
    size: 2,
    mass: 1
  });

  for (let stepIndex = 0; stepIndex < 60; stepIndex += 1) {
    world.step(1 / 120);
  }

  assert.equal(world.getBody('target').sleeping, true);

  world.createBoxBody({
    id: 'hitter',
    position: { x: -4, y: 0, z: 0 },
    size: 2,
    mass: 1,
    linearVelocity: { x: 8, y: 0, z: 0 }
  });

  let observedWake = false;
  for (let stepIndex = 0; stepIndex < 120; stepIndex += 1) {
    world.step(1 / 120);
    if (!world.getBody('target').sleeping) {
      observedWake = true;
      break;
    }
  }

  const target = world.getBody('target');
  const collisionState = world.getCollisionState();

  assert.equal(observedWake, true);
  assert.equal(target.sleeping, false);
  assert.ok(collisionState.summary.awakeBodyCount >= 1);
});

test('PhysicsWorld distance joint keeps two bodies in one constraint island and exposes joint state', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'joint-a',
    position: { x: 0, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createBoxBody({
    id: 'joint-b',
    position: { x: 3, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createBoxBody({
    id: 'joint-lone',
    position: { x: 12, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  const joint = world.createDistanceJoint({
    id: 'rope',
    bodyAId: 'joint-a',
    bodyBId: 'joint-b'
  });

  const summary = world.getWorldSummary();
  const anchors = world.getJointWorldAnchors('rope');
  const collisionState = world.getCollisionState();
  const ropeIsland = collisionState.islands.find((island) => island.jointIds.includes('rope'));

  assert.ok(joint);
  assert.equal(summary.jointCount, 1);
  assert.equal(summary.jointConstraintCount, 1);
  assert.equal(collisionState.summary.islandCount, 2);
  assert.ok(anchors);
  assert.ok(Math.abs(anchors.anchorA.x - 0) < 1e-6);
  assert.ok(Math.abs(anchors.anchorB.x - 3) < 1e-6);
  assert.ok(ropeIsland);
  assert.equal(ropeIsland.bodyIds.length, 2);
  assert.equal(ropeIsland.jointCount, 1);
  assert.equal(ropeIsland.constraintCount, 1);
});

test('PhysicsWorld distance joint transmits motion and limits separation drift', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 16,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'lead',
    position: { x: 0, y: 0, z: 0 },
    size: 2,
    mass: 1,
    linearVelocity: { x: 6, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'trail',
    position: { x: 3, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createDistanceJoint({
    id: 'link',
    bodyAId: 'lead',
    bodyBId: 'trail',
    distance: 3
  });

  let maxSolvedJointCount = 0;
  let maxJointImpulse = 0;
  for (let stepIndex = 0; stepIndex < 120; stepIndex += 1) {
    world.step(1 / 120);
    const solverStats = world.getSnapshot().lastSolverStats;
    maxSolvedJointCount = Math.max(maxSolvedJointCount, solverStats.solvedJointCount);
    maxJointImpulse = Math.max(maxJointImpulse, solverStats.jointImpulsesApplied);
  }

  const lead = world.getBody('lead');
  const trail = world.getBody('trail');
  const separation = Math.sqrt(
    (trail.position.x - lead.position.x) ** 2 +
    (trail.position.y - lead.position.y) ** 2 +
    (trail.position.z - lead.position.z) ** 2
  );
  assert.ok(trail.linearVelocity.x > 0.5, `expected joint to pull trailing body, got ${trail.linearVelocity.x}`);
  assert.ok(Math.abs(separation - 3) < 0.3, `expected distance to stay near 3, got ${separation}`);
  assert.ok(maxSolvedJointCount > 0);
  assert.ok(maxJointImpulse > 0);
});

test('PhysicsWorld distance-jointed resting pair sleeps together and draws joint debug primitives', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12,
    gravity: { x: 0, y: 0, z: 0 },
    sleepTimeThreshold: 0.2
  });
  world.createBoxBody({
    id: 'sleep-a',
    position: { x: 0, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createBoxBody({
    id: 'sleep-b',
    position: { x: 3, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createDistanceJoint({
    id: 'sleep-link',
    bodyAId: 'sleep-a',
    bodyBId: 'sleep-b',
    distance: 3
  });

  for (let stepIndex = 0; stepIndex < 80; stepIndex += 1) {
    world.step(1 / 120);
  }

  const collisionState = world.getCollisionState();
  const frame = world.buildDebugFrame();

  assert.equal(world.getBody('sleep-a').sleeping, true);
  assert.equal(world.getBody('sleep-b').sleeping, true);
  assert.equal(collisionState.summary.sleepingBodyCount, 2);
  assert.equal(collisionState.summary.islandCount, 1);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'distance-joint'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'distance-joint-anchor'));
});

test('PhysicsWorld waking one body in a sleeping distance-joint island wakes the connected body', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 12,
    gravity: { x: 0, y: 0, z: 0 },
    sleepTimeThreshold: 0.2
  });
  world.createBoxBody({
    id: 'driver',
    position: { x: 0, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createBoxBody({
    id: 'follower',
    position: { x: 3, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createDistanceJoint({
    id: 'wake-link',
    bodyAId: 'driver',
    bodyBId: 'follower',
    distance: 3
  });

  for (let stepIndex = 0; stepIndex < 80; stepIndex += 1) {
    world.step(1 / 120);
  }

  assert.equal(world.getBody('driver').sleeping, true);
  assert.equal(world.getBody('follower').sleeping, true);

  const driver = world.bodyRegistry.getMutable('driver');
  driver.linearVelocity.x = 5;
  world.wakeBody('driver');

  let followerWoke = false;
  for (let stepIndex = 0; stepIndex < 20; stepIndex += 1) {
    world.step(1 / 120);
    if (!world.getBody('follower').sleeping) {
      followerWoke = true;
      break;
    }
  }

  assert.equal(followerWoke, true);
  assert.equal(world.getBody('driver').sleeping, false);
  assert.equal(world.getBody('follower').sleeping, false);
});

test('PhysicsWorld point-to-point joint keeps shared anchors together and forms one island', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 16,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'ball-a',
    position: { x: -1, y: 0, z: 0 },
    size: 2,
    mass: 1,
    linearVelocity: { x: 4, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'ball-b',
    position: { x: 3, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createPointToPointJoint({
    id: 'ball-socket',
    bodyAId: 'ball-a',
    bodyBId: 'ball-b',
    worldAnchor: { x: 1, y: 0, z: 0 }
  });

  let maxSolvedJointCount = 0;
  for (let stepIndex = 0; stepIndex < 120; stepIndex += 1) {
    world.step(1 / 120);
    maxSolvedJointCount = Math.max(maxSolvedJointCount, world.getSnapshot().lastSolverStats.solvedJointCount);
  }

  const anchors = world.getJointWorldAnchors('ball-socket');
  const anchorError = Math.sqrt(
    (anchors.anchorB.x - anchors.anchorA.x) ** 2 +
    (anchors.anchorB.y - anchors.anchorA.y) ** 2 +
    (anchors.anchorB.z - anchors.anchorA.z) ** 2
  );
  const collisionState = world.getCollisionState();
  const island = collisionState.islands.find((entry) => entry.jointIds.includes('ball-socket'));

  assert.ok(anchorError < 0.2, `expected point-to-point anchors to stay close, got ${anchorError}`);
  assert.ok(world.getBody('ball-b').linearVelocity.x > 0.5, `expected second body to be dragged, got ${world.getBody('ball-b').linearVelocity.x}`);
  assert.ok(maxSolvedJointCount > 0);
  assert.ok(island);
  assert.equal(island.bodyIds.length, 2);
});

test('PhysicsWorld hinge joint keeps its pivot near the frame anchor and exposes hinge axes', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 20,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'frame',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'door',
    position: { x: 2, y: 0, z: 0 },
    size: 2,
    mass: 1,
    linearVelocity: { x: 0, y: 0, z: 6 }
  });
  world.createHingeJoint({
    id: 'door-hinge',
    bodyAId: 'frame',
    bodyBId: 'door',
    worldAnchor: { x: 1, y: 0, z: 0 },
    worldAxis: { x: 0, y: 1, z: 0 }
  });

  let maxSolvedJointCount = 0;
  for (let stepIndex = 0; stepIndex < 180; stepIndex += 1) {
    world.step(1 / 120);
    maxSolvedJointCount = Math.max(maxSolvedJointCount, world.getSnapshot().lastSolverStats.solvedJointCount);
  }

  const anchors = world.getJointWorldAnchors('door-hinge');
  const axes = world.getJointWorldAxes('door-hinge');
  const anchorError = Math.sqrt(
    (anchors.anchorB.x - anchors.anchorA.x) ** 2 +
    (anchors.anchorB.y - anchors.anchorA.y) ** 2 +
    (anchors.anchorB.z - anchors.anchorA.z) ** 2
  );
  const axisAlignment = axes.axisA.x * axes.axisB.x + axes.axisA.y * axes.axisB.y + axes.axisA.z * axes.axisB.z;
  const frame = world.buildDebugFrame();

  assert.ok(anchorError < 0.25, `expected hinge pivot to stay together, got ${anchorError}`);
  assert.ok(axisAlignment > 0.95, `expected hinge axes to stay aligned, got ${axisAlignment}`);
  assert.ok(Math.abs(world.getBody('door').position.x - 1) < 1.3, `expected door to orbit near hinge radius, got ${world.getBody('door').position.x}`);
  assert.ok(maxSolvedJointCount > 0);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'hinge-axis-a'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'hinge-axis-b'));
});

test('PhysicsWorld hinge-connected sleeping island wakes through the hinge link', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 14,
    gravity: { x: 0, y: 0, z: 0 },
    sleepTimeThreshold: 0.2
  });
  world.createBoxBody({
    id: 'hinge-frame',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'hinge-door',
    position: { x: 2, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createHingeJoint({
    id: 'sleep-hinge',
    bodyAId: 'hinge-frame',
    bodyBId: 'hinge-door',
    worldAnchor: { x: 1, y: 0, z: 0 },
    worldAxis: { x: 0, y: 1, z: 0 }
  });

  for (let stepIndex = 0; stepIndex < 80; stepIndex += 1) {
    world.step(1 / 120);
  }

  assert.equal(world.getBody('hinge-door').sleeping, true);

  const door = world.bodyRegistry.getMutable('hinge-door');
  door.linearVelocity.z = 4;
  world.wakeBody('hinge-door');
  world.step(1 / 120);

  assert.equal(world.getBody('hinge-door').sleeping, false);
  assert.ok(world.getSnapshot().lastSolverStats.solvedJointCount > 0);
});

test('PhysicsWorld distance joint spring and limits pull a stretched body toward its rest range', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 18,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'spring-anchor',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'spring-bob',
    position: { x: 8, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createDistanceJoint({
    id: 'spring-link',
    bodyAId: 'spring-anchor',
    bodyBId: 'spring-bob',
    distance: 4
  });
  world.configureDistanceJoint('spring-link', {
    minDistance: 3.5,
    maxDistance: 4.5,
    springFrequency: 3,
    dampingRatio: 1
  });

  let minSeparation = Infinity;
  let maxSolvedJointCount = 0;
  for (let stepIndex = 0; stepIndex < 240; stepIndex += 1) {
    world.step(1 / 120);
    const bob = world.getBody('spring-bob');
    minSeparation = Math.min(minSeparation, Math.abs(bob.position.x));
    maxSolvedJointCount = Math.max(maxSolvedJointCount, world.getSnapshot().lastSolverStats.solvedJointCount);
  }

  const joint = world.getJoint('spring-link');
  const bob = world.getBody('spring-bob');
  const separation = Math.sqrt((bob.position.x ** 2) + (bob.position.y ** 2) + (bob.position.z ** 2));

  assert.ok(minSeparation < 5.25, `expected spring to pull the body inward, got ${minSeparation}`);
  assert.ok(separation >= 3.2 && separation <= 4.8, `expected final separation to stay near the configured range, got ${separation}`);
  assert.equal(joint.minDistance, 3.5);
  assert.equal(joint.maxDistance, 4.5);
  assert.equal(joint.springFrequency, 3);
  assert.equal(joint.dampingRatio, 1);
  assert.ok(maxSolvedJointCount > 0);
});

test('PhysicsWorld hinge joint angular limits push an over-rotated door back toward the allowed range', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 24,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'limit-frame',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'limit-door',
    position: { x: 2, y: 0, z: 0 },
    rotation: createQuatFromAxisAngle({ x: 0, y: 1, z: 0 }, 1.1),
    size: 2,
    mass: 1
  });
  world.createHingeJoint({
    id: 'limit-hinge',
    bodyAId: 'limit-frame',
    bodyBId: 'limit-door',
    worldAnchor: { x: 1, y: 0, z: 0 },
    worldAxis: { x: 0, y: 1, z: 0 },
    localReferenceA: { x: 0, y: 0, z: 1 },
    localReferenceB: { x: 0, y: 0, z: 1 }
  });
  world.configureHingeJoint('limit-hinge', {
    lowerAngle: -0.3,
    upperAngle: 0.3,
    angularDamping: 6
  });

  let maxSolvedJointCount = 0;
  for (let stepIndex = 0; stepIndex < 480; stepIndex += 1) {
    world.step(1 / 120);
    maxSolvedJointCount = Math.max(maxSolvedJointCount, world.getSnapshot().lastSolverStats.solvedJointCount);
  }

  const hingeAngle = world.getJointAngle('limit-hinge');
  const joint = world.getJoint('limit-hinge');

  assert.ok(Math.abs(hingeAngle) < 0.45, `expected hinge limit to reduce the angle, got ${hingeAngle}`);
  assert.equal(joint.lowerAngle, -0.3);
  assert.equal(joint.upperAngle, 0.3);
  assert.ok(maxSolvedJointCount > 0);
});

test('PhysicsWorld hinge joint angular damping reduces spin around the hinge axis', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 16,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'damping-frame',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'damping-door',
    position: { x: 2, y: 0, z: 0 },
    size: 2,
    mass: 1,
    angularVelocity: { x: 0, y: 10, z: 0 }
  });
  world.createHingeJoint({
    id: 'damping-hinge',
    bodyAId: 'damping-frame',
    bodyBId: 'damping-door',
    worldAnchor: { x: 1, y: 0, z: 0 },
    worldAxis: { x: 0, y: 1, z: 0 }
  });
  world.configureHingeJoint('damping-hinge', {
    lowerAngle: -Math.PI,
    upperAngle: Math.PI,
    angularDamping: 10
  });

  const initialSpin = Math.abs(world.getBody('damping-door').angularVelocity.y);
  for (let stepIndex = 0; stepIndex < 120; stepIndex += 1) {
    world.step(1 / 120);
  }

  const finalSpin = Math.abs(world.getBody('damping-door').angularVelocity.y);
  assert.ok(finalSpin < initialSpin * 0.65, `expected hinge damping to reduce spin, got ${finalSpin} from ${initialSpin}`);
});

test('PhysicsWorld fixed joint keeps anchors and orientation frames aligned', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 22,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'fixed-frame',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'fixed-child',
    position: { x: 2, y: 0, z: 0 },
    rotation: createQuatFromAxisAngle({ x: 0, y: 0, z: 1 }, 0.45),
    size: 2,
    mass: 1,
    linearVelocity: { x: 0, y: 0, z: 4 },
    angularVelocity: { x: 0, y: 2, z: 0 }
  });
  world.createFixedJoint({
    id: 'fixed-link',
    bodyAId: 'fixed-frame',
    bodyBId: 'fixed-child',
    worldAnchor: { x: 1, y: 0, z: 0 }
  });

  let maxSolvedJointCount = 0;
  for (let stepIndex = 0; stepIndex < 180; stepIndex += 1) {
    world.step(1 / 120);
    maxSolvedJointCount = Math.max(maxSolvedJointCount, world.getSnapshot().lastSolverStats.solvedJointCount);
  }

  const anchors = world.getJointWorldAnchors('fixed-link');
  const axes = world.getJointWorldAxes('fixed-link');
  const references = world.getJointWorldReferences('fixed-link');
  const anchorError = Math.sqrt(
    (anchors.anchorB.x - anchors.anchorA.x) ** 2 +
    (anchors.anchorB.y - anchors.anchorA.y) ** 2 +
    (anchors.anchorB.z - anchors.anchorA.z) ** 2
  );
  const axisAlignment = dotVec3(axes.axisA, axes.axisB);
  const referenceAlignment = dotVec3(references.referenceA, references.referenceB);
  const collisionState = world.getCollisionState();
  const island = collisionState.islands.find((entry) => entry.jointIds.includes('fixed-link'));
  const frame = world.buildDebugFrame();

  assert.ok(anchorError < 0.4, `expected fixed joint anchors to stay together, got ${anchorError}`);
  assert.ok(axisAlignment > 0.95, `expected fixed joint axes to align, got ${axisAlignment}`);
  assert.ok(referenceAlignment > 0.95, `expected fixed joint references to align, got ${referenceAlignment}`);
  assert.ok(island);
  assert.ok(island.bodyIds.includes('fixed-child'));
  assert.ok(island.jointIds.includes('fixed-link'));
  assert.ok(maxSolvedJointCount > 0);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'fixed-joint'));
});

test('PhysicsWorld hinge motor drives angular motion about the hinge axis', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 18,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'motor-frame',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'motor-door',
    position: { x: 2, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createHingeJoint({
    id: 'motor-hinge',
    bodyAId: 'motor-frame',
    bodyBId: 'motor-door',
    worldAnchor: { x: 1, y: 0, z: 0 },
    worldAxis: { x: 0, y: 1, z: 0 }
  });
  world.configureHingeMotor('motor-hinge', {
    motorEnabled: true,
    motorSpeed: Math.PI * 2,
    maxMotorTorque: 80
  });

  let maxSolvedJointCount = 0;
  let maxAngularVelocity = 0;
  let maxAbsoluteAngle = 0;
  let observedConnectedCollision = false;
  for (let stepIndex = 0; stepIndex < 240; stepIndex += 1) {
    world.step(1 / 120);
    maxSolvedJointCount = Math.max(maxSolvedJointCount, world.getSnapshot().lastSolverStats.solvedJointCount);
    maxAngularVelocity = Math.max(maxAngularVelocity, Math.abs(world.getBody('motor-door').angularVelocity.y));
    maxAbsoluteAngle = Math.max(maxAbsoluteAngle, Math.abs(world.getJointAngle('motor-hinge') ?? 0));
    observedConnectedCollision = observedConnectedCollision ||
      world.getSnapshot().collision.contactPairs.some((pair) => pair.pairKey === 'motor-door:collider|motor-frame:collider');
  }

  const hingeAngle = world.getJointAngle('motor-hinge');
  const joint = world.getJoint('motor-hinge');
  const door = world.getBody('motor-door');
  const frame = world.buildDebugFrame();

  assert.ok(maxAbsoluteAngle > 0.12, `expected hinge motor to rotate the door, got max |angle| ${maxAbsoluteAngle}`);
  assert.ok(maxAngularVelocity > 0.5, `expected hinge motor to affect angular velocity, got max ${maxAngularVelocity}`);
  assert.equal(joint.motorEnabled, true);
  assert.equal(observedConnectedCollision, false);
  assert.ok(maxSolvedJointCount > 0);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'hinge-motor'));
});

test('PhysicsWorld fixed joint break thresholds disable the joint under high load', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 20,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'break-frame',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'break-child',
    position: { x: 2, y: 0, z: 0 },
    size: 2,
    mass: 1,
    linearVelocity: { x: 0, y: 0, z: 16 }
  });
  world.createFixedJoint({
    id: 'break-link',
    bodyAId: 'break-frame',
    bodyBId: 'break-child',
    worldAnchor: { x: 1, y: 0, z: 0 }
  });
  world.configureFixedJoint('break-link', {
    breakForce: 200,
    breakTorque: 120
  });

  let observedBreak = false;
  for (let stepIndex = 0; stepIndex < 60; stepIndex += 1) {
    world.step(1 / 120);
    if (world.getJoint('break-link').broken) {
      observedBreak = true;
      break;
    }
  }

  const joint = world.getJoint('break-link');

  assert.equal(observedBreak, true);
  assert.equal(joint.enabled, false);
  assert.equal(joint.broken, true);
  assert.ok(joint.lastAppliedForce > 0 || joint.lastAppliedTorque > 0, `expected a recorded breaking load, got ${joint.lastAppliedForce}/${joint.lastAppliedTorque}`);
});

test('PhysicsWorld hinge servo drives the joint toward a target angle', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    solverIterations: 20,
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'servo-frame',
    motionType: 'static',
    position: { x: 0, y: 0, z: 0 },
    size: 2
  });
  world.createBoxBody({
    id: 'servo-door',
    position: { x: 2, y: 0, z: 0 },
    size: 2,
    mass: 1
  });
  world.createHingeJoint({
    id: 'servo-hinge',
    bodyAId: 'servo-frame',
    bodyBId: 'servo-door',
    worldAnchor: { x: 1, y: 0, z: 0 },
    worldAxis: { x: 0, y: 1, z: 0 }
  });
  world.configureHingeServo('servo-hinge', {
    motorEnabled: true,
    motorTargetAngle: Math.PI / 4,
    maxMotorSpeed: Math.PI * 8,
    maxMotorTorque: 400
  });

  let maxSolvedJointCount = 0;
  for (let stepIndex = 0; stepIndex < 240; stepIndex += 1) {
    world.step(1 / 120);
    maxSolvedJointCount = Math.max(maxSolvedJointCount, world.getSnapshot().lastSolverStats.solvedJointCount);
  }

  const joint = world.getJoint('servo-hinge');
  const hingeAngle = world.getJointAngle('servo-hinge');
  const frame = world.buildDebugFrame();

  assert.equal(joint.motorMode, 'servo');
  assert.ok(hingeAngle > 0.25, `expected servo to rotate toward the target, got ${hingeAngle}`);
  assert.ok(Math.abs(hingeAngle - (Math.PI / 4)) < 0.5, `expected servo angle near target, got ${hingeAngle}`);
  assert.ok(maxSolvedJointCount > 0);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'hinge-motor'));
});
