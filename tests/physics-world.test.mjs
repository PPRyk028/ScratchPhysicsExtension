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

test('PhysicsWorld creates a cloth sheet with pinned top row debug primitives', () => {
  const world = new PhysicsWorld();
  const cloth = world.createClothSheet({
    id: 'cloth-1',
    rows: 4,
    columns: 5,
    spacing: 12,
    position: { x: -24, y: 60, z: 0 },
    pinMode: 'top-row'
  });

  const snapshot = world.getSnapshot();
  const frame = world.buildDebugFrame();

  assert.equal(cloth.rows, 4);
  assert.equal(cloth.columns, 5);
  assert.equal(cloth.particleIds.length, 20);
  assert.equal(snapshot.clothCount, 1);
  assert.equal(snapshot.particleCount, 20);
  assert.equal(frame.stats.clothCount, 1);
  assert.equal(frame.stats.particleCount, 20);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'cloth-structural-edge'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'cloth-pin'));
});

test('PhysicsWorld XPBD cloth keeps pinned particles fixed while lower particles sag', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120
  });
  world.createClothSheet({
    id: 'cloth-1',
    rows: 5,
    columns: 6,
    spacing: 10,
    position: { x: 0, y: 80, z: 0 },
    pinMode: 'top-row'
  });

  const before = world.getSnapshot().xpbd.particles;
  const topLeftBefore = before.find((particle) => particle.id === 'cloth-1:particle:0:0');
  const bottomBefore = before.find((particle) => particle.id === 'cloth-1:particle:4:3');

  for (let index = 0; index < 60; index += 1) {
    world.step(1 / 120);
  }

  const after = world.getSnapshot().xpbd.particles;
  const topLeftAfter = after.find((particle) => particle.id === 'cloth-1:particle:0:0');
  const bottomAfter = after.find((particle) => particle.id === 'cloth-1:particle:4:3');

  assert.deepEqual(topLeftAfter.position, topLeftBefore.position);
  assert.ok(bottomAfter.position.y < bottomBefore.position.y - 0.5, `expected bottom particle to sag, got before ${bottomBefore.position.y} after ${bottomAfter.position.y}`);
  assert.ok(world.getSnapshot().lastXpbdStats.solvedDistanceConstraints > 0);
});

test('PhysicsWorld updates cloth parameters and reflects them in the cloth record', () => {
  const world = new PhysicsWorld();
  world.createClothSheet({
    id: 'cloth-1',
    rows: 4,
    columns: 5,
    spacing: 12,
    position: { x: -24, y: 60, z: 0 },
    pinMode: 'top-row'
  });

  const cloth = world.configureCloth('cloth-1', {
    damping: 0.08,
    collisionMargin: 4,
    stretchCompliance: 0.001,
    shearCompliance: 0.002,
    bendCompliance: 0.003,
    selfCollisionEnabled: true,
    selfCollisionDistance: 9
  });

  assert.ok(cloth);
  assert.equal(cloth.damping, 0.08);
  assert.equal(cloth.collisionMargin, 4);
  assert.equal(cloth.stretchCompliance, 0.001);
  assert.equal(cloth.shearCompliance, 0.002);
  assert.equal(cloth.bendCompliance, 0.003);
  assert.equal(cloth.selfCollisionEnabled, true);
  assert.equal(cloth.selfCollisionDistance, 9);
  assert.ok(cloth.distanceConstraints.filter((constraint) => constraint.kind === 'stretch').every((constraint) => constraint.compliance === 0.001));
  assert.ok(cloth.distanceConstraints.filter((constraint) => constraint.kind === 'shear').every((constraint) => constraint.compliance === 0.002));
  assert.ok(cloth.distanceConstraints.filter((constraint) => constraint.kind === 'bend').every((constraint) => constraint.compliance === 0.003));
});

test('PhysicsWorld XPBD cloth self-collision separates overlapping non-neighbor particles', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    gravity: { x: 0, y: 0, z: 0 },
    xpbdIterations: 8
  });
  world.createClothSheet({
    id: 'cloth-1',
    rows: 4,
    columns: 4,
    spacing: 10,
    position: { x: 0, y: 40, z: 0 },
    pinMode: 'none'
  });
  world.configureCloth('cloth-1', {
    selfCollisionEnabled: true,
    selfCollisionDistance: 8,
    damping: 0
  });

  const particleA = world.particleWorld.particles.get('cloth-1:particle:0:0');
  const particleB = world.particleWorld.particles.get('cloth-1:particle:3:3');
  particleA.position = { x: 10, y: 40, z: 10 };
  particleA.predictedPosition = { x: 10, y: 40, z: 10 };
  particleA.previousPosition = { x: 10, y: 40, z: 10 };
  particleA.velocity = { x: 0, y: 0, z: 0 };
  particleB.position = { x: 10, y: 40, z: 10 };
  particleB.predictedPosition = { x: 10, y: 40, z: 10 };
  particleB.previousPosition = { x: 10, y: 40, z: 10 };
  particleB.velocity = { x: 0, y: 0, z: 0 };

  world.step(1 / 120);

  const afterA = world.particleWorld.particles.get('cloth-1:particle:0:0');
  const afterB = world.particleWorld.particles.get('cloth-1:particle:3:3');
  const separation = Math.sqrt(
    (afterA.position.x - afterB.position.x) ** 2 +
    (afterA.position.y - afterB.position.y) ** 2 +
    (afterA.position.z - afterB.position.z) ** 2
  );
  const snapshot = world.getSnapshot();
  const frame = world.buildDebugFrame();

  assert.ok(snapshot.lastXpbdStats.solvedSelfCollisions > 0, 'expected self-collision solver to run');
  assert.ok(snapshot.xpbd.lastSelfCollisionContacts.length > 0, 'expected self-collision contacts to be recorded');
  assert.ok(separation >= 7.5, `expected self-collision to separate particles, got ${separation}`);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'cloth-self-contact-point'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'cloth-self-contact-normal'));
});

test('PhysicsWorld XPBD cloth collides with static box colliders', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    xpbdIterations: 8,
    xpbdSubsteps: 2
  });
  world.createStaticBoxCollider({
    id: 'support-box',
    position: { x: 0, y: 48, z: 20 },
    size: 50
  });
  const cloth = world.createClothSheet({
    id: 'cloth-1',
    rows: 5,
    columns: 5,
    spacing: 10,
    position: { x: -20, y: 80, z: 0 },
    pinMode: 'top-row'
  });

  for (let index = 0; index < 180; index += 1) {
    world.step(1 / 120);
  }

  const snapshot = world.getSnapshot();
  const centerParticle = snapshot.xpbd.particles.find((particle) => particle.id === 'cloth-1:particle:2:2');
  const topFaceY = 48 + 25;
  const frame = world.buildDebugFrame();

  assert.ok(centerParticle, 'expected center cloth particle to exist');
  assert.ok(centerParticle.position.y >= topFaceY - 0.25, `expected cloth to stay above the support box top face, got ${centerParticle.position.y}`);
  assert.ok(snapshot.lastXpbdStats.solvedStaticCollisions > 0, 'expected cloth step to solve at least one static collision');
  assert.ok(snapshot.xpbd.lastCollisionContacts.some((contact) => contact.colliderId === 'support-box:collider' && contact.shapeType === 'box'));
  assert.equal(cloth.collisionMargin, 1.5);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'cloth-contact-point'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'cloth-contact-normal'));
});

test('PhysicsWorld XPBD cloth collides with static convex hull colliders', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    xpbdIterations: 8,
    xpbdSubsteps: 2
  });
  world.createStaticConvexHullCollider({
    id: 'support-hull',
    position: { x: 0, y: 48, z: 20 },
    vertices: [
      { x: -25, y: -25, z: -25 },
      { x: 25, y: -25, z: -25 },
      { x: 25, y: -25, z: 25 },
      { x: -25, y: -25, z: 25 },
      { x: -25, y: 25, z: -25 },
      { x: 25, y: 25, z: -25 },
      { x: 25, y: 25, z: 25 },
      { x: -25, y: 25, z: 25 }
    ]
  });
  world.createClothSheet({
    id: 'cloth-1',
    rows: 5,
    columns: 5,
    spacing: 10,
    position: { x: -20, y: 80, z: 0 },
    pinMode: 'top-row'
  });

  for (let index = 0; index < 180; index += 1) {
    world.step(1 / 120);
  }

  const snapshot = world.getSnapshot();
  const centerParticle = snapshot.xpbd.particles.find((particle) => particle.id === 'cloth-1:particle:2:2');
  const topFaceY = 48 + 25;
  const hullContacts = snapshot.xpbd.lastCollisionContacts.filter((contact) => contact.colliderId === 'support-hull:collider');

  assert.ok(centerParticle, 'expected center cloth particle to exist');
  assert.ok(centerParticle.position.y >= topFaceY - 0.25, `expected cloth to stay above the support hull top face, got ${centerParticle.position.y}`);
  assert.ok(hullContacts.length > 0, 'expected cloth to register convex-hull collision contacts');
  assert.ok(hullContacts.every((contact) => contact.shapeType === 'convex-hull'));
  assert.ok(snapshot.lastXpbdStats.solvedStaticCollisions > 0, 'expected cloth step to solve at least one static collision');
});

test('PhysicsWorld XPBD cloth collides with a dynamic box rigid body and slows it down', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    gravity: { x: 0, y: 0, z: 0 },
    xpbdIterations: 8,
    xpbdSubsteps: 2
  });
  world.createBoxBody({
    id: 'box-body',
    position: { x: 0, y: 40, z: 20 },
    size: 40,
    mass: 1,
    linearVelocity: { x: 0, y: 20, z: 0 }
  });
  world.createClothSheet({
    id: 'cloth-1',
    rows: 5,
    columns: 5,
    spacing: 10,
    position: { x: -20, y: 62, z: 0 },
    pinMode: 'top-row'
  });

  let sawDynamicCollision = false;
  let sawBoxContact = false;
  for (let index = 0; index < 90; index += 1) {
    world.step(1 / 120);
    const snapshot = world.getSnapshot();
    const stepStats = snapshot.lastXpbdStats;
    sawDynamicCollision = sawDynamicCollision || stepStats.solvedDynamicCollisions > 0;
    sawBoxContact = sawBoxContact || snapshot.xpbd.lastCollisionContacts.some((contact) => (
      contact.colliderId === 'box-body:collider' &&
      contact.motionType === 'dynamic' &&
      contact.bodyId === 'box-body'
    ));
  }

  const snapshot = world.getSnapshot();
  const body = world.getBody('box-body');
  assert.ok(sawDynamicCollision, 'expected cloth step to solve dynamic collisions');
  assert.ok(sawBoxContact, 'expected cloth to register contacts against the dynamic box');
  assert.ok(body.linearVelocity.y < 20, `expected cloth to reduce upward box velocity, got ${body.linearVelocity.y}`);
});

test('PhysicsWorld XPBD cloth collides with a dynamic convex hull rigid body and slows it down', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    gravity: { x: 0, y: 0, z: 0 },
    xpbdIterations: 8,
    xpbdSubsteps: 2
  });
  world.createConvexHullBody({
    id: 'hull-body',
    position: { x: 0, y: 40, z: 20 },
    mass: 1,
    linearVelocity: { x: 0, y: 18, z: 0 },
    vertices: [
      { x: -20, y: -20, z: -20 },
      { x: 20, y: -20, z: -20 },
      { x: 20, y: -20, z: 20 },
      { x: -20, y: -20, z: 20 },
      { x: -20, y: 20, z: -20 },
      { x: 20, y: 20, z: -20 },
      { x: 20, y: 20, z: 20 },
      { x: -20, y: 20, z: 20 }
    ]
  });
  world.createClothSheet({
    id: 'cloth-1',
    rows: 5,
    columns: 5,
    spacing: 10,
    position: { x: -20, y: 62, z: 0 },
    pinMode: 'top-row'
  });

  let sawDynamicCollision = false;
  let sawHullContact = false;
  for (let index = 0; index < 90; index += 1) {
    world.step(1 / 120);
    const snapshot = world.getSnapshot();
    sawDynamicCollision = sawDynamicCollision || snapshot.lastXpbdStats.solvedDynamicCollisions > 0;
    sawHullContact = sawHullContact || snapshot.xpbd.lastCollisionContacts.some((contact) => (
      contact.colliderId === 'hull-body:collider' &&
      contact.motionType === 'dynamic' &&
      contact.shapeType === 'convex-hull'
    ));
  }

  const snapshot = world.getSnapshot();
  const body = world.getBody('hull-body');

  assert.ok(sawDynamicCollision, 'expected cloth step to solve dynamic collisions');
  assert.ok(sawHullContact, 'expected cloth to register contacts against the dynamic convex hull');
  assert.ok(body.linearVelocity.y < 18, `expected cloth to reduce upward hull velocity, got ${body.linearVelocity.y}`);
});

test('PhysicsWorld creates a soft body cube with pinned top layer debug primitives', () => {
  const world = new PhysicsWorld();
  const softBody = world.createSoftBodyCube({
    id: 'soft-1',
    rows: 3,
    columns: 4,
    layers: 3,
    spacing: 12,
    position: { x: -18, y: 72, z: -12 },
    pinMode: 'top-layer'
  });

  const snapshot = world.getSnapshot();
  const frame = world.buildDebugFrame();

  assert.equal(softBody.layers, 3);
  assert.equal(softBody.rows, 3);
  assert.equal(softBody.columns, 4);
  assert.equal(softBody.particleIds.length, 36);
  assert.equal(snapshot.softBodyCount, 1);
  assert.equal(frame.stats.softBodyCount, 1);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'soft-body-structural-edge'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'soft-body-pin'));
});

test('PhysicsWorld updates soft body parameters and reflects them in the soft body record', () => {
  const world = new PhysicsWorld();
  world.createSoftBodyCube({
    id: 'soft-1',
    rows: 3,
    columns: 3,
    layers: 3,
    spacing: 10,
    position: { x: 0, y: 60, z: 0 },
    pinMode: 'none'
  });

  const softBody = world.configureSoftBody('soft-1', {
    damping: 0.08,
    collisionMargin: 4,
    stretchCompliance: 0.001,
    shearCompliance: 0.002,
    bendCompliance: 0.003,
    volumeCompliance: 0.00012
  });

  assert.ok(softBody);
  assert.equal(softBody.damping, 0.08);
  assert.equal(softBody.collisionMargin, 4);
  assert.equal(softBody.stretchCompliance, 0.001);
  assert.equal(softBody.shearCompliance, 0.002);
  assert.equal(softBody.bendCompliance, 0.003);
  assert.equal(softBody.volumeCompliance, 0.00012);
  assert.ok(softBody.distanceConstraints.filter((constraint) => constraint.kind === 'stretch').every((constraint) => constraint.compliance === 0.001));
  assert.ok(softBody.distanceConstraints.filter((constraint) => constraint.kind === 'shear').every((constraint) => constraint.compliance === 0.002));
  assert.ok(softBody.distanceConstraints.filter((constraint) => constraint.kind === 'bend').every((constraint) => constraint.compliance === 0.003));
  assert.ok(softBody.volumeConstraints.every((constraint) => constraint.compliance === 0.00012));
});

test('PhysicsWorld XPBD soft body collides with static box colliders', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    xpbdIterations: 8,
    xpbdSubsteps: 2
  });
  world.createStaticBoxCollider({
    id: 'support-box',
    position: { x: 0, y: 45, z: 0 },
    size: 30
  });
  world.createSoftBodyCube({
    id: 'soft-1',
    rows: 3,
    columns: 3,
    layers: 3,
    spacing: 10,
    position: { x: -10, y: 90, z: -10 },
    pinMode: 'none'
  });

  for (let index = 0; index < 220; index += 1) {
    world.step(1 / 120);
  }

  const snapshot = world.getSnapshot();
  const bottomParticles = snapshot.xpbd.particles.filter((particle) => particle.softBodyId === 'soft-1' && particle.layerIndex === 2);
  const minimumBottomY = Math.min(...bottomParticles.map((particle) => particle.position.y));
  const topFaceY = 45 + 15;
  const frame = world.buildDebugFrame();

  assert.ok(bottomParticles.length > 0, 'expected bottom-layer soft body particles');
  assert.ok(minimumBottomY >= topFaceY - 2.5, `expected soft body to stay above the support box top face, got ${minimumBottomY}`);
  assert.ok(snapshot.lastXpbdStats.solvedStaticCollisions > 0, 'expected soft body step to solve static collisions');
  assert.ok(snapshot.xpbd.lastCollisionContacts.some((contact) => contact.softBodyId === 'soft-1' && contact.colliderId === 'support-box:collider'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'soft-body-contact-point'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'soft-body-contact-normal'));
});

test('PhysicsWorld XPBD soft body volume constraints resist collapse under compression', () => {
  const createCompressedWorld = (volumeCompliance) => {
    const world = new PhysicsWorld({
      fixedDeltaTime: 1 / 120,
      gravity: { x: 0, y: 0, z: 0 },
      xpbdIterations: 8,
      xpbdSubsteps: 2
    });
    world.createSoftBodyCube({
      id: 'soft-1',
      rows: 3,
      columns: 3,
      layers: 3,
      spacing: 10,
      position: { x: -10, y: 40, z: -10 },
      pinMode: 'none',
      stretchCompliance: 0.2,
      shearCompliance: 0.2,
      bendCompliance: 0.2,
      volumeCompliance
    });

    for (const particleId of world.getSoftBody('soft-1').particleIds) {
      const particle = world.particleWorld.particles.get(particleId);
      const layerPosition = Number(particle.layerIndex ?? 0);
      particle.position.y = 40 - layerPosition * 3.2;
      particle.previousPosition = { ...particle.position };
      particle.predictedPosition = { ...particle.position };
      particle.velocity = { x: 0, y: 0, z: 0 };
    }

    return world;
  };

  const withoutVolume = createCompressedWorld(10);
  const withVolume = createCompressedWorld(0.00005);

  for (let stepIndex = 0; stepIndex < 60; stepIndex += 1) {
    withoutVolume.step(1 / 120);
    withVolume.step(1 / 120);
  }

  const getHeight = (world) => {
    const particles = world.getSnapshot().xpbd.particles.filter((particle) => particle.softBodyId === 'soft-1');
    const ys = particles.map((particle) => particle.position.y);
    return Math.max(...ys) - Math.min(...ys);
  };

  const heightWithoutVolume = getHeight(withoutVolume);
  const heightWithVolume = getHeight(withVolume);

  assert.ok(withVolume.getSnapshot().lastXpbdStats.solvedVolumeConstraints > 0, 'expected volume constraints to run');
  assert.ok(heightWithVolume > heightWithoutVolume + 1.5, `expected volume-preserving soft body to recover more height, got ${heightWithVolume} vs ${heightWithoutVolume}`);
});

test('PhysicsWorld XPBD soft body collides with a dynamic box rigid body and slows it down', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    gravity: { x: 0, y: 0, z: 0 },
    xpbdIterations: 8,
    xpbdSubsteps: 2
  });
  world.createBoxBody({
    id: 'box-body',
    position: { x: 0, y: 30, z: 0 },
    size: 36,
    mass: 1,
    linearVelocity: { x: 0, y: 16, z: 0 }
  });
  world.createSoftBodyCube({
    id: 'soft-1',
    rows: 3,
    columns: 3,
    layers: 3,
    spacing: 10,
    position: { x: -10, y: 70, z: -10 },
    pinMode: 'top-layer'
  });

  let sawDynamicCollision = false;
  let sawBoxContact = false;
  let maxAffectedBodies = 0;
  for (let index = 0; index < 120; index += 1) {
    world.step(1 / 120);
    const snapshot = world.getSnapshot();
    sawDynamicCollision = sawDynamicCollision || snapshot.lastXpbdStats.solvedDynamicCollisions > 0;
    sawBoxContact = sawBoxContact || snapshot.xpbd.lastCollisionContacts.some((contact) => (
      contact.softBodyId === 'soft-1' &&
      contact.colliderId === 'box-body:collider' &&
      contact.motionType === 'dynamic' &&
      contact.bodyId === 'box-body'
    ));
    maxAffectedBodies = Math.max(maxAffectedBodies, snapshot.lastXpbdStats.affectedDynamicBodyCount ?? 0);
  }

  const body = world.getBody('box-body');

  assert.ok(sawDynamicCollision, 'expected soft body step to solve dynamic collisions');
  assert.ok(sawBoxContact, 'expected soft body to register contacts against the dynamic box');
  assert.ok(maxAffectedBodies > 0, 'expected soft body collisions to affect at least one dynamic body');
  assert.ok(body.linearVelocity.y < 16, `expected soft body to reduce upward box velocity, got ${body.linearVelocity.y}`);
});

test('PhysicsWorld XPBD soft body collides with a dynamic convex hull rigid body and slows it down', () => {
  const world = new PhysicsWorld({
    fixedDeltaTime: 1 / 120,
    gravity: { x: 0, y: 0, z: 0 },
    xpbdIterations: 8,
    xpbdSubsteps: 2
  });
  world.createConvexHullBody({
    id: 'hull-body',
    position: { x: 0, y: 30, z: 0 },
    mass: 1,
    linearVelocity: { x: 0, y: 14, z: 0 },
    vertices: [
      { x: -18, y: -18, z: -18 },
      { x: 18, y: -18, z: -18 },
      { x: 18, y: -18, z: 18 },
      { x: -18, y: -18, z: 18 },
      { x: -18, y: 18, z: -18 },
      { x: 18, y: 18, z: -18 },
      { x: 18, y: 18, z: 18 },
      { x: -18, y: 18, z: 18 }
    ]
  });
  world.createSoftBodyCube({
    id: 'soft-1',
    rows: 3,
    columns: 3,
    layers: 3,
    spacing: 10,
    position: { x: -10, y: 70, z: -10 },
    pinMode: 'top-layer'
  });

  let sawDynamicCollision = false;
  let sawHullContact = false;
  let maxAffectedBodies = 0;
  for (let index = 0; index < 120; index += 1) {
    world.step(1 / 120);
    const snapshot = world.getSnapshot();
    sawDynamicCollision = sawDynamicCollision || snapshot.lastXpbdStats.solvedDynamicCollisions > 0;
    sawHullContact = sawHullContact || snapshot.xpbd.lastCollisionContacts.some((contact) => (
      contact.softBodyId === 'soft-1' &&
      contact.colliderId === 'hull-body:collider' &&
      contact.motionType === 'dynamic' &&
      contact.shapeType === 'convex-hull'
    ));
    maxAffectedBodies = Math.max(maxAffectedBodies, snapshot.lastXpbdStats.affectedDynamicBodyCount ?? 0);
  }

  const body = world.getBody('hull-body');

  assert.ok(sawDynamicCollision, 'expected soft body step to solve dynamic collisions');
  assert.ok(sawHullContact, 'expected soft body to register contacts against the dynamic convex hull');
  assert.ok(maxAffectedBodies > 0, 'expected soft body collisions to affect at least one dynamic body');
  assert.ok(body.linearVelocity.y < 14, `expected soft body to reduce upward hull velocity, got ${body.linearVelocity.y}`);
});

test('PhysicsWorld XPBD soft body friction against a static box reduces tangential drift', () => {
  const simulateSoftBodySlide = (friction) => {
    const world = new PhysicsWorld({
      fixedDeltaTime: 1 / 120,
      xpbdIterations: 8,
      xpbdSubsteps: 2
    });
    world.createMaterial({
      id: 'surface',
      friction,
      restitution: 0
    });
    world.createStaticBoxCollider({
      id: 'support-box',
      position: { x: 0, y: 48, z: 20 },
      size: 50,
      materialId: 'surface'
    });
    world.createSoftBodyCube({
      id: 'soft-1',
      rows: 3,
      columns: 3,
      layers: 3,
      spacing: 10,
      position: { x: -10, y: 78, z: 0 },
      pinMode: 'none'
    });
    world.configureSoftBody('soft-1', {
      damping: 0.01,
      collisionMargin: 3,
      stretchCompliance: 0,
      shearCompliance: 0.0002,
      bendCompliance: 0.0008,
      volumeCompliance: 0.00008
    });

    const softBody = world.getSoftBody('soft-1');
    for (const particleId of softBody.particleIds) {
      const particle = world.particleWorld.particles.get(particleId);
      particle.velocity = { x: 6, y: 0, z: 0 };
    }

    let maxStaticFrictionContacts = 0;
    for (let index = 0; index < 180; index += 1) {
      world.step(1 / 120);
      maxStaticFrictionContacts = Math.max(
        maxStaticFrictionContacts,
        world.getSnapshot().lastXpbdStats.solvedStaticFrictionCollisions ?? 0
      );
    }

    const particles = world.getSnapshot().xpbd.particles.filter((particle) => particle.softBodyId === 'soft-1');
    const averageVelocityX = particles.reduce((sum, particle) => sum + particle.velocity.x, 0) / particles.length;
    const centerOfMassX = particles.reduce((sum, particle) => sum + particle.position.x, 0) / particles.length;

    return {
      averageVelocityX,
      centerOfMassX,
      maxStaticFrictionContacts
    };
  };

  const lowFriction = simulateSoftBodySlide(0);
  const highFriction = simulateSoftBodySlide(1.2);

  assert.ok(highFriction.maxStaticFrictionContacts > 0, 'expected high-friction scenario to solve tangential static contacts');
  assert.ok(Math.abs(highFriction.averageVelocityX) < Math.abs(lowFriction.averageVelocityX) - 0.2,
    `expected high friction to reduce soft body drift, got ${highFriction.averageVelocityX} vs ${lowFriction.averageVelocityX}`);
  assert.ok(highFriction.centerOfMassX < lowFriction.centerOfMassX - 1,
    `expected high friction to keep the soft body closer to the support box, got ${highFriction.centerOfMassX} vs ${lowFriction.centerOfMassX}`);
});

test('PhysicsWorld XPBD soft body friction against a dynamic box transfers tangential motion more strongly', () => {
  const simulateDynamicSoftBodyFriction = (friction) => {
    const world = new PhysicsWorld({
      fixedDeltaTime: 1 / 120,
      gravity: { x: 0, y: 0, z: 0 },
      xpbdIterations: 8,
      xpbdSubsteps: 2
    });
    world.createMaterial({
      id: 'surface',
      friction,
      restitution: 0
    });
    world.createBoxBody({
      id: 'box-body',
      position: { x: 0, y: 40, z: 20 },
      size: 40,
      mass: 1,
      linearVelocity: { x: 8, y: 10, z: 0 },
      materialId: 'surface'
    });
    world.createSoftBodyCube({
      id: 'soft-1',
      rows: 3,
      columns: 3,
      layers: 3,
      spacing: 10,
      position: { x: -10, y: 74, z: 0 },
      pinMode: 'none'
    });
    world.configureSoftBody('soft-1', {
      damping: 0.01,
      collisionMargin: 3,
      stretchCompliance: 0,
      shearCompliance: 0.0002,
      bendCompliance: 0.0008,
      volumeCompliance: 0.00008
    });

    let maxDynamicFrictionContacts = 0;
    for (let index = 0; index < 120; index += 1) {
      world.step(1 / 120);
      maxDynamicFrictionContacts = Math.max(
        maxDynamicFrictionContacts,
        world.getSnapshot().lastXpbdStats.solvedDynamicFrictionCollisions ?? 0
      );
    }

    const particles = world.getSnapshot().xpbd.particles.filter((particle) => particle.softBodyId === 'soft-1');
    const averageVelocityX = particles.reduce((sum, particle) => sum + particle.velocity.x, 0) / particles.length;
    const boxBody = world.getBody('box-body');

    return {
      averageVelocityX,
      boxVelocityX: boxBody.linearVelocity.x,
      maxDynamicFrictionContacts
    };
  };

  const lowFriction = simulateDynamicSoftBodyFriction(0);
  const highFriction = simulateDynamicSoftBodyFriction(1.1);

  assert.ok(highFriction.maxDynamicFrictionContacts > 0, 'expected high-friction scenario to solve tangential dynamic contacts');
  assert.ok(highFriction.averageVelocityX > lowFriction.averageVelocityX + 0.25,
    `expected high friction to drag the soft body along, got ${highFriction.averageVelocityX} vs ${lowFriction.averageVelocityX}`);
  assert.ok(highFriction.boxVelocityX < lowFriction.boxVelocityX - 0.25,
    `expected high friction to slow the dynamic box more strongly, got ${highFriction.boxVelocityX} vs ${lowFriction.boxVelocityX}`);
});

test('PhysicsWorld exports and imports soft body definitions with stable ids and settings', () => {
  const world = new PhysicsWorld();
  world.createSoftBodyCube({
    id: 'soft-1',
    rows: 3,
    columns: 4,
    layers: 3,
    spacing: 8,
    position: { x: -12, y: 70, z: -16 },
    pinMode: 'top-layer'
  });
  world.configureSoftBody('soft-1', {
    damping: 0.07,
    collisionMargin: 3.5,
    stretchCompliance: 0.001,
    shearCompliance: 0.002,
    bendCompliance: 0.003,
    volumeCompliance: 0.0002
  });

  const scene = world.exportSceneDefinition();
  const restoredWorld = new PhysicsWorld();
  const imported = restoredWorld.importSceneDefinition(scene, {
    reset: true
  });
  const restored = restoredWorld.getSoftBody('soft-1');

  assert.equal(imported.softBodyCount, 1);
  assert.equal(restored.layers, 3);
  assert.equal(restored.rows, 3);
  assert.equal(restored.columns, 4);
  assert.equal(restored.pinMode, 'top-layer');
  assert.equal(restored.damping, 0.07);
  assert.equal(restored.collisionMargin, 3.5);
  assert.equal(restored.stretchCompliance, 0.001);
  assert.equal(restored.shearCompliance, 0.002);
  assert.equal(restored.bendCompliance, 0.003);
  assert.equal(restored.volumeCompliance, 0.0002);
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
  world.createClothSheet({
    id: 'cloth-1',
    rows: 3,
    columns: 4,
    spacing: 8,
    position: { x: -16, y: 60, z: 0 },
    pinMode: 'top-corners'
  });
  world.configureCloth('cloth-1', {
    damping: 0.07,
    collisionMargin: 4,
    stretchCompliance: 0.001,
    shearCompliance: 0.002,
    bendCompliance: 0.003,
    selfCollisionEnabled: true,
    selfCollisionDistance: 7
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
  assert.equal(imported.clothCount, 1);
  assert.equal(imported.particleCount, 12);
  assert.equal(restoredWorld.getBody('crate').mass, 2);
  assert.equal(restoredWorld.getCollider('ramp:collider').materialId, 'ice');
  assert.equal(restoredWorld.getJoint('link').type, 'distance-joint');
  assert.equal(restoredWorld.getCloth('cloth-1').pinMode, 'top-corners');
  assert.equal(restoredWorld.getCloth('cloth-1').selfCollisionEnabled, true);
  assert.equal(restoredWorld.getCloth('cloth-1').selfCollisionDistance, 7);
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

test('PhysicsWorld sensor colliders generate trigger enter/stay/exit events without rigid contacts', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createStaticBoxSensor({
    id: 'trigger-zone',
    position: { x: 0, y: 0, z: 0 },
    size: 8,
    collisionLayer: 8,
    collisionMask: 1
  });
  world.createBoxBody({
    id: 'trigger-probe',
    position: { x: 0, y: 0, z: 0 },
    size: 4,
    mass: 1
  });

  const enterState = world.getCollisionState();
  assert.equal(enterState.summary.sensorPairCount, 1);
  assert.equal(enterState.summary.contactCount, 0);
  assert.equal(world.getColliderTriggerEvents('trigger-zone:collider', 'enter').count, 1);
  assert.equal(world.getBodyTriggerEvents('trigger-probe', 'enter').events.length, 1);

  world.markCollisionStateDirty();
  const stayState = world.getCollisionState();
  assert.equal(stayState.summary.triggerStayCount, 1);

  const probe = world.bodyRegistry.getMutable('trigger-probe');
  probe.position.x = 30;
  world.markCollisionStateDirty();
  const exitState = world.getCollisionState();
  const frame = world.buildDebugFrame();

  assert.equal(exitState.summary.sensorPairCount, 0);
  assert.equal(exitState.summary.triggerExitCount, 1);
  assert.equal(world.getColliderTriggerEvents('trigger-zone:collider', 'exit').count, 1);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'sensor-collider'));
});

test('PhysicsWorld collision layers and masks filter broadphase pairs', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createStaticBoxCollider({
    id: 'filtered-floor',
    position: { x: 0, y: 0, z: 0 },
    size: 8,
    collisionLayer: 2,
    collisionMask: 0
  });
  world.createBoxBody({
    id: 'filtered-box',
    position: { x: 0, y: 0, z: 0 },
    size: 4,
    mass: 1
  });

  const filteredState = world.getCollisionState();
  assert.equal(filteredState.summary.pairCount, 0);
  assert.equal(filteredState.summary.contactCount, 0);

  world.configureColliderCollision('filtered-floor:collider', {
    collisionMask: 1
  });
  const activeState = world.getCollisionState();

  assert.equal(activeState.summary.pairCount, 1);
  assert.equal(activeState.summary.contactCount, 1);
  assert.equal(world.getCollider('filtered-floor:collider').collisionLayer, 2);
  assert.equal(world.getCollider('filtered-floor:collider').collisionMask, 1);
});

test('PhysicsWorld contact enter/stay/exit events are cached for bodies and colliders', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createBoxBody({
    id: 'event-box-a',
    position: { x: 0, y: 0, z: 0 },
    size: 4,
    mass: 1
  });
  world.createBoxBody({
    id: 'event-box-b',
    position: { x: 2.5, y: 0, z: 0 },
    size: 4,
    mass: 1
  });

  const enterState = world.getCollisionState();
  assert.equal(enterState.summary.contactEnterCount, 1);
  assert.equal(world.getBodyContactEvents('event-box-a', 'enter').count, 1);
  assert.equal(world.getColliderContactEvents('event-box-a:collider', 'enter').count, 1);

  world.markCollisionStateDirty();
  const stayState = world.getCollisionState();
  assert.equal(stayState.summary.contactStayCount, 1);

  const boxB = world.bodyRegistry.getMutable('event-box-b');
  boxB.position.x = 20;
  world.markCollisionStateDirty();
  const exitState = world.getCollisionState();

  assert.equal(exitState.summary.contactExitCount, 1);
  assert.equal(world.getBodyContactEvents('event-box-a', 'exit').count, 1);
  assert.equal(world.getColliderContactEvents('event-box-a:collider', 'exit').count, 1);
});

test('PhysicsWorld creates kinematic capsules and reports grounded state against floors', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });

  const character = world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  const ground = world.getKinematicGroundState('player');
  const snapshot = world.getSnapshot();

  assert.equal(character.id, 'player');
  assert.equal(character.bodyId, 'player');
  assert.equal(world.getBody('player').motionType, 'kinematic');
  assert.equal(snapshot.characterCount, 1);
  assert.equal(ground.grounded, true);
  assert.equal(ground.walkable, true);
  assert.equal(ground.colliderId, 'floor:collider');
  assert.ok((ground.angleDegrees ?? 999) < 1, `expected floor angle near 0, got ${ground.angleDegrees}`);
});

test('PhysicsWorld kinematic capsules stop before walls and keep ground state while moving', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createStaticBoxCollider({
    id: 'wall',
    position: { x: 20, y: 15, z: 0 },
    size: 10
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });

  const moveResult = world.moveKinematicCapsule('player', { x: 30, y: 0, z: 0 });
  const playerBody = world.getBody('player');
  const frame = world.buildDebugFrame();

  assert.equal(moveResult.blocked, true);
  assert.equal(moveResult.hitColliderId, 'wall:collider');
  assert.ok(playerBody.position.x > 8.5 && playerBody.position.x < 10.1, `expected player to stop before wall, got ${playerBody.position.x}`);
  assert.equal(world.isKinematicCapsuleGrounded('player'), true);
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'character-controller'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'character-ground-normal'));
});

test('PhysicsWorld kinematic controller hit callbacks cache enter, stay, and exit against colliders', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createStaticBoxCollider({
    id: 'wall',
    position: { x: 20, y: 0, z: 0 },
    size: 10
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 0, z: 0 },
    radius: 5,
    halfHeight: 10
  });

  world.moveKinematicCapsule('player', { x: 30, y: 0, z: 0 });

  let hitEvents = world.getKinematicCapsuleHitEvents('player', 'enter');
  assert.equal(hitEvents.colliderCount, 1);
  assert.equal(hitEvents.colliders[0].id, 'wall:collider');
  assert.equal(world.getControllerHitEventState().summary.enterCount, 1);

  const hitSummary = world.getKinematicCapsuleLastHit('player');
  assert.equal(hitSummary.colliderId, 'wall:collider');
  assert.ok(typeof hitSummary.algorithm === 'string' && hitSummary.algorithm.length > 0);

  let frame = world.buildDebugFrame();
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'character-hit-point'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'character-hit-normal'));

  world.moveKinematicCapsule('player', { x: 2, y: 0, z: 0 });
  hitEvents = world.getKinematicCapsuleHitEvents('player', 'stay');
  assert.equal(hitEvents.colliderCount, 1);
  assert.equal(hitEvents.colliders[0].id, 'wall:collider');
  assert.equal(world.getControllerHitEventState().summary.stayCount, 1);

  world.moveKinematicCapsule('player', { x: -6, y: 0, z: 0 });
  hitEvents = world.getKinematicCapsuleHitEvents('player', 'exit');
  assert.equal(hitEvents.colliderCount, 1);
  assert.equal(hitEvents.colliders[0].id, 'wall:collider');
  assert.equal(world.getControllerHitEventState().summary.exitCount, 1);

  frame = world.buildDebugFrame();
  assert.equal(frame.primitives.some((primitive) => primitive.category === 'character-hit-point'), false);
});

test('PhysicsWorld grounded kinematic controller hit callbacks prefer blocking walls over walkable ground', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: -9.81, z: 0 }
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -70, z: 0 },
    size: 140
  });
  world.createStaticBoxCollider({
    id: 'wall',
    position: { x: 40, y: 15, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 0, z: 0 },
    radius: 8,
    halfHeight: 16
  });
  world.configureKinematicController('player', {
    jumpSpeed: 9,
    gravityScale: 1,
    stepOffset: 8,
    groundSnapDistance: 3
  });
  world.setKinematicCapsuleMoveIntent('player', { x: 20, y: 0, z: 0 });

  let sawWallEnter = false;
  for (let frameIndex = 0; frameIndex < 120; frameIndex += 1) {
    world.step(1 / 60);
    const enterEvents = world.getKinematicCapsuleHitEvents('player', 'enter');
    const stayEvents = world.getKinematicCapsuleHitEvents('player', 'stay');
    const exitEvents = world.getKinematicCapsuleHitEvents('player', 'exit');
    const lastHit = world.getKinematicCapsuleLastHit('player');

    if (enterEvents.colliders.some((collider) => collider.id === 'wall:collider')) {
      sawWallEnter = true;
    }

    if (sawWallEnter) {
      assert.equal(lastHit?.colliderId, 'wall:collider');
      assert.equal(exitEvents.colliderCount, 0);
      assert.equal(
        enterEvents.colliders.some((collider) => collider.id === 'floor:collider') ||
        stayEvents.colliders.some((collider) => collider.id === 'floor:collider'),
        false
      );
    }

    if (sawWallEnter && frameIndex >= 80) {
      assert.equal(stayEvents.colliderCount, 1);
      assert.equal(stayEvents.colliders[0].id, 'wall:collider');
    }
  }

  assert.equal(sawWallEnter, true);
  const frame = world.buildDebugFrame();
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'character-hit-point'));
  assert.ok(frame.primitives.some((primitive) => primitive.category === 'character-hit-normal'));
});

test('PhysicsWorld exports and imports kinematic capsules with ground settings intact', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicCapsule('player', {
    skinWidth: 1.25,
    groundProbeDistance: 6,
    maxGroundAngleDegrees: 40
  });
  world.configureKinematicController('player', {
    jumpSpeed: 9,
    gravityScale: 1.2,
    stepOffset: 7,
    groundSnapDistance: 3,
    airControlFactor: 0.6,
    coyoteTimeSeconds: 0.12,
    jumpBufferSeconds: 0.14,
    rideMovingPlatforms: false
  });
  world.setKinematicCapsuleMoveIntent('player', { x: 4, y: 0, z: 0 });

  const scene = world.exportSceneDefinition();
  const importedWorld = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  const imported = importedWorld.importSceneDefinition(scene, {
    reset: true
  });
  const importedCharacter = importedWorld.getKinematicCapsule('player');

  assert.equal(imported.characterCount, 1);
  assert.equal(importedWorld.getSnapshot().characterCount, 1);
  assert.equal(importedCharacter.skinWidth, 1.25);
  assert.equal(importedCharacter.groundProbeDistance, 6);
  assert.equal(importedCharacter.maxGroundAngleDegrees, 40);
  assert.equal(importedCharacter.jumpSpeed, 9);
  assert.equal(importedCharacter.gravityScale, 1.2);
  assert.equal(importedCharacter.stepOffset, 7);
  assert.equal(importedCharacter.groundSnapDistance, 3);
  assert.equal(importedCharacter.airControlFactor, 0.6);
  assert.equal(importedCharacter.coyoteTimeSeconds, 0.12);
  assert.equal(importedCharacter.jumpBufferSeconds, 0.14);
  assert.equal(importedCharacter.rideMovingPlatforms, false);
  assert.equal(importedCharacter.moveIntent.x, 4);
  assert.equal(importedWorld.isKinematicCapsuleGrounded('player'), true);
});

test('PhysicsWorld kinematic controllers jump upward and land back on walkable ground', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    jumpSpeed: 9,
    gravityScale: 1,
    stepOffset: 6,
    groundSnapDistance: 2
  });

  const startY = world.getBody('player').position.y;
  assert.equal(world.isKinematicCapsuleGrounded('player'), true);

  world.jumpKinematicCapsule('player');
  world.step(1 / 60);

  const airborne = world.getKinematicCapsule('player');
  assert.equal(airborne.grounded, false);
  assert.ok(world.getBody('player').position.y > startY, `expected player to rise after jump, got ${world.getBody('player').position.y}`);
  assert.ok((airborne.verticalVelocity ?? 0) > 0, `expected positive vertical velocity after jump, got ${airborne.verticalVelocity}`);

  for (let index = 0; index < 180; index += 1) {
    world.step(1 / 60);
  }

  const landed = world.getKinematicCapsule('player');
  assert.equal(landed.grounded, true);
  assert.ok(Math.abs(world.getBody('player').position.y - startY) <= 0.55, `expected player to land near start height, got ${world.getBody('player').position.y}`);
  assert.equal(Math.abs(landed.verticalVelocity ?? 0) <= 1e-8, true);
});

test('PhysicsWorld kinematic controllers can set vertical velocity directly', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });

  const startY = world.getBody('player').position.y;
  world.setKinematicCapsuleVerticalVelocity('player', 10);
  world.step(1 / 60);

  const character = world.getKinematicCapsule('player');
  assert.ok(world.getBody('player').position.y > startY, `expected player to rise after setting vertical velocity, got ${world.getBody('player').position.y}`);
  assert.ok((character.verticalVelocity ?? 0) > 0, `expected positive vertical velocity, got ${character.verticalVelocity}`);
  assert.equal(character.grounded, false);
});

test('PhysicsWorld kinematic controllers can launch with horizontal and vertical velocity', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });

  const startPosition = { ...world.getBody('player').position };
  world.launchKinematicCapsule('player', { x: 6, y: 8, z: 0 });
  world.step(1 / 60);

  const character = world.getKinematicCapsule('player');
  const body = world.getBody('player');
  assert.ok(body.position.x > startPosition.x + 0.05, `expected launch to move player horizontally, got ${body.position.x}`);
  assert.ok(body.position.y > startPosition.y, `expected launch to move player upward, got ${body.position.y}`);
  assert.ok((character.verticalVelocity ?? 0) > 0, `expected positive launch vertical velocity, got ${character.verticalVelocity}`);
  assert.equal(character.grounded, false);
});

test('PhysicsWorld kinematic controllers smoothly update capsule size when crouching', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: 0, z: 0 }
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 20, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    gravityScale: 0,
    stepOffset: 0,
    groundSnapDistance: 0
  });

  const startCharacter = world.getKinematicCapsule('player');
  const startBody = world.getBody('player');
  const startFootY = startBody.position.y - startCharacter.halfHeight - startCharacter.radius;
  assert.equal(startCharacter.halfHeight, 10);
  assert.equal(startCharacter.radius, 5);

  world.setKinematicCapsuleCrouch('player', 6, 3, 20);
  world.step(1 / 60);

  const midCharacter = world.getKinematicCapsule('player');
  const midBody = world.getBody('player');
  assert.ok(midCharacter.halfHeight < 10, `expected half height to shrink, got ${midCharacter.halfHeight}`);
  assert.ok(midCharacter.radius < 5, `expected radius to shrink, got ${midCharacter.radius}`);
  assert.ok(Math.abs((midBody.position.y - midCharacter.halfHeight - midCharacter.radius) - startFootY) <= 1e-4, 'expected crouch to keep feet anchored');

  for (let index = 0; index < 20; index += 1) {
    world.step(1 / 60);
  }

  const crouchedCharacter = world.getKinematicCapsule('player');
  const crouchedBody = world.getBody('player');
  assert.ok(Math.abs(crouchedCharacter.halfHeight - 6) <= 0.5, `expected crouch half height near 6, got ${crouchedCharacter.halfHeight}`);
  assert.ok(Math.abs(crouchedCharacter.radius - 3) <= 0.5, `expected crouch radius near 3, got ${crouchedCharacter.radius}`);
  assert.equal(crouchedCharacter.crouchBlocked, false);
  assert.ok(Math.abs((crouchedBody.position.y - crouchedCharacter.halfHeight - crouchedCharacter.radius) - startFootY) <= 1e-4, 'expected crouched feet to remain anchored');

  world.setKinematicCapsuleCrouch('player', 10, 5, 20);
  for (let index = 0; index < 20; index += 1) {
    world.step(1 / 60);
  }

  const standingCharacter = world.getKinematicCapsule('player');
  const standingBody = world.getBody('player');
  assert.ok(Math.abs(standingCharacter.halfHeight - 10) <= 0.5, `expected standing half height near 10, got ${standingCharacter.halfHeight}`);
  assert.ok(Math.abs(standingCharacter.radius - 5) <= 0.5, `expected standing radius near 5, got ${standingCharacter.radius}`);
  assert.equal(standingCharacter.crouchBlocked, false);
  assert.ok(Math.abs((standingBody.position.y - standingCharacter.halfHeight - standingCharacter.radius) - startFootY) <= 1e-4, 'expected uncrouch to keep feet anchored');
});

test('PhysicsWorld kinematic controllers keep crouching under low ceilings and auto-uncrouch once clear', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -20, z: 0 },
    size: 40
  });
  world.createStaticBoxCollider({
    id: 'ceiling',
    position: { x: 0, y: 34, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 30, z: 0 },
    radius: 3,
    halfHeight: 6
  });
  world.configureKinematicController('player', {
    gravityScale: 1,
    stepOffset: 6,
    groundSnapDistance: 2
  });

  for (let index = 0; index < 90; index += 1) {
    world.step(1 / 60);
  }

  const crouchedCharacter = world.getKinematicCapsule('player');
  const crouchedBody = world.getBody('player');
  const anchoredFootY = crouchedBody.position.y - crouchedCharacter.halfHeight - crouchedCharacter.radius;
  world.setKinematicCapsuleCrouch('player', 10, 5, 30);

  for (let index = 0; index < 60; index += 1) {
    world.step(1 / 60);
  }

  const blockedCharacter = world.getKinematicCapsule('player');
  const blockedBody = world.getBody('player');
  assert.ok(Math.abs(blockedCharacter.halfHeight - 6) <= 0.25, `expected blocked uncrouch to keep crouched half height, got ${blockedCharacter.halfHeight}`);
  assert.ok(Math.abs(blockedCharacter.radius - 3) <= 0.25, `expected blocked uncrouch to keep crouched radius, got ${blockedCharacter.radius}`);
  assert.equal(blockedCharacter.crouchBlocked, true);
  assert.equal(blockedCharacter.crouchBlockedColliderId, 'ceiling:collider');
  assert.ok(Math.abs((blockedBody.position.y - blockedCharacter.halfHeight - blockedCharacter.radius) - anchoredFootY) <= 1e-6, 'expected blocked uncrouch to keep feet anchored');

  world.configureColliderCollision('ceiling:collider', {
    enabled: false
  });
  for (let index = 0; index < 60; index += 1) {
    world.step(1 / 60);
  }

  const standingCharacter = world.getKinematicCapsule('player');
  const standingBody = world.getBody('player');
  assert.ok(Math.abs(standingCharacter.halfHeight - 10) <= 0.5, `expected auto-uncrouch half height near 10, got ${standingCharacter.halfHeight}`);
  assert.ok(Math.abs(standingCharacter.radius - 5) <= 0.5, `expected auto-uncrouch radius near 5, got ${standingCharacter.radius}`);
  assert.equal(standingCharacter.crouchBlocked, false);
  assert.equal(standingCharacter.crouchBlockedColliderId, null);
  assert.ok(Math.abs((standingBody.position.y - standingCharacter.halfHeight - standingCharacter.radius) - anchoredFootY) <= 1e-6, 'expected auto-uncrouch to keep feet anchored');
});

test('PhysicsWorld kinematic controllers can drop through one-way platforms', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createStaticBoxCollider({
    id: 'platform',
    position: { x: 0, y: 12, z: 0 },
    size: 12,
    isOneWay: true
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 33, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    jumpSpeed: 9,
    gravityScale: 1,
    stepOffset: 6,
    groundSnapDistance: 2
  });

  for (let index = 0; index < 120; index += 1) {
    world.step(1 / 60);
  }
  const groundedCharacter = world.getKinematicCapsule('player');
  assert.equal(groundedCharacter.groundColliderId, 'platform:collider');

  world.dropThroughOneWayPlatforms('player', 0.25);
  for (let index = 0; index < 180; index += 1) {
    world.step(1 / 60);
  }

  const droppedCharacter = world.getKinematicCapsule('player');
  assert.equal(droppedCharacter.groundColliderId, 'floor:collider');
  const droppedBody = world.getBody('player');
  const expectedStandHeight = (droppedCharacter.halfHeight ?? 0) +
    (droppedCharacter.radius ?? 0) +
    (droppedCharacter.skinWidth ?? 0);
  assert.ok(
    droppedBody.position.y <= expectedStandHeight + 0.2,
    `expected player to settle on the floor, got ${droppedBody.position.y}`
  );
});

test('PhysicsWorld kinematic controllers pass upward through one-way box platforms and land on them from above', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createStaticBoxCollider({
    id: 'platform',
    position: { x: 0, y: 12, z: 0 },
    size: 12,
    isOneWay: true
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    jumpSpeed: 20,
    gravityScale: 1,
    stepOffset: 6,
    groundSnapDistance: 2
  });

  assert.equal(world.getCollider('platform:collider').isOneWay, true);
  world.jumpKinematicCapsule('player');

  let passedAbovePlatform = false;
  for (let index = 0; index < 120; index += 1) {
    world.step(1 / 60);
    if (world.getBody('player').position.y > 33) {
      passedAbovePlatform = true;
    }
  }

  assert.equal(passedAbovePlatform, true, 'expected player to pass upward through the one-way platform');

  for (let index = 0; index < 180; index += 1) {
    world.step(1 / 60);
  }

  const character = world.getKinematicCapsule('player');
  const body = world.getBody('player');
  assert.equal(character.grounded, true);
  assert.equal(character.groundColliderId, 'platform:collider');
  assert.ok(Math.abs(body.position.y - 33) <= 0.75, `expected player to land on the one-way platform, got ${body.position.y}`);
});

test('PhysicsWorld kinematic controllers support one-way convex hull platforms', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createStaticConvexHullCollider({
    id: 'platform',
    position: { x: 0, y: 12, z: 0 },
    vertices: [
      { x: -20, y: -6, z: -20 },
      { x: 20, y: -6, z: -20 },
      { x: 20, y: -6, z: 20 },
      { x: -20, y: -6, z: 20 },
      { x: -20, y: 6, z: -20 },
      { x: 20, y: 6, z: -20 },
      { x: 20, y: 6, z: 20 },
      { x: -20, y: 6, z: 20 }
    ]
  });
  world.configureColliderOneWay('platform:collider', true);
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    jumpSpeed: 20,
    gravityScale: 1,
    stepOffset: 6,
    groundSnapDistance: 2
  });

  world.jumpKinematicCapsule('player');
  let passedAbovePlatform = false;
  for (let index = 0; index < 120; index += 1) {
    world.step(1 / 60);
    if (world.getBody('player').position.y > 33) {
      passedAbovePlatform = true;
    }
  }

  assert.equal(passedAbovePlatform, true, 'expected player to pass upward through the one-way convex hull platform');

  for (let index = 0; index < 180; index += 1) {
    world.step(1 / 60);
  }

  const character = world.getKinematicCapsule('player');
  const body = world.getBody('player');
  assert.equal(character.grounded, true);
  assert.equal(character.groundColliderId, 'platform:collider');
  assert.ok(Math.abs(body.position.y - 33) <= 0.9, `expected player to land on the one-way convex hull platform, got ${body.position.y}`);
});

test('PhysicsWorld kinematic controllers scale airborne movement with air control', () => {
  const world = new PhysicsWorld();
  world.createKinematicCapsule({
    id: 'free-full',
    position: { x: 0, y: 40, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.createKinematicCapsule({
    id: 'free-soft',
    position: { x: 0, y: 40, z: 10 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('free-full', {
    airControlFactor: 1
  });
  world.configureKinematicController('free-soft', {
    airControlFactor: 0.25
  });
  world.setKinematicCapsuleMoveIntent('free-full', { x: 12, y: 0, z: 0 });
  world.setKinematicCapsuleMoveIntent('free-soft', { x: 12, y: 0, z: 0 });

  for (let index = 0; index < 20; index += 1) {
    world.step(1 / 60);
  }

  const fullBody = world.getBody('free-full');
  const softBody = world.getBody('free-soft');

  assert.ok(fullBody.position.x > softBody.position.x + 2, `expected full air control to move farther, got full=${fullBody.position.x} soft=${softBody.position.x}`);
});

test('PhysicsWorld kinematic controllers honor coyote time after leaving ledges', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'ledge',
    position: { x: 0, y: -10, z: 0 },
    size: 8
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    jumpSpeed: 9,
    coyoteTimeSeconds: 0.15
  });
  world.setKinematicCapsuleMoveIntent('player', { x: 12, y: 0, z: 0 });

  let airborneFrame = -1;
  for (let index = 0; index < 80; index += 1) {
    world.step(1 / 60);
    if (!world.isKinematicCapsuleGrounded('player')) {
      airborneFrame = index;
      break;
    }
  }

  assert.ok(airborneFrame >= 0, 'expected player to leave the ledge');
  const beforeJumpY = world.getBody('player').position.y;
  world.jumpKinematicCapsule('player');
  world.step(1 / 60);

  const character = world.getKinematicCapsule('player');
  assert.ok((character.verticalVelocity ?? 0) > 0, `expected coyote jump to produce upward velocity, got ${character.verticalVelocity}`);
  assert.ok(world.getBody('player').position.y > beforeJumpY, `expected coyote jump to raise the player, got y=${world.getBody('player').position.y}`);
});

test('PhysicsWorld kinematic controllers honor jump buffering shortly before landing', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 16, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    jumpSpeed: 9,
    jumpBufferSeconds: 0.5
  });

  world.step(1 / 60);
  world.getBody('player').position.y += 1.2;
  world.characterRegistry.getMutable('player').grounded = false;
  world.characterRegistry.getMutable('player').walkable = false;
  world.characterRegistry.getMutable('player').verticalVelocity = -1;
  world.jumpKinematicCapsule('player');

  let sawBufferedJump = false;
  for (let index = 0; index < 40; index += 1) {
    world.step(1 / 60);
    const character = world.getKinematicCapsule('player');
    if ((character.verticalVelocity ?? 0) > 0.5) {
      sawBufferedJump = true;
      break;
    }
  }

  assert.equal(sawBufferedJump, true, 'expected buffered jump to fire after landing');
});

test('PhysicsWorld kinematic controllers ride kinematic moving platforms via cached ground anchors', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: -9.81, z: 0 }
  });
  world.createBoxBody({
    id: 'platform',
    motionType: 'kinematic',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    rideMovingPlatforms: true
  });

  world.step(1 / 60);
  const platformBody = world.bodyRegistry.getMutable('platform');
  platformBody.position.x += 3;
  world.markCollisionStateDirty();
  world.step(1 / 60);

  const playerBody = world.getBody('player');
  const character = world.getKinematicCapsule('player');
  assert.ok(playerBody.position.x > 2, `expected player to be carried by the platform, got x=${playerBody.position.x}`);
  assert.ok((character.lastPlatformCarry?.x ?? 0) > 2, `expected recorded platform carry, got ${character.lastPlatformCarry?.x}`);
  assert.ok((character.platformVelocity?.x ?? 0) > 100, `expected platform velocity to be recorded, got ${character.platformVelocity?.x}`);
  assert.equal(character.lastPlatformBodyId, 'platform');
});

test('PhysicsWorld kinematic controllers inherit horizontal platform velocity when jumping off moving platforms', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: -9.81, z: 0 }
  });
  world.createBoxBody({
    id: 'platform',
    motionType: 'kinematic',
    position: { x: 0, y: -10, z: 0 },
    size: 20
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    jumpSpeed: 9,
    rideMovingPlatforms: true
  });

  world.step(1 / 60);
  const platformBody = world.bodyRegistry.getMutable('platform');
  platformBody.position.x += 2;
  world.markCollisionStateDirty();
  world.jumpKinematicCapsule('player');
  world.step(1 / 60);

  const xAfterJump = world.getBody('player').position.x;
  const afterJumpCharacter = world.getKinematicCapsule('player');
  assert.ok((afterJumpCharacter.inheritedVelocity?.x ?? 0) > 100, `expected inherited velocity from moving platform, got ${afterJumpCharacter.inheritedVelocity?.x}`);

  for (let index = 0; index < 5; index += 1) {
    world.step(1 / 60);
  }

  const playerBody = world.getBody('player');
  const character = world.getKinematicCapsule('player');
  assert.equal(character.grounded, false);
  assert.ok(playerBody.position.x > xAfterJump + 4, `expected airborne player to keep horizontal inertia, got x=${playerBody.position.x}`);
});

test('PhysicsWorld kinematic controllers keep platform inertia after walking off moving platforms', () => {
  const world = new PhysicsWorld({
    gravity: { x: 0, y: -9.81, z: 0 }
  });
  world.createBoxBody({
    id: 'platform',
    motionType: 'kinematic',
    position: { x: 0, y: -10, z: 0 },
    size: 8
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('player', {
    rideMovingPlatforms: true
  });

  world.step(1 / 60);
  const platformBody = world.bodyRegistry.getMutable('platform');
  platformBody.position.x += 1.5;
  world.markCollisionStateDirty();
  world.step(1 / 60);

  world.setKinematicCapsuleMoveIntent('player', { x: 12, y: 0, z: 0 });
  let airborne = false;
  for (let index = 0; index < 40; index += 1) {
    platformBody.position.x += 1.5;
    world.markCollisionStateDirty();
    world.step(1 / 60);
    if (!world.isKinematicCapsuleGrounded('player')) {
      airborne = true;
      break;
    }
  }

  assert.equal(airborne, true, 'expected player to walk off the moving platform');
  const xWhenLeaving = world.getBody('player').position.x;
  world.setKinematicCapsuleMoveIntent('player', { x: 0, y: 0, z: 0 });
  world.step(1 / 60);

  const character = world.getKinematicCapsule('player');
  const playerBody = world.getBody('player');
  assert.ok((character.inheritedVelocity?.x ?? 0) > 80, `expected walk-off inertia to preserve horizontal platform speed, got ${character.inheritedVelocity?.x}`);
  assert.ok(playerBody.position.x > xWhenLeaving + 1, `expected player to keep drifting after walking off, got x=${playerBody.position.x}`);
});

test('PhysicsWorld kinematic controllers project movement onto walkable slopes', () => {
  const world = new PhysicsWorld();
  world.createStaticConvexHullCollider({
    id: 'ramp',
    vertices: [
      { x: -10, y: -2, z: -6 },
      { x: 10, y: -2, z: -6 },
      { x: 10, y: -2, z: 6 },
      { x: -10, y: -2, z: 6 },
      { x: -10, y: 4, z: -6 },
      { x: -10, y: 4, z: 6 }
    ]
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 6, y: 14.2, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.setKinematicCapsuleMoveIntent('player', { x: -18, y: 0, z: 0 });

  for (let index = 0; index < 5; index += 1) {
    world.step(1 / 60);
  }

  const body = world.getBody('player');
  const character = world.getKinematicCapsule('player');
  assert.ok(body.position.x < 5, `expected player to move uphill, got x=${body.position.x}`);
  assert.ok(body.position.y > 14.5, `expected player to climb the slope, got y=${body.position.y}`);
  assert.equal(character.grounded, true);
  assert.equal(character.walkable, true);
});

test('PhysicsWorld kinematic ground probes use the fast static convex face path on walkable ramps', () => {
  const world = new PhysicsWorld();
  world.createStaticConvexHullCollider({
    id: 'ramp',
    vertices: [
      { x: -10, y: -2, z: -6 },
      { x: 10, y: -2, z: -6 },
      { x: 10, y: -2, z: 6 },
      { x: -10, y: -2, z: 6 },
      { x: -10, y: 4, z: -6 },
      { x: -10, y: 4, z: 6 }
    ]
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 6, y: 14.2, z: 0 },
    radius: 5,
    halfHeight: 10
  });

  const character = world.getKinematicCapsule('player');
  const body = world.getBody('player');
  const shape = world.getShape(character.shapeId);
  const hit = world.characterShapeCastAgainstWorld(character, body, shape, {
    origin: body.position,
    direction: { x: 0, y: -1, z: 0 },
    maxDistance: character.groundProbeDistance + character.skinWidth + 1e-4,
    rotation: body.rotation,
    excludeBodyId: body.id,
    excludeColliderIds: body.colliderIds,
    ignoreDynamicTargets: false,
    ignoreSensors: true,
    storeResult: false,
    queryMode: 'ground'
  });

  assert.equal(hit.hit, true);
  assert.equal(hit.algorithm, 'character-ground-face-v1');
});

test('PhysicsWorld kinematic motion queries use the fast static convex face path against walls', () => {
  const world = new PhysicsWorld();
  world.createStaticConvexHullCollider({
    id: 'wall',
    vertices: [
      { x: -2, y: -8, z: -8 },
      { x: 2, y: -8, z: -8 },
      { x: 2, y: 8, z: -8 },
      { x: -2, y: 8, z: -8 },
      { x: -2, y: -8, z: 8 },
      { x: 2, y: -8, z: 8 },
      { x: 2, y: 8, z: 8 },
      { x: -2, y: 8, z: 8 }
    ]
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: -12, y: 0, z: 0 },
    radius: 2,
    halfHeight: 4
  });

  const character = world.getKinematicCapsule('player');
  const body = world.getBody('player');
  const shape = world.getShape(character.shapeId);
  const hit = world.characterShapeCastAgainstWorld(character, body, shape, {
    origin: body.position,
    direction: { x: 1, y: 0, z: 0 },
    maxDistance: 16,
    rotation: body.rotation,
    excludeBodyId: body.id,
    excludeColliderIds: body.colliderIds,
    ignoreDynamicTargets: false,
    ignoreSensors: true,
    storeResult: false,
    queryMode: 'motion'
  });

  assert.equal(hit.hit, true);
  assert.equal(hit.algorithm, 'character-motion-face-v1');
  assert.equal(hit.colliderId, 'wall:collider');
});

test('PhysicsWorld kinematic controllers keep moving across flat static convex hull tops', () => {
  const world = new PhysicsWorld();
  world.createStaticConvexHullCollider({
    id: 'platform',
    vertices: [
      { x: -20, y: -2, z: -20 },
      { x: 20, y: -2, z: -20 },
      { x: 20, y: -2, z: 20 },
      { x: -20, y: -2, z: 20 },
      { x: -20, y: 2, z: -20 },
      { x: 20, y: 2, z: -20 },
      { x: 20, y: 2, z: 20 },
      { x: -20, y: 2, z: 20 }
    ]
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 17, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.setKinematicCapsuleMoveIntent('player', { x: 8, y: 0, z: 0 });

  for (let index = 0; index < 20; index += 1) {
    world.step(1 / 60);
  }

  const body = world.getBody('player');
  const character = world.getKinematicCapsule('player');
  assert.ok(body.position.x > 2.5, `expected player to keep moving across convex hull top, got x=${body.position.x}`);
  assert.equal(character.grounded, true);
  assert.equal(character.walkable, true);
});

test('PhysicsWorld kinematic controllers build stable ground face cache on static convex hull tops', () => {
  const world = new PhysicsWorld();
  world.createStaticConvexHullCollider({
    id: 'platform',
    vertices: [
      { x: -20, y: -2, z: -20 },
      { x: 20, y: -2, z: -20 },
      { x: 20, y: -2, z: 20 },
      { x: -20, y: -2, z: 20 },
      { x: -20, y: 2, z: -20 },
      { x: 20, y: 2, z: -20 },
      { x: 20, y: 2, z: 20 },
      { x: -20, y: 2, z: 20 }
    ]
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 17, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.setKinematicCapsuleMoveIntent('player', { x: 6, y: 0, z: 0 });

  for (let index = 0; index < 12; index += 1) {
    world.step(1 / 60);
  }

  const internalCharacter = world.characterRegistry.getMutable('player');
  assert.equal(internalCharacter.groundFaceCache.colliderId, 'platform:collider');
  assert.ok(internalCharacter.groundFaceCache.stableFrames >= 3,
    `expected ground face cache to build temporal coherence, got ${internalCharacter.groundFaceCache.stableFrames}`);
});

test('PhysicsWorld kinematic controllers build stable ground broadphase candidate caches on static convex hull tops', () => {
  const world = new PhysicsWorld();
  world.createStaticConvexHullCollider({
    id: 'platform',
    vertices: [
      { x: -20, y: -2, z: -20 },
      { x: 20, y: -2, z: -20 },
      { x: 20, y: -2, z: 20 },
      { x: -20, y: -2, z: 20 },
      { x: -20, y: 2, z: -20 },
      { x: 20, y: 2, z: -20 },
      { x: 20, y: 2, z: 20 },
      { x: -20, y: 2, z: 20 }
    ]
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 17, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.setKinematicCapsuleMoveIntent('player', { x: 6, y: 0, z: 0 });

  for (let index = 0; index < 12; index += 1) {
    world.step(1 / 60);
  }

  const internalCharacter = world.characterRegistry.getMutable('player');
  assert.ok(internalCharacter.groundCandidateCache.bounds, 'expected ground candidate cache bounds to exist');
  assert.ok(internalCharacter.groundCandidateCache.colliderIds.includes('platform:collider'),
    `expected ground candidate cache to include platform collider, got ${internalCharacter.groundCandidateCache.colliderIds.join(', ')}`);
  assert.ok(internalCharacter.groundCandidateCache.stableFrames >= 2,
    `expected stable ground candidate cache frames, got ${internalCharacter.groundCandidateCache.stableFrames}`);
});

test('PhysicsWorld kinematic motion queries build stable broadphase candidate caches against static convex walls', () => {
  const world = new PhysicsWorld();
  world.createStaticConvexHullCollider({
    id: 'wall',
    vertices: [
      { x: -2, y: -8, z: -8 },
      { x: 2, y: -8, z: -8 },
      { x: 2, y: 8, z: -8 },
      { x: -2, y: 8, z: -8 },
      { x: -2, y: -8, z: 8 },
      { x: 2, y: -8, z: 8 },
      { x: 2, y: 8, z: 8 },
      { x: -2, y: 8, z: 8 }
    ]
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: -12, y: 0, z: 0 },
    radius: 2,
    halfHeight: 4
  });

  const internalCharacter = world.characterRegistry.getMutable('player');
  const body = world.getBody('player');
  const shape = world.getShape(internalCharacter.shapeId);

  for (let index = 0; index < 4; index += 1) {
    world.characterShapeCastAgainstWorld(internalCharacter, body, shape, {
      origin: body.position,
      direction: { x: 1, y: 0, z: 0 },
      maxDistance: 16,
      rotation: body.rotation,
      excludeBodyId: body.id,
      excludeColliderIds: body.colliderIds,
      ignoreDynamicTargets: false,
      ignoreSensors: true,
      storeResult: false,
      queryMode: 'motion'
    });
  }

  assert.ok(internalCharacter.motionCandidateCache.bounds, 'expected motion candidate cache bounds to exist');
  assert.ok(internalCharacter.motionCandidateCache.colliderIds.includes('wall:collider'),
    `expected motion candidate cache to include wall collider, got ${internalCharacter.motionCandidateCache.colliderIds.join(', ')}`);
  assert.ok(internalCharacter.motionCandidateCache.stableFrames >= 3,
    `expected stable motion candidate cache frames, got ${internalCharacter.motionCandidateCache.stableFrames}`);
});

test('PhysicsWorld kinematic overlap recovery resolves shallow penetration into static convex hull tops', () => {
  const world = new PhysicsWorld();
  world.createStaticConvexHullCollider({
    id: 'platform',
    vertices: [
      { x: -20, y: -2, z: -20 },
      { x: 20, y: -2, z: -20 },
      { x: 20, y: -2, z: 20 },
      { x: -20, y: -2, z: 20 },
      { x: -20, y: 2, z: -20 },
      { x: 20, y: 2, z: -20 },
      { x: 20, y: 2, z: 20 },
      { x: -20, y: 2, z: 20 }
    ]
  });
  world.createKinematicCapsule({
    id: 'player',
    position: { x: 0, y: 16.25, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.setKinematicCapsuleMoveIntent('player', { x: 4, y: 0, z: 0 });

  world.step(1 / 60);

  const body = world.getBody('player');
  const character = world.getKinematicCapsule('player');
  assert.ok(body.position.y > 16.25, `expected overlap recovery to push the player upward, got y=${body.position.y}`);
  assert.ok(body.position.x > 0.01, `expected player to keep moving after recovery, got x=${body.position.x}`);
  assert.equal(character.grounded, true);
  assert.ok((character.lastRecoveryDistance ?? 0) > 0, `expected overlap recovery to record a positive distance, got ${character.lastRecoveryDistance}`);
});

test('PhysicsWorld kinematic controllers use step offset to climb low ledges', () => {
  const world = new PhysicsWorld();
  world.createStaticBoxCollider({
    id: 'floor',
    position: { x: 0, y: -10, z: 0 },
    size: 60
  });
  world.createStaticBoxCollider({
    id: 'step-a',
    position: { x: 0, y: 5, z: 0 },
    size: 10
  });
  world.createStaticBoxCollider({
    id: 'step-b',
    position: { x: 0, y: 5, z: 12 },
    size: 10
  });
  world.createKinematicCapsule({
    id: 'stepper',
    position: { x: -14, y: 15, z: 0 },
    radius: 5,
    halfHeight: 10
  });
  world.createKinematicCapsule({
    id: 'blocked',
    position: { x: -14, y: 15, z: 12 },
    radius: 5,
    halfHeight: 10
  });
  world.configureKinematicController('stepper', {
    stepOffset: 11,
    groundSnapDistance: 2
  });
  world.configureKinematicController('blocked', {
    stepOffset: 0,
    groundSnapDistance: 2
  });
  world.setKinematicCapsuleMoveIntent('stepper', { x: 8, y: 0, z: 0 });
  world.setKinematicCapsuleMoveIntent('blocked', { x: 8, y: 0, z: 0 });

  for (let index = 0; index < 120; index += 1) {
    world.step(1 / 60);
  }

  const steppedBody = world.getBody('stepper');
  const blockedBody = world.getBody('blocked');

  assert.ok(steppedBody.position.x > -2, `expected stepper to climb over the ledge, got x=${steppedBody.position.x}`);
  assert.ok(steppedBody.position.y > 20, `expected stepper to stand on the raised step, got y=${steppedBody.position.y}`);
  assert.ok(blockedBody.position.x < -9, `expected blocked character to stop before the ledge, got x=${blockedBody.position.x}`);
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
