import { PhysicsWorld, createVec3 } from '../physics/index.js';
import { getConvexHullPresetVertexText } from '../shared/convex-hull-presets.js';
import { resolveClothPreset } from '../shared/cloth-presets.js';
import { resolveSoftBodyPreset } from '../shared/soft-body-presets.js';

function formatVector(vector) {
  return `${vector.x}, ${vector.y}, ${vector.z}`;
}

function formatOptionalNumber(value, fallback = 'none') {
  if (value === undefined || value === null || value === '') {
    return fallback;
  }

  return Number.isFinite(Number(value)) ? `${Number(value)}` : fallback;
}

function formatIdentifierList(records) {
  if (!Array.isArray(records) || records.length === 0) {
    return 'none';
  }

  return records.map((record) => record.id).join(', ');
}

function normalizeEventPhase(phase) {
  const resolvedPhase = String(phase ?? '').trim().toLowerCase();
  if (resolvedPhase === 'enter' || resolvedPhase === 'exit') {
    return resolvedPhase;
  }

  return 'stay';
}

function radiansToDegrees(value) {
  return Number(value) * (180 / Math.PI);
}

function createDefaultConvexHullVertices() {
  return [
    createVec3(-50, -50, -50),
    createVec3(50, -50, -50),
    createVec3(50, -50, 50),
    createVec3(-50, -50, 50),
    createVec3(0, 50, 0)
  ];
}

function parseConvexHullVertices(verticesText) {
  const parsedText = String(verticesText ?? '').trim();
  if (!parsedText) {
    return createDefaultConvexHullVertices();
  }

  const records = parsedText
    .split(/[\n;|]+/)
    .map((entry) => entry.trim())
    .filter(Boolean);

  const vertices = [];
  for (const record of records) {
    const parts = record
      .split(/[\s,]+/)
      .map((value) => Number(value))
      .filter((value) => Number.isFinite(value));

    if (parts.length < 3) {
      continue;
    }

    vertices.push(createVec3(parts[0], parts[1], parts[2]));
  }

  return vertices.length >= 4 ? vertices : createDefaultConvexHullVertices();
}

function summarizeSnapshot(snapshot) {
  return `${snapshot.bodyCount} bodies | ${snapshot.characterCount ?? 0} characters | ${snapshot.clothCount ?? 0} cloths | ${snapshot.softBodyCount ?? 0} soft bodies | ${snapshot.particleCount ?? 0} particles | ${snapshot.colliderCount} colliders | ${snapshot.jointCount} joints | ${snapshot.collision.summary.pairCount} pairs | ${snapshot.collision.summary.contactCount} contacts | ${snapshot.collision.summary.sensorPairCount ?? 0} triggers | ${snapshot.collision.summary.islandCount} islands | ${snapshot.collision.summary.sleepingBodyCount} sleeping | ${snapshot.materialCount} materials | gravity ${formatVector(snapshot.gravity)} | camera pos ${formatVector(snapshot.debugCamera.position)} | camera angles ${formatVector(snapshot.debugCamera.target)} | frames ${snapshot.renderFrameCount}`;
}

function joinIds(records) {
  if (!records.length) {
    return 'none';
  }

  return records.map((record) => record.id).join(', ');
}

function summarizeDeformableContacts(snapshot, deformableId) {
  const contacts = Array.isArray(snapshot?.xpbd?.lastCollisionContacts)
    ? snapshot.xpbd.lastCollisionContacts.filter((contact) => contact.deformableId === deformableId)
    : [];

  let staticCount = 0;
  let dynamicCount = 0;
  for (const contact of contacts) {
    if (contact.motionType === 'dynamic') {
      dynamicCount += 1;
    } else {
      staticCount += 1;
    }
  }

  return {
    total: contacts.length,
    staticCount,
    dynamicCount
  };
}

export class Engine3D {
  constructor(hostBridge) {
    this.hostBridge = hostBridge;
    this.world = new PhysicsWorld();
    this.lastFrame = null;
    this.lastSceneIoSummary = 'No scene import/export yet';
  }

  resetScene() {
    this.world.reset();
    this.lastFrame = null;
    this.hostBridge.log('Scene reset');
  }

  resetWorld() {
    this.resetScene();
  }

  getConvexHullPresetVertices(presetId, scale) {
    return getConvexHullPresetVertexText(presetId, scale);
  }

  setCameraPosition(x, y, z) {
    this.world.setDebugCameraPosition(createVec3(x, y, z));
    this.hostBridge.log(`Camera moved to ${x}, ${y}, ${z}`);
  }

  setCameraTarget(x, y, z) {
    this.world.setDebugCameraTarget(createVec3(x, y, z));
    this.hostBridge.log(`Camera angles set to pitch ${x}, yaw ${y}, roll ${z}`);
  }

  setGravity(x, y, z) {
    this.world.setGravity(createVec3(x, y, z));
    this.hostBridge.log(`Gravity set to ${x}, ${y}, ${z}`);
  }

  createMaterial(id, friction, restitution, density) {
    const material = this.world.createMaterial({
      id,
      friction,
      restitution,
      density
    });

    this.hostBridge.log(`Material ${material.id} registered`);
    return material;
  }

  addCube(id, x, y, z, size) {
    const { body } = this.world.createBoxBody({
      id,
      position: createVec3(x, y, z),
      size
    });

    this.hostBridge.log(`Cube ${body.id} registered`);
    return body;
  }

  createBoxRigidBody(id, x, y, z, size, mass, materialId = '') {
    const { body, collider } = this.world.createBoxBody({
      id,
      position: createVec3(x, y, z),
      size,
      mass,
      materialId
    });

    this.hostBridge.log(`Rigid body ${body.id} registered with collider ${collider.id}`);
    return body;
  }

  createStaticBoxCollider(id, x, y, z, size, materialId = '') {
    const { collider } = this.world.createStaticBoxCollider({
      id,
      position: createVec3(x, y, z),
      size,
      materialId
    });

    this.hostBridge.log(`Static collider ${collider.id} registered`);
    return collider;
  }

  createStaticBoxSensor(id, x, y, z, size, collisionLayer = 1, collisionMask = 0x7fffffff) {
    const { collider } = this.world.createStaticBoxSensor({
      id,
      position: createVec3(x, y, z),
      size,
      collisionLayer,
      collisionMask
    });

    this.hostBridge.log(`Static sensor ${collider.id} registered`);
    return collider;
  }

  createKinematicCapsule(id, x, y, z, radius, halfHeight, collisionLayer = 1, collisionMask = 0x7fffffff) {
    const character = this.world.createKinematicCapsule({
      id,
      position: createVec3(x, y, z),
      radius,
      halfHeight,
      collisionLayer,
      collisionMask
    });

    if (!character) {
      this.hostBridge.log(`Kinematic capsule ${id} could not be created`);
      return null;
    }

    this.hostBridge.log(`Kinematic capsule ${character.id} registered with body ${character.bodyId}`);
    return character;
  }

  configureKinematicCapsule(id, skinWidth, groundProbeDistance, maxGroundAngleDegrees) {
    const character = this.world.configureKinematicCapsule(id, {
      skinWidth,
      groundProbeDistance,
      maxGroundAngleDegrees
    });

    if (!character) {
      this.hostBridge.log(`Kinematic capsule ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Kinematic capsule ${character.id} configured`);
    return character;
  }

  configureKinematicController(id, jumpSpeed, gravityScale, stepOffset, groundSnapDistance) {
    const character = this.world.configureKinematicController(id, {
      jumpSpeed,
      gravityScale,
      stepOffset,
      groundSnapDistance
    });

    if (!character) {
      this.hostBridge.log(`Kinematic controller ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Kinematic controller ${character.id} configured`);
    return character;
  }

  setKinematicCapsuleMoveIntent(id, dx, dy, dz) {
    return this.world.setKinematicCapsuleMoveIntent(id, createVec3(dx, dy, dz));
  }

  jumpKinematicCapsule(id) {
    const character = this.world.jumpKinematicCapsule(id);
    if (!character) {
      this.hostBridge.log(`Kinematic capsule ${id} could not jump`);
      return null;
    }

    this.hostBridge.log(`Kinematic capsule ${character.id} jump requested`);
    return character;
  }

  moveKinematicCapsule(id, dx, dy, dz) {
    return this.world.moveKinematicCapsule(id, createVec3(dx, dy, dz));
  }

  createConvexHullRigidBody(id, verticesText, x, y, z, mass, materialId = '') {
    const vertices = parseConvexHullVertices(verticesText);
    const { body, collider } = this.world.createConvexHullBody({
      id,
      position: createVec3(x, y, z),
      vertices,
      mass,
      materialId
    });

    this.hostBridge.log(`Convex hull body ${body.id} registered with collider ${collider.id} and ${vertices.length} vertices`);
    return body;
  }

  createPresetConvexHullRigidBody(id, presetId, x, y, z, scale, mass, materialId = '') {
    return this.createConvexHullRigidBody(
      id,
      this.getConvexHullPresetVertices(presetId, scale),
      x,
      y,
      z,
      mass,
      materialId
    );
  }

  createStaticConvexHullCollider(id, verticesText, x, y, z, materialId = '') {
    const vertices = parseConvexHullVertices(verticesText);
    const { collider } = this.world.createStaticConvexHullCollider({
      id,
      position: createVec3(x, y, z),
      vertices,
      materialId
    });

    this.hostBridge.log(`Static convex hull collider ${collider.id} registered with ${vertices.length} vertices`);
    return collider;
  }

  createStaticConvexHullSensor(id, verticesText, x, y, z, collisionLayer = 1, collisionMask = 0x7fffffff) {
    const vertices = parseConvexHullVertices(verticesText);
    const { collider } = this.world.createStaticConvexHullSensor({
      id,
      position: createVec3(x, y, z),
      vertices,
      collisionLayer,
      collisionMask
    });

    this.hostBridge.log(`Static convex hull sensor ${collider.id} registered with ${vertices.length} vertices`);
    return collider;
  }

  createPresetStaticConvexHullCollider(id, presetId, x, y, z, scale, materialId = '') {
    return this.createStaticConvexHullCollider(
      id,
      this.getConvexHullPresetVertices(presetId, scale),
      x,
      y,
      z,
      materialId
    );
  }

  createPresetStaticConvexHullSensor(id, presetId, x, y, z, scale, collisionLayer = 1, collisionMask = 0x7fffffff) {
    return this.createStaticConvexHullSensor(
      id,
      this.getConvexHullPresetVertices(presetId, scale),
      x,
      y,
      z,
      collisionLayer,
      collisionMask
    );
  }

  createClothSheet(id, rows, columns, spacing, x, y, z, pinMode = 'top-row') {
    const cloth = this.world.createClothSheet({
      id,
      rows,
      columns,
      spacing,
      position: createVec3(x, y, z),
      pinMode
    });

    this.hostBridge.log(`Cloth ${cloth.id} registered with ${cloth.rows}x${cloth.columns} particles and pin mode ${cloth.pinMode}`);
    return cloth;
  }

  createSoftBodyCube(id, rows, columns, layers, spacing, x, y, z, pinMode = 'none') {
    const softBody = this.world.createSoftBodyCube({
      id,
      rows,
      columns,
      layers,
      spacing,
      position: createVec3(x, y, z),
      pinMode
    });

    this.hostBridge.log(`Soft body ${softBody.id} registered with ${softBody.layers}x${softBody.rows}x${softBody.columns} particles and pin mode ${softBody.pinMode}`);
    return softBody;
  }

  configureCloth(id, damping, margin, stretch, shear, bend, selfCollisionEnabled, selfCollisionDistance) {
    const cloth = this.world.configureCloth(id, {
      damping,
      collisionMargin: margin,
      stretchCompliance: stretch,
      shearCompliance: shear,
      bendCompliance: bend,
      selfCollisionEnabled,
      selfCollisionDistance
    });

    if (!cloth) {
      this.hostBridge.log(`Cloth ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Cloth ${cloth.id} configured`);
    return cloth;
  }

  configureClothPreset(id, presetId) {
    const preset = resolveClothPreset(presetId);
    const cloth = this.world.configureCloth(id, {
      damping: preset.damping,
      collisionMargin: preset.collisionMargin,
      stretchCompliance: preset.stretchCompliance,
      shearCompliance: preset.shearCompliance,
      bendCompliance: preset.bendCompliance,
      selfCollisionEnabled: preset.selfCollisionEnabled,
      selfCollisionDistance: preset.selfCollisionDistance
    });

    if (!cloth) {
      this.hostBridge.log(`Cloth ${id} could not be configured with preset ${preset.id}`);
      return null;
    }

    this.hostBridge.log(`Cloth ${cloth.id} configured with preset ${preset.id}`);
    return cloth;
  }

  configureSoftBody(id, damping, margin, stretch, shear, bend, volume) {
    const softBody = this.world.configureSoftBody(id, {
      damping,
      collisionMargin: margin,
      stretchCompliance: stretch,
      shearCompliance: shear,
      bendCompliance: bend,
      volumeCompliance: volume
    });

    if (!softBody) {
      this.hostBridge.log(`Soft body ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Soft body ${softBody.id} configured`);
    return softBody;
  }

  configureSoftBodyPreset(id, presetId) {
    const preset = resolveSoftBodyPreset(presetId);
    const softBody = this.world.configureSoftBody(id, {
      damping: preset.damping,
      collisionMargin: preset.collisionMargin,
      stretchCompliance: preset.stretchCompliance,
      shearCompliance: preset.shearCompliance,
      bendCompliance: preset.bendCompliance,
      volumeCompliance: preset.volumeCompliance
    });

    if (!softBody) {
      this.hostBridge.log(`Soft body ${id} could not be configured with preset ${preset.id}`);
      return null;
    }

    this.hostBridge.log(`Soft body ${softBody.id} configured with preset ${preset.id}`);
    return softBody;
  }

  configureBodyCollision(id, collisionLayer, collisionMask) {
    const body = this.world.configureBodyCollision(id, {
      collisionLayer,
      collisionMask
    });

    if (!body) {
      this.hostBridge.log(`Rigid body ${id} could not be configured for collision filtering`);
      return null;
    }

    this.hostBridge.log(`Rigid body ${body.id} collision filtering configured`);
    return body;
  }

  configureColliderCollision(id, collisionLayer, collisionMask, isSensor) {
    const collider = this.world.configureColliderCollision(id, {
      collisionLayer,
      collisionMask,
      isSensor
    });

    if (!collider) {
      this.hostBridge.log(`Collider ${id} could not be configured for collision filtering`);
      return null;
    }

    this.hostBridge.log(`Collider ${collider.id} collision filtering configured`);
    return collider;
  }

  createDistanceJoint(id, bodyAId, bodyBId, distance = 0) {
    const joint = this.world.createDistanceJoint({
      id,
      bodyAId,
      bodyBId,
      distance: distance > 0 ? distance : undefined
    });

    if (!joint) {
      this.hostBridge.log(`Distance joint ${id} could not be created`);
      return null;
    }

    this.hostBridge.log(`Distance joint ${joint.id} registered between ${joint.bodyAId} and ${joint.bodyBId}`);
    return joint;
  }

  createPointToPointJoint(id, bodyAId, bodyBId, x, y, z) {
    const joint = this.world.createPointToPointJoint({
      id,
      bodyAId,
      bodyBId,
      worldAnchor: createVec3(x, y, z)
    });

    if (!joint) {
      this.hostBridge.log(`Point-to-point joint ${id} could not be created`);
      return null;
    }

    this.hostBridge.log(`Point-to-point joint ${joint.id} registered between ${joint.bodyAId} and ${joint.bodyBId}`);
    return joint;
  }

  createHingeJoint(id, bodyAId, bodyBId, x, y, z, axisX, axisY, axisZ) {
    const joint = this.world.createHingeJoint({
      id,
      bodyAId,
      bodyBId,
      worldAnchor: createVec3(x, y, z),
      worldAxis: createVec3(axisX, axisY, axisZ)
    });

    if (!joint) {
      this.hostBridge.log(`Hinge joint ${id} could not be created`);
      return null;
    }

    this.hostBridge.log(`Hinge joint ${joint.id} registered between ${joint.bodyAId} and ${joint.bodyBId}`);
    return joint;
  }

  createFixedJoint(id, bodyAId, bodyBId, x, y, z) {
    const joint = this.world.createFixedJoint({
      id,
      bodyAId,
      bodyBId,
      worldAnchor: createVec3(x, y, z)
    });

    if (!joint) {
      this.hostBridge.log(`Fixed joint ${id} could not be created`);
      return null;
    }

    this.hostBridge.log(`Fixed joint ${joint.id} registered between ${joint.bodyAId} and ${joint.bodyBId}`);
    return joint;
  }

  configureDistanceJoint(id, minLength, maxLength, spring, damping) {
    const joint = this.world.configureDistanceJoint(id, {
      minDistance: minLength,
      maxDistance: maxLength,
      springFrequency: spring,
      dampingRatio: damping
    });

    if (!joint) {
      this.hostBridge.log(`Distance joint ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Distance joint ${joint.id} configured`);
    return joint;
  }

  configureHingeJoint(id, lowerAngleDegrees, upperAngleDegrees, damping) {
    const joint = this.world.configureHingeJoint(id, {
      lowerAngle: lowerAngleDegrees * (Math.PI / 180),
      upperAngle: upperAngleDegrees * (Math.PI / 180),
      angularDamping: damping
    });

    if (!joint) {
      this.hostBridge.log(`Hinge joint ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Hinge joint ${joint.id} configured`);
    return joint;
  }

  configureFixedJoint(id, breakForce, breakTorque) {
    const joint = this.world.configureFixedJoint(id, {
      breakForce,
      breakTorque
    });

    if (!joint) {
      this.hostBridge.log(`Fixed joint ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Fixed joint ${joint.id} configured`);
    return joint;
  }

  configureHingeMotor(id, speedDegreesPerSecond, maxTorque) {
    const joint = this.world.configureHingeMotor(id, {
      motorMode: 'speed',
      motorEnabled: maxTorque > 0,
      motorSpeed: speedDegreesPerSecond * (Math.PI / 180),
      maxMotorTorque: maxTorque
    });

    if (!joint) {
      this.hostBridge.log(`Hinge motor ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Hinge motor ${joint.id} configured`);
    return joint;
  }

  configureHingeServo(id, targetAngleDegrees, maxSpeedDegreesPerSecond, maxTorque) {
    const joint = this.world.configureHingeServo(id, {
      motorEnabled: maxTorque > 0,
      motorTargetAngle: targetAngleDegrees * (Math.PI / 180),
      maxMotorSpeed: maxSpeedDegreesPerSecond * (Math.PI / 180),
      maxMotorTorque: maxTorque
    });

    if (!joint) {
      this.hostBridge.log(`Hinge servo ${id} could not be configured`);
      return null;
    }

    this.hostBridge.log(`Hinge servo ${joint.id} configured`);
    return joint;
  }

  raycast(x, y, z, dx, dy, dz, length) {
    const result = this.world.raycast({
      origin: createVec3(x, y, z),
      direction: createVec3(dx, dy, dz),
      maxDistance: length
    });

    this.hostBridge.log(this.getLastRaycastSummary());
    return result;
  }

  stepWorld(seconds) {
    const stats = this.world.step(seconds);
    this.hostBridge.log(`Physics world stepped ${stats.performedSubsteps} substeps`);
    return stats;
  }

  sphereCast(x, y, z, radius, dx, dy, dz, length) {
    const result = this.world.sphereCast({
      origin: createVec3(x, y, z),
      radius,
      direction: createVec3(dx, dy, dz),
      maxDistance: length
    });

    this.hostBridge.log(this.getLastShapeCastSummary());
    return result;
  }

  capsuleCast(x, y, z, radius, halfHeight, dx, dy, dz, length) {
    const result = this.world.capsuleCast({
      origin: createVec3(x, y, z),
      radius,
      halfHeight,
      direction: createVec3(dx, dy, dz),
      maxDistance: length
    });

    this.hostBridge.log(this.getLastShapeCastSummary());
    return result;
  }

  renderDebugFrame() {
    const debugFrame = this.world.buildDebugFrame();
    const snapshot = this.world.getSnapshot();

    this.lastFrame = {
      frameNumber: debugFrame.frameNumber,
      debugFrame,
      plannedDrawCalls: debugFrame.primitives,
      snapshot,
      summary: `${this.hostBridge.getDisplayName()} frame ${debugFrame.frameNumber} | ${debugFrame.primitives.length} debug primitives | ${debugFrame.stats.broadphasePairCount} pairs | ${debugFrame.stats.contactPairCount} contacts`
    };

    this.hostBridge.emitFrame(this.lastFrame);
    return this.lastFrame;
  }

  showDebugOverlay() {
    if (typeof this.hostBridge.showDebugOverlay === 'function') {
      this.hostBridge.showDebugOverlay();
    }
  }

  hideDebugOverlay() {
    if (typeof this.hostBridge.hideDebugOverlay === 'function') {
      this.hostBridge.hideDebugOverlay();
    }
  }

  setDebugOverlayLayers(layersText) {
    if (typeof this.hostBridge.setDebugOverlayLayers === 'function') {
      this.hostBridge.setDebugOverlayLayers(layersText);
    }
  }

  resetDebugOverlayLayers() {
    if (typeof this.hostBridge.resetDebugOverlayLayers === 'function') {
      this.hostBridge.resetDebugOverlayLayers();
    }
  }

  exportSceneJson() {
    const sceneDefinition = this.world.exportSceneDefinition();
    const clothCount = sceneDefinition.xpbd?.cloths?.length ?? 0;
    const softBodyCount = sceneDefinition.xpbd?.softBodies?.length ?? 0;
    this.lastSceneIoSummary = `Scene exported | ${sceneDefinition.bodies.length} bodies | ${sceneDefinition.characters?.length ?? 0} characters | ${clothCount} cloths | ${softBodyCount} soft bodies | ${sceneDefinition.colliders.length} colliders | ${sceneDefinition.joints.length} joints | ${sceneDefinition.materials.length} materials`;
    return JSON.stringify(sceneDefinition);
  }

  loadSceneJson(sceneJsonText) {
    try {
      const parsedScene = JSON.parse(String(sceneJsonText ?? '').trim() || '{}');
      const imported = this.world.importSceneDefinition(parsedScene, {
        reset: true
      });
      this.lastFrame = null;
      this.lastSceneIoSummary = `Scene loaded | ${imported.bodyCount} bodies | ${imported.characterCount ?? 0} characters | ${imported.clothCount ?? 0} cloths | ${imported.softBodyCount ?? 0} soft bodies | ${imported.colliderCount} colliders | ${imported.jointCount} joints | ${imported.materialCount} materials`;
      this.hostBridge.log(this.lastSceneIoSummary);
      return true;
    } catch (error) {
      const message = `Scene load failed | ${error?.message ?? 'invalid JSON'}`;
      this.lastSceneIoSummary = message;
      this.hostBridge.log(message);
      return false;
    }
  }

  getSceneSummary() {
    return summarizeSnapshot(this.world.getSnapshot());
  }

  getWorldSummary() {
    return summarizeSnapshot(this.world.getSnapshot());
  }

  getRigidBodySummary(id) {
    const body = this.world.getBody(id);
    if (!body) {
      return `Rigid body ${id} not found`;
    }

    const primaryCollider = body.primaryColliderId ? this.world.getCollider(body.primaryColliderId) : null;
    const collisionSummary = primaryCollider
      ? ` | layer:${primaryCollider.collisionLayer} | mask:${primaryCollider.collisionMask} | sensor:${primaryCollider.isSensor ? 'on' : 'off'}`
      : '';
    return `${body.id} | motion:${body.motionType} | sleeping:${body.sleeping ? 'yes' : 'no'} | position ${formatVector(body.position)} | velocity ${formatVector(body.linearVelocity)} | colliders ${body.colliderIds.length}${collisionSummary}`;
  }

  getKinematicCapsuleSummary(id) {
    const character = this.world.getKinematicCapsule(id);
    if (!character) {
      return `Kinematic capsule ${id} not found`;
    }

    const body = character.bodyId ? this.world.getBody(character.bodyId) : null;
    const collider = character.colliderId ? this.world.getCollider(character.colliderId) : null;
    return `${character.id} | body:${character.bodyId} | collider:${character.colliderId} | radius:${character.radius} | half height:${character.halfHeight} | skin:${character.skinWidth} | probe:${character.groundProbeDistance} | max slope:${character.maxGroundAngleDegrees} | jump:${character.jumpSpeed} | gravity scale:${character.gravityScale} | step offset:${character.stepOffset} | snap:${character.groundSnapDistance} | grounded:${character.grounded ? 'yes' : 'no'} | walkable:${character.walkable ? 'yes' : 'no'} | vertical velocity:${formatOptionalNumber(character.verticalVelocity)} | move intent ${formatVector(character.moveIntent ?? createVec3())} | position ${formatVector(body?.position ?? createVec3())}${collider ? ` | layer:${collider.collisionLayer} | mask:${collider.collisionMask}` : ''}`;
  }

  getKinematicGroundSummary(id) {
    const groundState = this.world.getKinematicGroundState(id);
    if (!groundState) {
      return `Kinematic capsule ${id} not found`;
    }

    return `${id} ground | grounded:${groundState.grounded ? 'yes' : 'no'} | walkable:${groundState.walkable ? 'yes' : 'no'} | distance:${formatOptionalNumber(groundState.distance)} | angle:${formatOptionalNumber(groundState.angleDegrees)}deg | collider:${groundState.colliderId || 'none'} | body:${groundState.bodyId || 'static'} | point ${formatVector(groundState.point)} | normal ${formatVector(groundState.normal)}`;
  }

  isKinematicCapsuleGrounded(id) {
    return this.world.isKinematicCapsuleGrounded(id);
  }

  getColliderSummary(id) {
    const collider = this.world.getCollider(id);
    if (!collider) {
      return `Collider ${id} not found`;
    }

    const pose = this.world.getColliderWorldPose(id);
    return `${collider.id} | body:${collider.bodyId || 'static'} | shape:${collider.shapeId} | material:${collider.materialId} | sensor:${collider.isSensor ? 'on' : 'off'} | layer:${collider.collisionLayer} | mask:${collider.collisionMask} | position ${formatVector(pose.position)}`;
  }

  getMaterialSummary(id) {
    const material = this.world.getMaterial(id);
    if (!material) {
      return `Material ${id} not found`;
    }

    return `${material.id} | friction:${material.friction} | restitution:${material.restitution} | density:${material.density}`;
  }

  getJointSummary(id) {
    const joint = this.world.getJoint(id);
    if (!joint) {
      return `Joint ${id} not found`;
    }

    const anchors = this.world.getJointWorldAnchors(id);
    const axes = this.world.getJointWorldAxes(id);
    const axisSummary = axes ? ` | axisA ${formatVector(axes.axisA)} | axisB ${formatVector(axes.axisB)}` : '';
    const rangeSummary = ` | range:${formatOptionalNumber(joint.minDistance)}..${formatOptionalNumber(joint.maxDistance)} | spring:${formatOptionalNumber(joint.springFrequency, '0')} | damping:${formatOptionalNumber(joint.dampingRatio, '0')}`;
    const hingeAngle = this.world.getJointAngle(id);
    const hingeSummary = joint.type === 'hinge-joint'
      ? ` | angle:${formatOptionalNumber(radiansToDegrees(hingeAngle))}deg | limits:${formatOptionalNumber(radiansToDegrees(joint.lowerAngle))}..${formatOptionalNumber(radiansToDegrees(joint.upperAngle))}deg | angular damping:${formatOptionalNumber(joint.angularDamping, '0')} | motor mode:${joint.motorMode ?? 'speed'} | motor:${formatOptionalNumber(radiansToDegrees(joint.motorSpeed), '0')}deg/s @ ${formatOptionalNumber(joint.maxMotorTorque, '0')}${joint.motorMode === 'servo' ? ` | target:${formatOptionalNumber(radiansToDegrees(joint.motorTargetAngle), '0')}deg` : ''}`
      : '';
    const fixedSummary = joint.type === 'fixed-joint'
      ? ` | break force:${formatOptionalNumber(joint.breakForce)} | break torque:${formatOptionalNumber(joint.breakTorque)}`
      : '';
    return `${joint.id} | type:${joint.type} | enabled:${joint.enabled !== false ? 'yes' : 'no'} | broken:${joint.broken === true ? 'yes' : 'no'} | force:${formatOptionalNumber(joint.lastAppliedForce, '0')} | torque:${formatOptionalNumber(joint.lastAppliedTorque, '0')} | bodies:${joint.bodyAId}->${joint.bodyBId} | distance:${joint.distance} | anchorA ${formatVector(anchors.anchorA)} | anchorB ${formatVector(anchors.anchorB)}${axisSummary}${rangeSummary}${hingeSummary}${fixedSummary}`;
  }

  getClothSummary(id) {
    const cloth = this.world.getCloth(id);
    if (!cloth) {
      return `Cloth ${id} not found`;
    }

    return `${cloth.id} | type:${cloth.type} | rows:${cloth.rows} | columns:${cloth.columns} | particles:${cloth.particleIds.length} | pin:${cloth.pinMode} | spacing:${cloth.spacing} | damping:${cloth.damping} | margin:${cloth.collisionMargin} | stretch:${cloth.stretchCompliance} | shear:${cloth.shearCompliance} | bend:${cloth.bendCompliance} | self:${cloth.selfCollisionEnabled ? 'on' : 'off'} | self distance:${cloth.selfCollisionDistance}`;
  }

  getSoftBodySummary(id) {
    const softBody = this.world.getSoftBody(id);
    if (!softBody) {
      return `Soft body ${id} not found`;
    }

    const snapshot = this.world.getSnapshot();
    const contacts = summarizeDeformableContacts(snapshot, softBody.id);

    return `${softBody.id} | type:${softBody.type} | layers:${softBody.layers} | rows:${softBody.rows} | columns:${softBody.columns} | particles:${softBody.particleIds.length} | pin:${softBody.pinMode} | spacing:${softBody.spacing} | damping:${softBody.damping} | margin:${softBody.collisionMargin} | stretch:${softBody.stretchCompliance} | shear:${softBody.shearCompliance} | bend:${softBody.bendCompliance} | volume:${softBody.volumeCompliance} | contacts:${contacts.total} | static contacts:${contacts.staticCount} | dynamic contacts:${contacts.dynamicCount}`;
  }

  queryPointBodies(x, y, z) {
    const result = this.world.queryPoint(createVec3(x, y, z));
    return `${result.count.bodies} bodies at point ${x}, ${y}, ${z} | ${joinIds(result.bodies)}`;
  }

  queryPointColliders(x, y, z) {
    const result = this.world.queryPoint(createVec3(x, y, z));
    return `${result.count.colliders} colliders at point ${x}, ${y}, ${z} | ${joinIds(result.colliders)}`;
  }

  queryAabbBodies(x, y, z, hx, hy, hz) {
    const result = this.world.queryAabb({
      center: createVec3(x, y, z),
      halfExtents: createVec3(hx, hy, hz)
    });

    return `${result.count.bodies} bodies in AABB ${x}, ${y}, ${z} | ${joinIds(result.bodies)}`;
  }

  queryAabbColliders(x, y, z, hx, hy, hz) {
    const result = this.world.queryAabb({
      center: createVec3(x, y, z),
      halfExtents: createVec3(hx, hy, hz)
    });

    return `${result.count.colliders} colliders in AABB ${x}, ${y}, ${z} | ${joinIds(result.colliders)}`;
  }

  queryBodyContacts(id) {
    const body = this.world.getBody(id);
    if (!body) {
      return `Rigid body ${id} not found`;
    }

    const result = this.world.getBodiesTouchingBody(id);
    return `${result.count} bodies touching ${id} | ${formatIdentifierList(result.bodies)}`;
  }

  queryColliderContacts(id) {
    const collider = this.world.getCollider(id);
    if (!collider) {
      return `Collider ${id} not found`;
    }

    const result = this.world.getCollidersTouchingCollider(id);
    return `${result.count} colliders touching ${id} | ${formatIdentifierList(result.colliders)}`;
  }

  queryBodyContactEvents(id, phase) {
    const body = this.world.getBody(id);
    if (!body) {
      return `Rigid body ${id} not found`;
    }

    const resolvedPhase = normalizeEventPhase(phase);
    const result = this.world.getBodyContactEvents(id, resolvedPhase);
    return `${result.count} bodies in ${resolvedPhase} contact events for ${id} | ${formatIdentifierList(result.bodies)}`;
  }

  queryBodyTriggerEvents(id, phase) {
    const body = this.world.getBody(id);
    if (!body) {
      return `Rigid body ${id} not found`;
    }

    const resolvedPhase = normalizeEventPhase(phase);
    const result = this.world.getBodyTriggerEvents(id, resolvedPhase);
    return `${result.count} bodies in ${resolvedPhase} trigger events for ${id} | ${formatIdentifierList(result.bodies)}`;
  }

  queryColliderContactEvents(id, phase) {
    const collider = this.world.getCollider(id);
    if (!collider) {
      return `Collider ${id} not found`;
    }

    const resolvedPhase = normalizeEventPhase(phase);
    const result = this.world.getColliderContactEvents(id, resolvedPhase);
    return `${result.count} colliders in ${resolvedPhase} contact events for ${id} | ${formatIdentifierList(result.colliders)}`;
  }

  queryColliderTriggerEvents(id, phase) {
    const collider = this.world.getCollider(id);
    if (!collider) {
      return `Collider ${id} not found`;
    }

    const resolvedPhase = normalizeEventPhase(phase);
    const result = this.world.getColliderTriggerEvents(id, resolvedPhase);
    return `${result.count} colliders in ${resolvedPhase} trigger events for ${id} | ${formatIdentifierList(result.colliders)}`;
  }

  getContactEventsSummary() {
    const summary = this.world.getCollisionState().summary;
    return `contact events | enter:${summary.contactEnterCount} | stay:${summary.contactStayCount} | exit:${summary.contactExitCount}`;
  }

  getTriggerEventsSummary() {
    const summary = this.world.getCollisionState().summary;
    return `trigger events | enter:${summary.triggerEnterCount} | stay:${summary.triggerStayCount} | exit:${summary.triggerExitCount}`;
  }

  getLastRaycastSummary() {
    const raycast = this.world.getLastRaycast();
    if (!raycast) {
      return 'No raycast performed yet';
    }

    if (!raycast.hit) {
      return `Ray miss | origin ${formatVector(raycast.origin)} | direction ${formatVector(raycast.direction)} | length ${raycast.maxDistance}`;
    }

    return `Ray hit ${raycast.colliderId} | body:${raycast.bodyId || 'static'} | distance ${raycast.distance} | point ${formatVector(raycast.point)} | normal ${formatVector(raycast.normal)}`;
  }

  getLastShapeCastSummary() {
    const shapeCast = this.world.getLastShapeCast();
    if (!shapeCast) {
      return 'No shape cast performed yet';
    }

    if (!shapeCast.hit) {
      return `${shapeCast.castType} cast miss | origin ${formatVector(shapeCast.origin)} | direction ${formatVector(shapeCast.direction)} | length ${shapeCast.maxDistance}`;
    }

    return `${shapeCast.castType} cast hit ${shapeCast.colliderId} | body:${shapeCast.bodyId || 'static'} | distance ${shapeCast.distance} | point ${formatVector(shapeCast.point)} | normal ${formatVector(shapeCast.normal)}`;
  }

  getCcdSummary() {
    const events = this.world.getLastCcdEvents();
    if (!events.length) {
      return 'No CCD events in the last step';
    }

    const labels = events.map((event) => `${event.bodyId}->${event.targetColliderId}@${event.distance}`);
    return `${events.length} CCD events | ${labels.join(' | ')}`;
  }

  getLastFrameSummary() {
    if (!this.lastFrame) {
      return 'No frame rendered yet';
    }

    return this.lastFrame.summary;
  }

  getDebugOverlaySummary() {
    if (typeof this.hostBridge.getDebugOverlaySummary !== 'function') {
      return 'debug overlay unavailable';
    }

    return this.hostBridge.getDebugOverlaySummary();
  }

  getSceneIoSummary() {
    return this.lastSceneIoSummary;
  }

  getHostSummary() {
    const capabilities = this.hostBridge.getCapabilities();
    return `${this.hostBridge.getDisplayName()} | runtime:${capabilities.runtime ? 'yes' : 'no'} | renderer:${capabilities.renderer ? 'yes' : 'no'} | sandbox:${capabilities.sandbox ? 'yes' : 'no'}`;
  }
}
