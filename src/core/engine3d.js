import { PhysicsWorld, createVec3 } from '../physics/index.js';

function formatVector(vector) {
  return `${vector.x}, ${vector.y}, ${vector.z}`;
}

function formatOptionalNumber(value, fallback = 'none') {
  if (value === undefined || value === null || value === '') {
    return fallback;
  }

  return Number.isFinite(Number(value)) ? `${Number(value)}` : fallback;
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
  return `${snapshot.bodyCount} bodies | ${snapshot.colliderCount} colliders | ${snapshot.jointCount} joints | ${snapshot.collision.summary.pairCount} pairs | ${snapshot.collision.summary.contactCount} contacts | ${snapshot.collision.summary.islandCount} islands | ${snapshot.collision.summary.sleepingBodyCount} sleeping | ${snapshot.materialCount} materials | gravity ${formatVector(snapshot.gravity)} | camera pos ${formatVector(snapshot.debugCamera.position)} | camera angles ${formatVector(snapshot.debugCamera.target)} | frames ${snapshot.renderFrameCount}`;
}

function joinIds(records) {
  if (!records.length) {
    return 'none';
  }

  return records.map((record) => record.id).join(', ');
}

export class Engine3D {
  constructor(hostBridge) {
    this.hostBridge = hostBridge;
    this.world = new PhysicsWorld();
    this.lastFrame = null;
  }

  resetScene() {
    this.world.reset();
    this.lastFrame = null;
    this.hostBridge.log('Scene reset');
  }

  resetWorld() {
    this.resetScene();
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

    return `${body.id} | motion:${body.motionType} | sleeping:${body.sleeping ? 'yes' : 'no'} | position ${formatVector(body.position)} | velocity ${formatVector(body.linearVelocity)} | colliders ${body.colliderIds.length}`;
  }

  getColliderSummary(id) {
    const collider = this.world.getCollider(id);
    if (!collider) {
      return `Collider ${id} not found`;
    }

    const pose = this.world.getColliderWorldPose(id);
    return `${collider.id} | body:${collider.bodyId || 'static'} | shape:${collider.shapeId} | material:${collider.materialId} | position ${formatVector(pose.position)}`;
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

  getHostSummary() {
    const capabilities = this.hostBridge.getCapabilities();
    return `${this.hostBridge.getDisplayName()} | runtime:${capabilities.runtime ? 'yes' : 'no'} | renderer:${capabilities.renderer ? 'yes' : 'no'} | sandbox:${capabilities.sandbox ? 'yes' : 'no'}`;
  }
}
