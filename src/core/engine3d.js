import { PhysicsWorld, createVec3 } from '../physics/index.js';

function formatVector(vector) {
  return `${vector.x}, ${vector.y}, ${vector.z}`;
}

function summarizeSnapshot(snapshot) {
  return `${snapshot.bodyCount} bodies | ${snapshot.colliderCount} colliders | ${snapshot.collision.summary.pairCount} pairs | ${snapshot.collision.summary.contactCount} contacts | ${snapshot.materialCount} materials | gravity ${formatVector(snapshot.gravity)} | camera ${formatVector(snapshot.debugCamera.position)} | frames ${snapshot.renderFrameCount}`;
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

  stepWorld(seconds) {
    const stats = this.world.step(seconds);
    this.hostBridge.log(`Physics world stepped ${stats.performedSubsteps} substeps`);
    return stats;
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

    return `${body.id} | motion:${body.motionType} | position ${formatVector(body.position)} | velocity ${formatVector(body.linearVelocity)} | colliders ${body.colliderIds.length}`;
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

  getLastFrameSummary() {
    if (!this.lastFrame) {
      return 'No frame rendered yet';
    }

    return this.lastFrame.summary;
  }

  getHostSummary() {
    const capabilities = this.hostBridge.getCapabilities();
    return `${this.hostBridge.getDisplayName()} | runtime:${capabilities.runtime ? 'yes' : 'no'} | renderer:${capabilities.renderer ? 'yes' : 'no'} | sandbox:${capabilities.sandbox ? 'yes' : 'no'}`;
  }
}
