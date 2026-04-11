import { computeLocalShapeAabb, computeShapeWorldAabb, createAabbFromCenterHalfExtents, testAabbOverlap, testPointInAabb } from '../collision/aabb.js';
import { buildBroadphasePairs, cloneBroadphasePair, cloneBroadphaseProxy, createBroadphaseProxy } from '../collision/broadphase.js';
import { cloneContactPair, runNarrowphase } from '../collision/narrowphase.js';
import { composePoses } from '../collision/support.js';
import { DEFAULT_DEBUG_COLORS, createDebugFrame, createDebugLine, createDebugPoint, createDebugWireBox } from '../debug/debug-primitives.js';
import { cloneManifold, ManifoldCache } from '../manifold/manifold-cache.js';
import { cloneQuat, createIdentityQuat, integrateQuat, inverseRotateVec3ByQuat, rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, addVec3, cloneVec3, createVec3, lengthSquaredVec3 } from '../math/vec3.js';
import { solveNormalContactConstraints } from '../rigid/normal-contact-solver.js';
import { BodyRegistry } from './body-registry.js';
import { ColliderRegistry } from './collider-registry.js';
import { MaterialRegistry } from './material-registry.js';
import { ShapeRegistry } from './shape-registry.js';

const DEFAULT_MATERIAL_ID = 'material-default';

function toPositiveNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
}

function toNonNegativeNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function cloneCamera(camera) {
  return {
    position: cloneVec3(camera.position)
  };
}

function createQueryResult(type, options = {}) {
  return {
    type,
    bodies: options.bodies ?? [],
    colliders: options.colliders ?? [],
    count: {
      bodies: options.bodies?.length ?? 0,
      colliders: options.colliders?.length ?? 0
    },
    input: options.input ?? {}
  };
}

function createEmptySolverStats(iterations = 0) {
  return {
    iterations,
    manifoldCount: 0,
    warmStartedContactCount: 0,
    solvedContactCount: 0,
    solvedTangentContactCount: 0,
    restitutionContactCount: 0,
    skippedContactCount: 0,
    impulsesApplied: 0,
    frictionImpulsesApplied: 0,
    positionCorrections: 0,
    maxPenetration: 0
  };
}

function cloneSolverStats(solverStats) {
  return {
    iterations: solverStats.iterations,
    manifoldCount: solverStats.manifoldCount,
    warmStartedContactCount: solverStats.warmStartedContactCount,
    solvedContactCount: solverStats.solvedContactCount,
    solvedTangentContactCount: solverStats.solvedTangentContactCount,
    restitutionContactCount: solverStats.restitutionContactCount,
    skippedContactCount: solverStats.skippedContactCount,
    impulsesApplied: solverStats.impulsesApplied,
    frictionImpulsesApplied: solverStats.frictionImpulsesApplied,
    positionCorrections: solverStats.positionCorrections,
    maxPenetration: solverStats.maxPenetration
  };
}

function createEmptyCollisionState(iterations = 0) {
  return {
    broadphaseProxies: [],
    broadphasePairs: [],
    contactPairs: [],
    manifolds: [],
    solverStats: createEmptySolverStats(iterations),
    summary: {
      proxyCount: 0,
      pairCount: 0,
      contactCount: 0,
      manifoldCount: 0,
      unsupportedPairCount: 0,
      pairKinds: {},
      algorithms: {}
    }
  };
}

function cloneCollisionSummary(summary) {
  return {
    proxyCount: summary.proxyCount,
    pairCount: summary.pairCount,
    contactCount: summary.contactCount,
    manifoldCount: summary.manifoldCount,
    unsupportedPairCount: summary.unsupportedPairCount,
    pairKinds: { ...summary.pairKinds },
    algorithms: { ...summary.algorithms }
  };
}

function cloneCollisionState(collisionState) {
  return {
    broadphaseProxies: collisionState.broadphaseProxies.map((proxy) => cloneBroadphaseProxy(proxy)),
    broadphasePairs: collisionState.broadphasePairs.map((pair) => cloneBroadphasePair(pair)),
    contactPairs: collisionState.contactPairs.map((contactPair) => cloneContactPair(contactPair)),
    manifolds: collisionState.manifolds.map((manifold) => cloneManifold(manifold)),
    solverStats: cloneSolverStats(collisionState.solverStats),
    summary: cloneCollisionSummary(collisionState.summary)
  };
}

function countByPairKind(pairs) {
  return pairs.reduce((counts, pair) => {
    counts[pair.pairKind] = (counts[pair.pairKind] ?? 0) + 1;
    return counts;
  }, {});
}

function mergeSolverStats(target, source) {
  target.iterations = Math.max(target.iterations, source.iterations);
  target.manifoldCount = Math.max(target.manifoldCount, source.manifoldCount);
  target.warmStartedContactCount += source.warmStartedContactCount;
  target.solvedContactCount += source.solvedContactCount;
  target.solvedTangentContactCount += source.solvedTangentContactCount;
  target.restitutionContactCount += source.restitutionContactCount;
  target.skippedContactCount += source.skippedContactCount;
  target.impulsesApplied += source.impulsesApplied;
  target.frictionImpulsesApplied += source.frictionImpulsesApplied;
  target.positionCorrections += source.positionCorrections;
  target.maxPenetration = Math.max(target.maxPenetration, source.maxPenetration);
}

function combineMaterialProperties(materialA, materialB) {
  const frictionA = Number(materialA?.friction ?? 0.5);
  const frictionB = Number(materialB?.friction ?? 0.5);
  const restitutionA = Number(materialA?.restitution ?? 0);
  const restitutionB = Number(materialB?.restitution ?? 0);

  return {
    friction: Math.sqrt(Math.max(0, frictionA) * Math.max(0, frictionB)),
    restitution: Math.max(0, restitutionA, restitutionB),
    restitutionThreshold: 1
  };
}

function safeInverse(value) {
  return value > 1e-8 ? 1 / value : 0;
}

function cloneDiagonalInverse(inertia) {
  return createVec3(
    safeInverse(Number(inertia?.x ?? 0)),
    safeInverse(Number(inertia?.y ?? 0)),
    safeInverse(Number(inertia?.z ?? 0))
  );
}

function computeBoxInertia(halfExtents, mass) {
  const hx = Math.abs(Number(halfExtents?.x ?? 0));
  const hy = Math.abs(Number(halfExtents?.y ?? 0));
  const hz = Math.abs(Number(halfExtents?.z ?? 0));

  return createVec3(
    (mass / 3) * (hy * hy + hz * hz),
    (mass / 3) * (hx * hx + hz * hz),
    (mass / 3) * (hx * hx + hy * hy)
  );
}

function computeSphereInertia(radius, mass) {
  const resolvedRadius = Math.abs(Number(radius ?? 0));
  const scalar = 0.4 * mass * resolvedRadius * resolvedRadius;
  return createVec3(scalar, scalar, scalar);
}

function computeCapsuleInertia(shape, mass) {
  const radius = Math.abs(Number(shape?.geometry?.radius ?? 0));
  const halfHeight = Math.abs(Number(shape?.geometry?.halfHeight ?? 0));
  return computeBoxInertia(createVec3(radius, halfHeight + radius, radius), mass);
}

function computeConvexHullInertia(shape, mass) {
  const localAabb = computeLocalShapeAabb(shape);
  return computeBoxInertia(localAabb?.halfExtents ?? createVec3(), mass);
}

function computeShapeInertia(shape, mass) {
  if (!shape || mass <= 0) {
    return createVec3();
  }

  if (shape.type === 'box') {
    return computeBoxInertia(shape.geometry.halfExtents, mass);
  }

  if (shape.type === 'sphere') {
    return computeSphereInertia(shape.geometry.radius, mass);
  }

  if (shape.type === 'capsule') {
    return computeCapsuleInertia(shape, mass);
  }

  if (shape.type === 'convex-hull') {
    return computeConvexHullInertia(shape, mass);
  }

  return createVec3();
}

function clearVector(vector) {
  vector.x = 0;
  vector.y = 0;
  vector.z = 0;
}

export class PhysicsWorld {
  constructor(options = {}) {
    this.fixedDeltaTime = toPositiveNumber(options.fixedDeltaTime, 1 / 60);
    this.maxSubsteps = Math.max(1, Math.floor(toPositiveNumber(options.maxSubsteps, 4)));
    this.gravity = cloneVec3(options.gravity ?? createVec3(0, -9.81, 0));
    this.solverIterations = Math.max(1, Math.floor(toPositiveNumber(options.solverIterations, 8)));
    this.solverBaumgarte = toNonNegativeNumber(options.solverBaumgarte, 0.2);
    this.allowedPenetration = toNonNegativeNumber(options.allowedPenetration, 0.01);
    this.positionCorrectionPercent = toNonNegativeNumber(options.positionCorrectionPercent, 0.8);
    this.shapeRegistry = new ShapeRegistry();
    this.bodyRegistry = new BodyRegistry();
    this.colliderRegistry = new ColliderRegistry();
    this.materialRegistry = new MaterialRegistry();
    this.manifoldCache = new ManifoldCache();
    this.resetRuntimeState();
    this.bootstrapDefaultMaterials();
  }

  bootstrapDefaultMaterials() {
    this.materialRegistry.createMaterial({
      id: DEFAULT_MATERIAL_ID,
      friction: 0.5,
      restitution: 0,
      density: 1
    });
  }

  resetRuntimeState() {
    this.debugCamera = {
      position: createVec3(0, 0, 400)
    };
    this.accumulatorSeconds = 0;
    this.simulationTick = 0;
    this.renderFrameCount = 0;
    this.lastStepStats = {
      requestedDeltaSeconds: 0,
      performedSubsteps: 0,
      simulationTick: 0,
      remainingAccumulatorSeconds: 0
    };
    this.lastSolverStats = createEmptySolverStats(this.solverIterations);
    this.collisionStateDirty = true;
    this.collisionState = createEmptyCollisionState(this.solverIterations);
    this.manifoldCache.clear();
  }

  reset() {
    this.shapeRegistry.clear();
    this.bodyRegistry.clear();
    this.colliderRegistry.clear();
    this.materialRegistry.clear();
    this.bootstrapDefaultMaterials();
    this.resetRuntimeState();
  }

  setDebugCameraPosition(position) {
    this.debugCamera.position = cloneVec3(position);
  }

  setGravity(gravity) {
    this.gravity = cloneVec3(gravity);
  }

  markCollisionStateDirty() {
    this.collisionStateDirty = true;
  }

  createMaterial(options = {}) {
    return this.materialRegistry.createMaterial(options);
  }

  resolveMaterialId(materialId) {
    const resolvedId = String(materialId ?? '').trim();
    if (resolvedId && this.materialRegistry.has(resolvedId)) {
      return resolvedId;
    }

    return DEFAULT_MATERIAL_ID;
  }

  createBoxShape(options = {}) {
    const shape = this.shapeRegistry.createBoxShape(options);
    this.markCollisionStateDirty();
    return shape;
  }

  createSphereShape(options = {}) {
    const shape = this.shapeRegistry.createSphereShape(options);
    this.markCollisionStateDirty();
    return shape;
  }

  createCapsuleShape(options = {}) {
    const shape = this.shapeRegistry.createCapsuleShape(options);
    this.markCollisionStateDirty();
    return shape;
  }

  createConvexHullShape(options = {}) {
    const shape = this.shapeRegistry.createConvexHullShape(options);
    this.markCollisionStateDirty();
    return shape;
  }

  createRigidBody(options = {}) {
    const body = this.bodyRegistry.createRigidBody(options);
    const updatedBody = this.updateBodyMassProperties(body.id) ?? body;
    this.markCollisionStateDirty();
    return updatedBody;
  }

  createCollider(options = {}) {
    const collider = this.colliderRegistry.createCollider({
      ...options,
      materialId: this.resolveMaterialId(options.materialId)
    });

    if (collider.bodyId) {
      this.bodyRegistry.attachCollider(collider.bodyId, collider.id, collider.shapeId);
      this.updateBodyMassProperties(collider.bodyId);
    }

    this.markCollisionStateDirty();
    return collider;
  }

  updateBodyMassProperties(bodyId) {
    const body = this.bodyRegistry.getMutable(bodyId);
    if (!body) {
      return null;
    }

    if (!body.enabled || body.motionType !== 'dynamic' || body.mass <= 0) {
      body.inverseMass = 0;
      body.inertia = createVec3();
      body.inverseInertia = createVec3();
      return this.getBody(bodyId);
    }

    const shape = body.shapeId ? this.getShape(body.shapeId) : null;
    const inertia = computeShapeInertia(shape, body.mass);
    body.inverseMass = safeInverse(body.mass);
    body.inertia = inertia;
    body.inverseInertia = cloneDiagonalInverse(inertia);
    return this.getBody(bodyId);
  }

  createBoxBody(options = {}) {
    const resolvedId = String(options.id ?? '').trim() || null;
    const size = toPositiveNumber(options.size, 1);
    const shape = this.createBoxShape({
      id: resolvedId ? `${resolvedId}:shape` : null,
      halfExtents: createVec3(size / 2, size / 2, size / 2),
      userData: options.shapeUserData ?? null
    });
    const body = this.createRigidBody({
      id: resolvedId,
      motionType: options.motionType ?? 'dynamic',
      position: options.position ?? createVec3(),
      rotation: options.rotation ?? createIdentityQuat(),
      linearVelocity: options.linearVelocity ?? createVec3(),
      angularVelocity: options.angularVelocity ?? createVec3(),
      mass: options.mass ?? 1,
      userData: options.bodyUserData ?? null
    });
    const collider = this.createCollider({
      id: resolvedId ? `${resolvedId}:collider` : null,
      shapeId: shape.id,
      bodyId: body.id,
      materialId: options.materialId,
      userData: options.colliderUserData ?? null
    });

    return {
      shape,
      body: this.getBody(body.id),
      collider
    };
  }

  createSphereBody(options = {}) {
    const resolvedId = String(options.id ?? '').trim() || null;
    const radius = toPositiveNumber(options.radius, 0.5);
    const shape = this.createSphereShape({
      id: resolvedId ? `${resolvedId}:shape` : null,
      radius,
      userData: options.shapeUserData ?? null
    });
    const body = this.createRigidBody({
      id: resolvedId,
      motionType: options.motionType ?? 'dynamic',
      position: options.position ?? createVec3(),
      rotation: options.rotation ?? createIdentityQuat(),
      linearVelocity: options.linearVelocity ?? createVec3(),
      angularVelocity: options.angularVelocity ?? createVec3(),
      mass: options.mass ?? 1,
      userData: options.bodyUserData ?? null
    });
    const collider = this.createCollider({
      id: resolvedId ? `${resolvedId}:collider` : null,
      shapeId: shape.id,
      bodyId: body.id,
      materialId: options.materialId,
      userData: options.colliderUserData ?? null
    });

    return {
      shape,
      body: this.getBody(body.id),
      collider
    };
  }

  createCapsuleBody(options = {}) {
    const resolvedId = String(options.id ?? '').trim() || null;
    const shape = this.createCapsuleShape({
      id: resolvedId ? `${resolvedId}:shape` : null,
      radius: options.radius,
      halfHeight: options.halfHeight,
      userData: options.shapeUserData ?? null
    });
    const body = this.createRigidBody({
      id: resolvedId,
      motionType: options.motionType ?? 'dynamic',
      position: options.position ?? createVec3(),
      rotation: options.rotation ?? createIdentityQuat(),
      linearVelocity: options.linearVelocity ?? createVec3(),
      angularVelocity: options.angularVelocity ?? createVec3(),
      mass: options.mass ?? 1,
      userData: options.bodyUserData ?? null
    });
    const collider = this.createCollider({
      id: resolvedId ? `${resolvedId}:collider` : null,
      shapeId: shape.id,
      bodyId: body.id,
      materialId: options.materialId,
      userData: options.colliderUserData ?? null
    });

    return {
      shape,
      body: this.getBody(body.id),
      collider
    };
  }

  createConvexHullBody(options = {}) {
    const resolvedId = String(options.id ?? '').trim() || null;
    const shape = this.createConvexHullShape({
      id: resolvedId ? `${resolvedId}:shape` : null,
      vertices: options.vertices ?? [],
      userData: options.shapeUserData ?? null
    });
    const body = this.createRigidBody({
      id: resolvedId,
      motionType: options.motionType ?? 'dynamic',
      position: options.position ?? createVec3(),
      rotation: options.rotation ?? createIdentityQuat(),
      linearVelocity: options.linearVelocity ?? createVec3(),
      angularVelocity: options.angularVelocity ?? createVec3(),
      mass: options.mass ?? 1,
      userData: options.bodyUserData ?? null
    });
    const collider = this.createCollider({
      id: resolvedId ? `${resolvedId}:collider` : null,
      shapeId: shape.id,
      bodyId: body.id,
      materialId: options.materialId,
      userData: options.colliderUserData ?? null
    });

    return {
      shape,
      body: this.getBody(body.id),
      collider
    };
  }

  createStaticBoxCollider(options = {}) {
    const resolvedId = String(options.id ?? '').trim() || null;
    const size = toPositiveNumber(options.size, 1);
    const shape = this.createBoxShape({
      id: resolvedId ? `${resolvedId}:shape` : null,
      halfExtents: createVec3(size / 2, size / 2, size / 2),
      userData: options.shapeUserData ?? null
    });
    const collider = this.createCollider({
      id: resolvedId ? `${resolvedId}:collider` : null,
      shapeId: shape.id,
      bodyId: null,
      materialId: options.materialId,
      localPose: {
        position: options.position ?? createVec3(),
        rotation: options.rotation ?? createIdentityQuat()
      },
      userData: options.colliderUserData ?? null
    });

    return {
      shape,
      collider
    };
  }

  getBody(id) {
    return this.bodyRegistry.get(id);
  }

  getShape(id) {
    return this.shapeRegistry.get(id);
  }

  getCollider(id) {
    return this.colliderRegistry.get(id);
  }

  getMaterial(id) {
    return this.materialRegistry.get(id);
  }

  getEffectiveMaterialForCollider(colliderId) {
    const collider = this.getCollider(colliderId);
    if (!collider) {
      return null;
    }

    return this.getMaterial(collider.materialId) ?? this.getMaterial(DEFAULT_MATERIAL_ID);
  }

  getBodyColliders(bodyId) {
    const body = this.getBody(bodyId);
    if (!body) {
      return [];
    }

    return body.colliderIds
      .map((colliderId) => this.getCollider(colliderId))
      .filter(Boolean);
  }

  getWorldSummary() {
    const collisionState = this.getCollisionState();

    return {
      bodyCount: this.bodyRegistry.count(),
      shapeCount: this.shapeRegistry.count(),
      colliderCount: this.colliderRegistry.count(),
      materialCount: this.materialRegistry.count(),
      broadphaseProxyCount: collisionState.summary.proxyCount,
      broadphasePairCount: collisionState.summary.pairCount,
      contactPairCount: collisionState.summary.contactCount,
      manifoldCount: collisionState.summary.manifoldCount,
      simulationTick: this.simulationTick,
      renderFrameCount: this.renderFrameCount,
      fixedDeltaTime: this.fixedDeltaTime,
      gravity: cloneVec3(this.gravity)
    };
  }

  step(deltaSeconds = this.fixedDeltaTime) {
    const requestedDeltaSeconds = toNonNegativeNumber(deltaSeconds, this.fixedDeltaTime);
    if (requestedDeltaSeconds === 0) {
      return {
        ...this.lastStepStats
      };
    }

    this.accumulatorSeconds += requestedDeltaSeconds;
    let performedSubsteps = 0;
    const aggregatedSolverStats = createEmptySolverStats(this.solverIterations);

    while (this.accumulatorSeconds + 1e-12 >= this.fixedDeltaTime && performedSubsteps < this.maxSubsteps) {
      this.integrateRigidBodies(this.fixedDeltaTime);
      const solverStats = this.solveRigidContacts(this.fixedDeltaTime, this.simulationTick + 1);
      mergeSolverStats(aggregatedSolverStats, solverStats);
      this.accumulatorSeconds -= this.fixedDeltaTime;
      this.simulationTick += 1;
      performedSubsteps += 1;
    }

    if (performedSubsteps === this.maxSubsteps && this.accumulatorSeconds > this.fixedDeltaTime) {
      this.accumulatorSeconds = this.fixedDeltaTime;
    }

    this.lastSolverStats = aggregatedSolverStats;
    if (!this.collisionStateDirty) {
      this.collisionState.solverStats = cloneSolverStats(this.lastSolverStats);
    }

    this.lastStepStats = {
      requestedDeltaSeconds,
      performedSubsteps,
      simulationTick: this.simulationTick,
      remainingAccumulatorSeconds: this.accumulatorSeconds
    };

    return {
      ...this.lastStepStats
    };
  }

  applyInverseInertia(body, worldVector) {
    if (!body || body.motionType !== 'dynamic' || !body.inverseInertia) {
      return createVec3();
    }

    const rotation = body.rotation ?? createIdentityQuat();
    const localVector = inverseRotateVec3ByQuat(rotation, worldVector ?? createVec3());
    const localResult = createVec3(
      localVector.x * Number(body.inverseInertia.x ?? 0),
      localVector.y * Number(body.inverseInertia.y ?? 0),
      localVector.z * Number(body.inverseInertia.z ?? 0)
    );
    return rotateVec3ByQuat(rotation, localResult);
  }

  integrateRigidBodies(deltaTime) {
    let movedAnyDynamicBody = false;

    this.bodyRegistry.forEachMutable((body) => {
      if (!body.enabled || body.sleeping || body.motionType !== 'dynamic') {
        return;
      }

      const linearAcceleration = addScaledVec3(this.gravity, body.forceAccumulator, body.inverseMass);
      const angularAcceleration = this.applyInverseInertia(body, body.torqueAccumulator);
      body.linearVelocity = addScaledVec3(body.linearVelocity, linearAcceleration, deltaTime);
      body.angularVelocity = addScaledVec3(body.angularVelocity, angularAcceleration, deltaTime);
      body.position = addScaledVec3(body.position, body.linearVelocity, deltaTime);
      body.rotation = integrateQuat(body.rotation, body.angularVelocity, deltaTime);
      movedAnyDynamicBody = movedAnyDynamicBody ||
        lengthSquaredVec3(body.linearVelocity) > 1e-12 ||
        lengthSquaredVec3(body.angularVelocity) > 1e-12 ||
        lengthSquaredVec3(linearAcceleration) > 1e-12 ||
        lengthSquaredVec3(angularAcceleration) > 1e-12;
      clearVector(body.forceAccumulator);
      clearVector(body.torqueAccumulator);
    });

    if (movedAnyDynamicBody) {
      this.markCollisionStateDirty();
    }
  }

  solveRigidContacts(deltaTime, simulationTick) {
    const initialResults = this.buildCollisionResults();
    const manifolds = this.manifoldCache.syncFromContactPairs(initialResults.contactPairs, simulationTick);
    const solverStats = solveNormalContactConstraints({
      bodyRegistry: this.bodyRegistry,
      manifolds,
      deltaTime,
      iterations: this.solverIterations,
      baumgarte: this.solverBaumgarte,
      allowedPenetration: this.allowedPenetration,
      positionCorrectionPercent: this.positionCorrectionPercent
    });

    this.lastSolverStats = solverStats;
    const finalResults = this.buildCollisionResults();
    const finalManifolds = this.manifoldCache.syncFromContactPairs(finalResults.contactPairs, simulationTick);
    this.commitCollisionState(finalResults, finalManifolds, solverStats);
    this.collisionStateDirty = false;
    return cloneSolverStats(solverStats);
  }

  getColliderWorldPose(colliderId) {
    const collider = this.colliderRegistry.get(colliderId);
    if (!collider) {
      return null;
    }

    if (!collider.bodyId) {
      return {
        position: cloneVec3(collider.localPose.position),
        rotation: cloneQuat(collider.localPose.rotation)
      };
    }

    const body = this.bodyRegistry.get(collider.bodyId);
    if (!body) {
      return {
        position: cloneVec3(collider.localPose.position),
        rotation: cloneQuat(collider.localPose.rotation)
      };
    }

    return composePoses({
      position: body.position,
      rotation: body.rotation ?? createIdentityQuat()
    }, collider.localPose);
  }

  buildBroadphaseProxies() {
    const broadphaseProxies = [];

    this.colliderRegistry.forEachMutable((mutableCollider) => {
      if (!mutableCollider.enabled || !mutableCollider.shapeId) {
        return;
      }

      const shape = this.shapeRegistry.get(mutableCollider.shapeId);
      const worldPose = this.getColliderWorldPose(mutableCollider.id);
      if (!shape || !worldPose) {
        return;
      }

      const aabb = computeShapeWorldAabb(shape, worldPose);
      if (!aabb) {
        return;
      }

      const body = mutableCollider.bodyId ? this.bodyRegistry.get(mutableCollider.bodyId) : null;
      broadphaseProxies.push(createBroadphaseProxy({
        colliderId: mutableCollider.id,
        bodyId: mutableCollider.bodyId,
        shapeId: shape.id,
        materialId: mutableCollider.materialId,
        shapeType: shape.type,
        motionType: body?.motionType ?? 'static',
        isSensor: mutableCollider.isSensor,
        aabb
      }));
    });

    return broadphaseProxies;
  }

  buildCollisionResults() {
    const broadphaseProxies = this.buildBroadphaseProxies();
    const broadphasePairs = buildBroadphasePairs(broadphaseProxies);
    const narrowphase = runNarrowphase(broadphasePairs, {
      getShape: (shapeId) => this.getShape(shapeId),
      getPose: (colliderId) => this.getColliderWorldPose(colliderId)
    });
    const contactPairs = narrowphase.contactPairs.map((contactPair) => {
      const materialA = this.getEffectiveMaterialForCollider(contactPair.colliderAId);
      const materialB = this.getEffectiveMaterialForCollider(contactPair.colliderBId);
      const materialProperties = combineMaterialProperties(materialA, materialB);

      return cloneContactPair({
        ...contactPair,
        ...materialProperties
      });
    });

    return {
      broadphaseProxies,
      broadphasePairs,
      contactPairs,
      unsupportedPairCount: narrowphase.summary.unsupportedPairCount,
      algorithms: contactPairs.reduce((counts, contactPair) => {
        counts[contactPair.algorithm] = (counts[contactPair.algorithm] ?? 0) + 1;
        return counts;
      }, {}),
      pairKinds: countByPairKind(broadphasePairs)
    };
  }

  commitCollisionState(results, manifolds, solverStats) {
    this.collisionState = {
      broadphaseProxies: results.broadphaseProxies,
      broadphasePairs: results.broadphasePairs,
      contactPairs: results.contactPairs,
      manifolds: Array.isArray(manifolds) ? manifolds.map((manifold) => cloneManifold(manifold)) : [],
      solverStats: cloneSolverStats(solverStats ?? this.lastSolverStats),
      summary: {
        proxyCount: results.broadphaseProxies.length,
        pairCount: results.broadphasePairs.length,
        contactCount: results.contactPairs.length,
        manifoldCount: Array.isArray(manifolds) ? manifolds.length : 0,
        unsupportedPairCount: results.unsupportedPairCount,
        pairKinds: { ...results.pairKinds },
        algorithms: { ...results.algorithms }
      }
    };
  }

  ensureCollisionState() {
    if (this.collisionStateDirty) {
      const results = this.buildCollisionResults();
      const manifolds = this.manifoldCache.syncFromContactPairs(results.contactPairs, this.simulationTick);
      this.commitCollisionState(results, manifolds, this.lastSolverStats);
      this.collisionStateDirty = false;
    }

    return this.collisionState;
  }

  getCollisionState() {
    return cloneCollisionState(this.ensureCollisionState());
  }

  queryPoint(point) {
    const resolvedPoint = cloneVec3(point);
    const collisionState = this.ensureCollisionState();
    const colliders = [];
    const bodies = [];
    const seenBodyIds = new Set();

    for (const proxy of collisionState.broadphaseProxies) {
      if (!testPointInAabb(resolvedPoint, proxy.aabb)) {
        continue;
      }

      const collider = this.colliderRegistry.get(proxy.colliderId);
      if (!collider) {
        continue;
      }

      colliders.push(collider);
      if (proxy.bodyId && !seenBodyIds.has(proxy.bodyId)) {
        const body = this.bodyRegistry.get(proxy.bodyId);
        if (body) {
          seenBodyIds.add(proxy.bodyId);
          bodies.push(body);
        }
      }
    }

    return createQueryResult('point', {
      bodies,
      colliders,
      input: {
        point: resolvedPoint
      }
    });
  }

  queryAabb(options = {}) {
    const center = cloneVec3(options.center ?? createVec3());
    const halfExtents = cloneVec3(options.halfExtents ?? createVec3(0.5, 0.5, 0.5));
    const queryBounds = createAabbFromCenterHalfExtents(center, halfExtents);
    const collisionState = this.ensureCollisionState();
    const colliders = [];
    const bodies = [];
    const seenBodyIds = new Set();

    for (const proxy of collisionState.broadphaseProxies) {
      if (!testAabbOverlap(queryBounds, proxy.aabb)) {
        continue;
      }

      const collider = this.colliderRegistry.get(proxy.colliderId);
      if (!collider) {
        continue;
      }

      colliders.push(collider);
      if (proxy.bodyId && !seenBodyIds.has(proxy.bodyId)) {
        const body = this.bodyRegistry.get(proxy.bodyId);
        if (body) {
          seenBodyIds.add(proxy.bodyId);
          bodies.push(body);
        }
      }
    }

    return createQueryResult('aabb', {
      bodies,
      colliders,
      input: {
        center,
        halfExtents
      }
    });
  }

  buildDebugFrame() {
    const collisionState = this.ensureCollisionState();
    this.renderFrameCount += 1;
    const primitives = [];

    for (const proxy of collisionState.broadphaseProxies) {
      const collider = this.getCollider(proxy.colliderId);
      const shape = this.getShape(proxy.shapeId);
      const pose = this.getColliderWorldPose(proxy.colliderId);
      if (!collider || !shape || !pose) {
        continue;
      }

      const body = proxy.bodyId ? this.bodyRegistry.get(proxy.bodyId) : null;
      const source = {
        bodyId: proxy.bodyId,
        colliderId: proxy.colliderId,
        shapeId: shape.id,
        materialId: collider.materialId
      };
      const wireHalfExtents = shape.type === 'box'
        ? shape.geometry.halfExtents
        : proxy.aabb.halfExtents;

      primitives.push(
        createDebugWireBox({
          id: `${proxy.colliderId}:wire-box`,
          category: body ? 'rigid-body' : 'static-collider',
          center: pose.position,
          halfExtents: wireHalfExtents,
          rotation: pose.rotation,
          color: body
            ? (body.sleeping ? DEFAULT_DEBUG_COLORS.sleepingRigidBodyWireframe : DEFAULT_DEBUG_COLORS.rigidBodyWireframe)
            : DEFAULT_DEBUG_COLORS.staticColliderWireframe,
          source
        })
      );

      primitives.push(
        createDebugPoint({
          id: `${proxy.colliderId}:center-of-mass`,
          category: body ? 'center-of-mass' : 'collider-origin',
          position: body?.position ?? pose.position,
          color: DEFAULT_DEBUG_COLORS.rigidBodyCenter,
          size: 4,
          source
        })
      );

      primitives.push(
        createDebugWireBox({
          id: `${proxy.colliderId}:aabb`,
          category: 'broadphase-aabb',
          center: proxy.aabb.center,
          halfExtents: proxy.aabb.halfExtents,
          color: DEFAULT_DEBUG_COLORS.broadphaseAabb,
          source
        })
      );
    }

    for (const manifold of collisionState.manifolds) {
      for (const contact of manifold.contacts) {
        const source = {
          pairKey: manifold.pairKey,
          colliderAId: manifold.colliderAId,
          colliderBId: manifold.colliderBId,
          accumulatedNormalImpulse: contact.accumulatedNormalImpulse
        };

        primitives.push(
          createDebugPoint({
            id: `${contact.id}:point`,
            category: 'contact-point',
            position: contact.position,
            color: DEFAULT_DEBUG_COLORS.contactPoint,
            size: 5,
            source
          })
        );

        primitives.push(
          createDebugLine({
            id: `${contact.id}:normal`,
            category: 'contact-normal',
            start: contact.position,
            end: addScaledVec3(contact.position, manifold.normal, Math.max(10, contact.penetration * 20)),
            color: DEFAULT_DEBUG_COLORS.contactNormal,
            source
          })
        );
      }
    }

    return createDebugFrame({
      frameNumber: this.renderFrameCount,
      simulationTick: this.simulationTick,
      camera: cloneCamera(this.debugCamera),
      primitives,
      stats: {
        bodyCount: this.bodyRegistry.count(),
        shapeCount: this.shapeRegistry.count(),
        colliderCount: this.colliderRegistry.count(),
        materialCount: this.materialRegistry.count(),
        broadphaseProxyCount: collisionState.summary.proxyCount,
        broadphasePairCount: collisionState.summary.pairCount,
        contactPairCount: collisionState.summary.contactCount,
        manifoldCount: collisionState.summary.manifoldCount,
        solverIterations: collisionState.solverStats.iterations,
        solvedContactCount: collisionState.solverStats.solvedContactCount,
        solvedTangentContactCount: collisionState.solverStats.solvedTangentContactCount,
        restitutionContactCount: collisionState.solverStats.restitutionContactCount,
        warmStartedContactCount: collisionState.solverStats.warmStartedContactCount,
        fixedDeltaTime: this.fixedDeltaTime,
        performedSubsteps: this.lastStepStats.performedSubsteps
      }
    });
  }

  getSnapshot() {
    const collisionState = this.getCollisionState();

    return {
      debugCamera: cloneCamera(this.debugCamera),
      bodyCount: this.bodyRegistry.count(),
      shapeCount: this.shapeRegistry.count(),
      colliderCount: this.colliderRegistry.count(),
      materialCount: this.materialRegistry.count(),
      bodies: this.bodyRegistry.list(),
      shapes: this.shapeRegistry.list(),
      colliders: this.colliderRegistry.list(),
      materials: this.materialRegistry.list(),
      collision: collisionState,
      simulationTick: this.simulationTick,
      renderFrameCount: this.renderFrameCount,
      fixedDeltaTime: this.fixedDeltaTime,
      gravity: cloneVec3(this.gravity),
      accumulatorSeconds: this.accumulatorSeconds,
      lastStepStats: {
        ...this.lastStepStats
      },
      lastSolverStats: cloneSolverStats(this.lastSolverStats)
    };
  }
}
