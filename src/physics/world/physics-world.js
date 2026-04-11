import { computeLocalShapeAabb, computeShapeWorldAabb, createAabbFromCenterHalfExtents, intersectRayAabb, testAabbOverlap, testPointInAabb } from '../collision/aabb.js';
import { buildBroadphasePairs, cloneBroadphasePair, cloneBroadphaseProxy, createBroadphaseProxy } from '../collision/broadphase.js';
import { cloneContactPair, runNarrowphase } from '../collision/narrowphase.js';
import { capsuleCastShape, castConvexShapesWithToi, cloneRaycastResult, cloneShapeCastResult, computeSweptShapeAabb, createRaycastResult, createShapeCastResult, raycastShape, sphereCastShape } from '../collision/raycast.js';
import { composePoses, createTangentBasis, transformPointByPose } from '../collision/support.js';
import { DEFAULT_DEBUG_COLORS, createDebugFrame, createDebugLine, createDebugPoint, createDebugWireBox } from '../debug/debug-primitives.js';
import { cloneManifold, ManifoldCache } from '../manifold/manifold-cache.js';
import { cloneQuat, createIdentityQuat, integrateQuat, inverseRotateVec3ByQuat, rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, addVec3, cloneVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';
import { buildRigidBodyIslandGraph, cloneRigidBodyIslandGraph, cloneRigidBodyIsland } from '../rigid/islands.js';
import { solveJointConstraints } from '../rigid/distance-joint-solver.js';
import { solveNormalContactConstraints } from '../rigid/normal-contact-solver.js';
import { BodyRegistry } from './body-registry.js';
import { ColliderRegistry } from './collider-registry.js';
import { JointRegistry } from './joint-registry.js';
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

function toOptionalNumber(value, fallback = null) {
  if (value === undefined || value === null || value === '') {
    return fallback;
  }

  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function toOptionalNonNegativeNumber(value, fallback = null) {
  const parsed = toOptionalNumber(value, fallback);
  return parsed !== null && parsed >= 0 ? parsed : fallback;
}

function normalizeDistanceLimits(minDistance, maxDistance) {
  let resolvedMin = toOptionalNonNegativeNumber(minDistance, null);
  let resolvedMax = toOptionalNonNegativeNumber(maxDistance, null);

  if (resolvedMin !== null && resolvedMax !== null && resolvedMin > resolvedMax) {
    const temp = resolvedMin;
    resolvedMin = resolvedMax;
    resolvedMax = temp;
  }

  return {
    minDistance: resolvedMin,
    maxDistance: resolvedMax
  };
}

function clampDistanceToRange(distance, minDistance, maxDistance) {
  let resolvedDistance = toNonNegativeNumber(distance, 0);

  if (minDistance !== null) {
    resolvedDistance = Math.max(resolvedDistance, minDistance);
  }

  if (maxDistance !== null) {
    resolvedDistance = Math.min(resolvedDistance, maxDistance);
  }

  return resolvedDistance;
}

function normalizeAngleLimits(lowerAngle, upperAngle) {
  let resolvedLower = toOptionalNumber(lowerAngle, null);
  let resolvedUpper = toOptionalNumber(upperAngle, null);

  if (resolvedLower !== null && resolvedUpper !== null && resolvedLower > resolvedUpper) {
    const temp = resolvedLower;
    resolvedLower = resolvedUpper;
    resolvedUpper = temp;
  }

  return {
    lowerAngle: resolvedLower,
    upperAngle: resolvedUpper
  };
}

function cloneCamera(camera) {
  return {
    position: cloneVec3(camera.position),
    target: cloneVec3(camera.target)
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

function createRaycastMiss(options = {}) {
  const origin = cloneVec3(options.origin ?? createVec3());
  const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
  const maxDistance = toNonNegativeNumber(options.maxDistance, 0);

  return createRaycastResult({
    hit: false,
    origin,
    direction,
    maxDistance,
    proxyCount: options.proxyCount ?? 0,
    candidateCount: options.candidateCount ?? 0,
    testedShapeCount: options.testedShapeCount ?? 0
  });
}

function createShapeCastMiss(options = {}) {
  const origin = cloneVec3(options.origin ?? createVec3());
  const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
  const maxDistance = toNonNegativeNumber(options.maxDistance, 0);

  return createShapeCastResult({
    castType: options.castType ?? 'shape',
    hit: false,
    origin,
    direction,
    maxDistance,
    radius: options.radius ?? 0,
    halfHeight: options.halfHeight ?? 0,
    rotation: options.rotation ?? createIdentityQuat(),
    sampleOrigins: options.sampleOrigins ?? [],
    sampleEndPoints: options.sampleEndPoints ?? [],
    proxyCount: options.proxyCount ?? 0,
    candidateCount: options.candidateCount ?? 0,
    testedShapeCount: options.testedShapeCount ?? 0
  });
}

function cloneOptionalVec3(vector) {
  return vector ? cloneVec3(vector) : null;
}

function createCcdEvent(options = {}) {
  return {
    bodyId: String(options.bodyId ?? '').trim() || null,
    colliderId: String(options.colliderId ?? '').trim() || null,
    castType: String(options.castType ?? 'shape').trim() || 'shape',
    startPosition: cloneVec3(options.startPosition ?? createVec3()),
    endPosition: cloneVec3(options.endPosition ?? createVec3()),
    hitPosition: cloneOptionalVec3(options.hitPosition ?? null),
    normal: cloneOptionalVec3(options.normal ?? null),
    targetColliderId: String(options.targetColliderId ?? '').trim() || null,
    targetBodyId: String(options.targetBodyId ?? '').trim() || null,
    distance: Number(options.distance ?? 0),
    algorithm: String(options.algorithm ?? '').trim() || null
  };
}

function cloneCcdEvent(event) {
  return createCcdEvent(event);
}

function cloneCcdEvents(events) {
  return Array.isArray(events) ? events.map((event) => cloneCcdEvent(event)) : [];
}

function createEmptySolverStats(iterations = 0) {
  return {
    iterations,
    manifoldCount: 0,
    jointCount: 0,
    warmStartedContactCount: 0,
    warmStartedJointCount: 0,
    solvedContactCount: 0,
    solvedTangentContactCount: 0,
    solvedJointCount: 0,
    restitutionContactCount: 0,
    skippedContactCount: 0,
    skippedJointCount: 0,
    impulsesApplied: 0,
    frictionImpulsesApplied: 0,
    jointImpulsesApplied: 0,
    positionCorrections: 0,
    maxPenetration: 0,
    maxJointError: 0
  };
}

function createEmptyIslandState() {
  return {
    islands: [],
    islandCount: 0,
    bodyCount: 0,
    sleepingBodyCount: 0,
    awakeBodyCount: 0
  };
}

function cloneIslandState(islandState) {
  return cloneRigidBodyIslandGraph(islandState ?? createEmptyIslandState());
}

function cloneSolverStats(solverStats) {
  return {
    iterations: Number(solverStats.iterations ?? 0),
    manifoldCount: Number(solverStats.manifoldCount ?? 0),
    jointCount: Number(solverStats.jointCount ?? 0),
    warmStartedContactCount: Number(solverStats.warmStartedContactCount ?? 0),
    warmStartedJointCount: Number(solverStats.warmStartedJointCount ?? 0),
    solvedContactCount: Number(solverStats.solvedContactCount ?? 0),
    solvedTangentContactCount: Number(solverStats.solvedTangentContactCount ?? 0),
    solvedJointCount: Number(solverStats.solvedJointCount ?? 0),
    restitutionContactCount: Number(solverStats.restitutionContactCount ?? 0),
    skippedContactCount: Number(solverStats.skippedContactCount ?? 0),
    skippedJointCount: Number(solverStats.skippedJointCount ?? 0),
    impulsesApplied: Number(solverStats.impulsesApplied ?? 0),
    frictionImpulsesApplied: Number(solverStats.frictionImpulsesApplied ?? 0),
    jointImpulsesApplied: Number(solverStats.jointImpulsesApplied ?? 0),
    positionCorrections: Number(solverStats.positionCorrections ?? 0),
    maxPenetration: Number(solverStats.maxPenetration ?? 0),
    maxJointError: Number(solverStats.maxJointError ?? 0)
  };
}

function createEmptyCollisionState(iterations = 0) {
  return {
    broadphaseProxies: [],
    broadphasePairs: [],
    contactPairs: [],
    manifolds: [],
    islands: [],
    joints: [],
    solverStats: createEmptySolverStats(iterations),
    summary: {
      proxyCount: 0,
      pairCount: 0,
      contactCount: 0,
      manifoldCount: 0,
      jointCount: 0,
      islandCount: 0,
      sleepingBodyCount: 0,
      awakeBodyCount: 0,
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
    jointCount: Number(summary.jointCount ?? 0),
    islandCount: summary.islandCount,
    sleepingBodyCount: summary.sleepingBodyCount,
    awakeBodyCount: summary.awakeBodyCount,
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
    islands: collisionState.islands.map((island) => cloneRigidBodyIsland(island)),
    joints: Array.isArray(collisionState.joints)
      ? collisionState.joints.map((joint) => ({
        ...joint,
        localAnchorA: cloneVec3(joint.localAnchorA),
        localAnchorB: cloneVec3(joint.localAnchorB),
        localAxisA: cloneVec3(joint.localAxisA ?? createVec3(0, 1, 0)),
        localAxisB: cloneVec3(joint.localAxisB ?? createVec3(0, 1, 0)),
        localReferenceA: cloneVec3(joint.localReferenceA ?? createVec3(1, 0, 0)),
        localReferenceB: cloneVec3(joint.localReferenceB ?? createVec3(1, 0, 0)),
        breakForce: joint.breakForce ?? null,
        breakTorque: joint.breakTorque ?? null,
        broken: joint.broken === true,
        lastAppliedForce: Number(joint.lastAppliedForce ?? 0),
        lastAppliedTorque: Number(joint.lastAppliedTorque ?? 0),
        motorMode: joint.motorMode ?? 'speed',
        motorTargetAngle: Number(joint.motorTargetAngle ?? 0),
        motorServoGain: Number(joint.motorServoGain ?? 8),
        accumulatedLinearImpulse: cloneVec3(joint.accumulatedLinearImpulse ?? createVec3()),
        accumulatedAngularImpulse: cloneVec3(joint.accumulatedAngularImpulse ?? createVec3()),
        accumulatedMotorImpulse: Number(joint.accumulatedMotorImpulse ?? 0)
      }))
      : [],
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
  target.iterations = Math.max(Number(target.iterations ?? 0), Number(source.iterations ?? 0));
  target.manifoldCount = Math.max(Number(target.manifoldCount ?? 0), Number(source.manifoldCount ?? 0));
  target.jointCount = Math.max(Number(target.jointCount ?? 0), Number(source.jointCount ?? 0));
  target.warmStartedContactCount = Number(target.warmStartedContactCount ?? 0) + Number(source.warmStartedContactCount ?? 0);
  target.warmStartedJointCount = Number(target.warmStartedJointCount ?? 0) + Number(source.warmStartedJointCount ?? 0);
  target.solvedContactCount = Number(target.solvedContactCount ?? 0) + Number(source.solvedContactCount ?? 0);
  target.solvedTangentContactCount = Number(target.solvedTangentContactCount ?? 0) + Number(source.solvedTangentContactCount ?? 0);
  target.solvedJointCount = Number(target.solvedJointCount ?? 0) + Number(source.solvedJointCount ?? 0);
  target.restitutionContactCount = Number(target.restitutionContactCount ?? 0) + Number(source.restitutionContactCount ?? 0);
  target.skippedContactCount = Number(target.skippedContactCount ?? 0) + Number(source.skippedContactCount ?? 0);
  target.skippedJointCount = Number(target.skippedJointCount ?? 0) + Number(source.skippedJointCount ?? 0);
  target.impulsesApplied = Number(target.impulsesApplied ?? 0) + Number(source.impulsesApplied ?? 0);
  target.frictionImpulsesApplied = Number(target.frictionImpulsesApplied ?? 0) + Number(source.frictionImpulsesApplied ?? 0);
  target.jointImpulsesApplied = Number(target.jointImpulsesApplied ?? 0) + Number(source.jointImpulsesApplied ?? 0);
  target.positionCorrections = Number(target.positionCorrections ?? 0) + Number(source.positionCorrections ?? 0);
  target.maxPenetration = Math.max(Number(target.maxPenetration ?? 0), Number(source.maxPenetration ?? 0));
  target.maxJointError = Math.max(Number(target.maxJointError ?? 0), Number(source.maxJointError ?? 0));
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

function buildConvexHullDebugLines(shape, pose, color, source, category) {
  const vertices = Array.isArray(shape?.geometry?.vertices) ? shape.geometry.vertices : [];
  const debugEdges = Array.isArray(shape?.geometry?.debugEdges) ? shape.geometry.debugEdges : [];
  if (vertices.length === 0 || debugEdges.length === 0) {
    return [];
  }

  return debugEdges.map((edge, index) => createDebugLine({
    id: `${source.colliderId}:wire-edge:${index}`,
    category,
    start: transformPointByPose(pose, vertices[edge.startIndex]),
    end: transformPointByPose(pose, vertices[edge.endIndex]),
    color,
    source
  }));
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

function computeBoundingSphereRadius(shape) {
  if (!shape) {
    return 0;
  }

  if (shape.type === 'sphere') {
    return Math.abs(Number(shape.geometry.radius ?? 0));
  }

  if (shape.type === 'capsule') {
    return Math.abs(Number(shape.geometry.radius ?? 0)) + Math.abs(Number(shape.geometry.halfHeight ?? 0));
  }

  const localAabb = computeLocalShapeAabb(shape);
  const halfExtents = localAabb?.halfExtents ?? createVec3();
  return Math.sqrt(
    halfExtents.x * halfExtents.x +
    halfExtents.y * halfExtents.y +
    halfExtents.z * halfExtents.z
  );
}

function buildCastSampleOrigins(castType, origin, halfHeight, rotation) {
  const resolvedOrigin = cloneVec3(origin);
  if (castType !== 'capsule' || Math.abs(Number(halfHeight ?? 0)) <= 1e-8) {
    return [resolvedOrigin];
  }

  const axisOffset = rotateVec3ByQuat(rotation ?? createIdentityQuat(), createVec3(0, halfHeight, 0));
  return [
    resolvedOrigin,
    addVec3(resolvedOrigin, axisOffset),
    addScaledVec3(resolvedOrigin, axisOffset, -1)
  ];
}

function clearVector(vector) {
  vector.x = 0;
  vector.y = 0;
  vector.z = 0;
}

function zeroBodyMotion(body) {
  clearVector(body.linearVelocity);
  clearVector(body.angularVelocity);
  clearVector(body.forceAccumulator);
  clearVector(body.torqueAccumulator);
}

function resetJointSolverState(joint) {
  joint.accumulatedImpulse = 0;
  joint.accumulatedLinearImpulse = createVec3();
  joint.accumulatedAngularImpulse = createVec3();
  joint.accumulatedMotorImpulse = 0;
  joint.lastAppliedForce = 0;
  joint.lastAppliedTorque = 0;
}

export class PhysicsWorld {
  constructor(options = {}) {
    this.fixedDeltaTime = toPositiveNumber(options.fixedDeltaTime, 1 / 60);
    this.maxSubsteps = Math.max(1, Math.floor(toPositiveNumber(options.maxSubsteps, 4)));
    this.gravity = cloneVec3(options.gravity ?? createVec3(0, -9.81, 0));
    this.ccdEnabled = options.ccdEnabled !== false;
    this.ccdMotionThreshold = toNonNegativeNumber(options.ccdMotionThreshold, 2);
    this.ccdSafetyMargin = toNonNegativeNumber(options.ccdSafetyMargin, 0.001);
    this.solverIterations = Math.max(1, Math.floor(toPositiveNumber(options.solverIterations, 8)));
    this.solverBaumgarte = toNonNegativeNumber(options.solverBaumgarte, 0.2);
    this.allowedPenetration = toNonNegativeNumber(options.allowedPenetration, 0.01);
    this.positionCorrectionPercent = toNonNegativeNumber(options.positionCorrectionPercent, 0.8);
    this.sleepEnabled = options.sleepEnabled !== false;
    this.sleepLinearThreshold = toNonNegativeNumber(options.sleepLinearThreshold, 0.05);
    this.sleepAngularThreshold = toNonNegativeNumber(options.sleepAngularThreshold, 0.1);
    this.sleepTimeThreshold = toNonNegativeNumber(options.sleepTimeThreshold, 0.5);
    this.shapeRegistry = new ShapeRegistry();
    this.bodyRegistry = new BodyRegistry();
    this.colliderRegistry = new ColliderRegistry();
    this.jointRegistry = new JointRegistry();
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
      position: createVec3(0, 0, 400),
      target: createVec3(0, 0, 0)
    };
    this.accumulatorSeconds = 0;
    this.simulationTick = 0;
    this.renderFrameCount = 0;
    this.lastStepStats = {
      requestedDeltaSeconds: 0,
      performedSubsteps: 0,
      simulationTick: 0,
      remainingAccumulatorSeconds: 0,
      ccdEventCount: 0
    };
    this.lastSolverStats = createEmptySolverStats(this.solverIterations);
    this.lastRaycast = null;
    this.lastShapeCast = null;
    this.lastCcdEvents = [];
    this.lastIslandState = createEmptyIslandState();
    this.collisionStateDirty = true;
    this.collisionState = createEmptyCollisionState(this.solverIterations);
    this.manifoldCache.clear();
  }

  reset() {
    this.shapeRegistry.clear();
    this.bodyRegistry.clear();
    this.colliderRegistry.clear();
    this.jointRegistry.clear();
    this.materialRegistry.clear();
    this.bootstrapDefaultMaterials();
    this.resetRuntimeState();
  }

  setDebugCameraPosition(position) {
    this.debugCamera.position = cloneVec3(position);
  }

  setDebugCameraTarget(target) {
    this.debugCamera.target = cloneVec3(target);
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

  resolveBodyLocalAnchor(bodyId, options = {}) {
    const body = bodyId ? this.bodyRegistry.get(bodyId) : null;
    if (!body) {
      return cloneVec3(options.localAnchor ?? createVec3());
    }

    if (options.worldAnchor) {
      const worldOffset = subtractVec3(options.worldAnchor, body.position);
      return inverseRotateVec3ByQuat(body.rotation ?? createIdentityQuat(), worldOffset);
    }

    return cloneVec3(options.localAnchor ?? createVec3());
  }

  resolveBodyLocalAxis(bodyId, options = {}) {
    const body = bodyId ? this.bodyRegistry.get(bodyId) : null;
    if (!body) {
      return normalizeVec3(options.localAxis ?? createVec3(0, 1, 0), createVec3(0, 1, 0));
    }

    if (options.worldAxis) {
      return normalizeVec3(
        inverseRotateVec3ByQuat(body.rotation ?? createIdentityQuat(), normalizeVec3(options.worldAxis, createVec3(0, 1, 0))),
        createVec3(0, 1, 0)
      );
    }

    return normalizeVec3(options.localAxis ?? createVec3(0, 1, 0), createVec3(0, 1, 0));
  }

  resolveJointBodies(options = {}) {
    const bodyAId = String(options.bodyAId ?? '').trim() || null;
    const bodyBId = String(options.bodyBId ?? '').trim() || null;
    if (!bodyAId || !bodyBId || bodyAId === bodyBId) {
      return null;
    }

    const bodyA = this.getBody(bodyAId);
    const bodyB = this.getBody(bodyBId);
    if (!bodyA || !bodyB) {
      return null;
    }

    return { bodyAId, bodyBId, bodyA, bodyB };
  }

  createDistanceJoint(options = {}) {
    const resolvedBodies = this.resolveJointBodies(options);
    if (!resolvedBodies) {
      return null;
    }

    const { bodyAId, bodyBId, bodyA, bodyB } = resolvedBodies;

    const localAnchorA = this.resolveBodyLocalAnchor(bodyAId, {
      localAnchor: options.localAnchorA,
      worldAnchor: options.worldAnchorA
    });
    const localAnchorB = this.resolveBodyLocalAnchor(bodyBId, {
      localAnchor: options.localAnchorB,
      worldAnchor: options.worldAnchorB
    });
    const poseA = {
      position: bodyA.position,
      rotation: bodyA.rotation ?? createIdentityQuat()
    };
    const poseB = {
      position: bodyB.position,
      rotation: bodyB.rotation ?? createIdentityQuat()
    };
    const worldAnchorA = addVec3(poseA.position, rotateVec3ByQuat(poseA.rotation, localAnchorA));
    const worldAnchorB = addVec3(poseB.position, rotateVec3ByQuat(poseB.rotation, localAnchorB));
    const limits = normalizeDistanceLimits(options.minDistance, options.maxDistance);
    const distance = clampDistanceToRange(
      options.distance ?? Math.sqrt(lengthSquaredVec3(subtractVec3(worldAnchorB, worldAnchorA))),
      limits.minDistance,
      limits.maxDistance
    );

    const joint = this.jointRegistry.createDistanceJoint({
      ...options,
      bodyAId,
      bodyBId,
      localAnchorA,
      localAnchorB,
      distance,
      minDistance: limits.minDistance,
      maxDistance: limits.maxDistance
    });

    this.wakeBodies([bodyAId, bodyBId]);
    this.markCollisionStateDirty();
    return joint;
  }

  createPointToPointJoint(options = {}) {
    const resolvedBodies = this.resolveJointBodies(options);
    if (!resolvedBodies) {
      return null;
    }

    const { bodyAId, bodyBId, bodyA, bodyB } = resolvedBodies;
    const sharedWorldAnchor = cloneVec3(options.worldAnchor ?? scaleVec3(addVec3(bodyA.position, bodyB.position), 0.5));
    const localAnchorA = this.resolveBodyLocalAnchor(bodyAId, {
      localAnchor: options.localAnchorA,
      worldAnchor: options.worldAnchorA ?? sharedWorldAnchor
    });
    const localAnchorB = this.resolveBodyLocalAnchor(bodyBId, {
      localAnchor: options.localAnchorB,
      worldAnchor: options.worldAnchorB ?? sharedWorldAnchor
    });

    const joint = this.jointRegistry.createPointToPointJoint({
      ...options,
      bodyAId,
      bodyBId,
      localAnchorA,
      localAnchorB
    });

    this.wakeBodies([bodyAId, bodyBId]);
    this.markCollisionStateDirty();
    return joint;
  }

  createHingeJoint(options = {}) {
    const resolvedBodies = this.resolveJointBodies(options);
    if (!resolvedBodies) {
      return null;
    }

    const { bodyAId, bodyBId, bodyA, bodyB } = resolvedBodies;
    const sharedWorldAnchor = cloneVec3(options.worldAnchor ?? scaleVec3(addVec3(bodyA.position, bodyB.position), 0.5));
    const sharedWorldAxis = normalizeVec3(options.worldAxis ?? createVec3(0, 1, 0), createVec3(0, 1, 0));
    const localAnchorA = this.resolveBodyLocalAnchor(bodyAId, {
      localAnchor: options.localAnchorA,
      worldAnchor: options.worldAnchorA ?? sharedWorldAnchor
    });
    const localAnchorB = this.resolveBodyLocalAnchor(bodyBId, {
      localAnchor: options.localAnchorB,
      worldAnchor: options.worldAnchorB ?? sharedWorldAnchor
    });
    const localAxisA = this.resolveBodyLocalAxis(bodyAId, {
      localAxis: options.localAxisA,
      worldAxis: options.worldAxisA ?? sharedWorldAxis
    });
    const localAxisB = this.resolveBodyLocalAxis(bodyBId, {
      localAxis: options.localAxisB,
      worldAxis: options.worldAxisB ?? sharedWorldAxis
    });
    const tangentBasis = createTangentBasis(sharedWorldAxis);
    const localReferenceA = this.resolveBodyLocalAxis(bodyAId, {
      localAxis: options.localReferenceA,
      worldAxis: options.worldReferenceA ?? tangentBasis.tangentA
    });
    const localReferenceB = this.resolveBodyLocalAxis(bodyBId, {
      localAxis: options.localReferenceB,
      worldAxis: options.worldReferenceB ?? tangentBasis.tangentA
    });

    const joint = this.jointRegistry.createHingeJoint({
      ...options,
      bodyAId,
      bodyBId,
      localAnchorA,
      localAnchorB,
      localAxisA,
      localAxisB,
      localReferenceA,
      localReferenceB
    });

    this.wakeBodies([bodyAId, bodyBId]);
    this.markCollisionStateDirty();
    return joint;
  }

  createFixedJoint(options = {}) {
    const resolvedBodies = this.resolveJointBodies(options);
    if (!resolvedBodies) {
      return null;
    }

    const { bodyAId, bodyBId, bodyA, bodyB } = resolvedBodies;
    const sharedWorldAnchor = cloneVec3(options.worldAnchor ?? scaleVec3(addVec3(bodyA.position, bodyB.position), 0.5));
    const sharedWorldAxis = normalizeVec3(
      options.worldAxis ?? rotateVec3ByQuat(bodyA.rotation ?? createIdentityQuat(), createVec3(0, 1, 0)),
      createVec3(0, 1, 0)
    );
    const sharedWorldReference = normalizeVec3(
      options.worldReference ?? rotateVec3ByQuat(bodyA.rotation ?? createIdentityQuat(), createVec3(1, 0, 0)),
      createVec3(1, 0, 0)
    );
    const localAnchorA = this.resolveBodyLocalAnchor(bodyAId, {
      localAnchor: options.localAnchorA,
      worldAnchor: options.worldAnchorA ?? sharedWorldAnchor
    });
    const localAnchorB = this.resolveBodyLocalAnchor(bodyBId, {
      localAnchor: options.localAnchorB,
      worldAnchor: options.worldAnchorB ?? sharedWorldAnchor
    });
    const localAxisA = this.resolveBodyLocalAxis(bodyAId, {
      localAxis: options.localAxisA,
      worldAxis: options.worldAxisA ?? sharedWorldAxis
    });
    const localAxisB = this.resolveBodyLocalAxis(bodyBId, {
      localAxis: options.localAxisB,
      worldAxis: options.worldAxisB ?? sharedWorldAxis
    });
    const localReferenceA = this.resolveBodyLocalAxis(bodyAId, {
      localAxis: options.localReferenceA,
      worldAxis: options.worldReferenceA ?? sharedWorldReference
    });
    const localReferenceB = this.resolveBodyLocalAxis(bodyBId, {
      localAxis: options.localReferenceB,
      worldAxis: options.worldReferenceB ?? sharedWorldReference
    });

    const joint = this.jointRegistry.createFixedJoint({
      ...options,
      bodyAId,
      bodyBId,
      localAnchorA,
      localAnchorB,
      localAxisA,
      localAxisB,
      localReferenceA,
      localReferenceB
    });

    this.wakeBodies([bodyAId, bodyBId]);
    this.markCollisionStateDirty();
    return joint;
  }

  configureDistanceJoint(jointId, options = {}) {
    const joint = this.jointRegistry.getMutable(jointId);
    if (!joint || joint.type !== 'distance-joint') {
      return null;
    }

    const limits = normalizeDistanceLimits(
      options.minDistance ?? joint.minDistance,
      options.maxDistance ?? joint.maxDistance
    );
    joint.minDistance = limits.minDistance;
    joint.maxDistance = limits.maxDistance;

    if (options.distance !== undefined) {
      joint.distance = clampDistanceToRange(options.distance, joint.minDistance, joint.maxDistance);
    } else {
      joint.distance = clampDistanceToRange(joint.distance, joint.minDistance, joint.maxDistance);
    }

    if (options.springFrequency !== undefined) {
      joint.springFrequency = toNonNegativeNumber(options.springFrequency, joint.springFrequency ?? 0);
    }

    if (options.dampingRatio !== undefined) {
      joint.dampingRatio = toNonNegativeNumber(options.dampingRatio, joint.dampingRatio ?? 0);
    }

    resetJointSolverState(joint);

    this.wakeBodies([joint.bodyAId, joint.bodyBId]);
    this.markCollisionStateDirty();
    return this.getJoint(joint.id);
  }

  configureHingeJoint(jointId, options = {}) {
    const joint = this.jointRegistry.getMutable(jointId);
    if (!joint || joint.type !== 'hinge-joint') {
      return null;
    }

    const limits = normalizeAngleLimits(
      options.lowerAngle ?? joint.lowerAngle,
      options.upperAngle ?? joint.upperAngle
    );
    joint.lowerAngle = limits.lowerAngle;
    joint.upperAngle = limits.upperAngle;

    if (options.angularDamping !== undefined) {
      joint.angularDamping = toNonNegativeNumber(options.angularDamping, joint.angularDamping ?? 0);
    }

    resetJointSolverState(joint);

    this.wakeBodies([joint.bodyAId, joint.bodyBId]);
    this.markCollisionStateDirty();
    return this.getJoint(joint.id);
  }

  configureFixedJoint(jointId, options = {}) {
    const joint = this.jointRegistry.getMutable(jointId);
    if (!joint || joint.type !== 'fixed-joint') {
      return null;
    }

    if (options.breakForce !== undefined) {
      joint.breakForce = toOptionalNonNegativeNumber(options.breakForce, null);
    }

    if (options.breakTorque !== undefined) {
      joint.breakTorque = toOptionalNonNegativeNumber(options.breakTorque, null);
    }

    if (options.enabled !== undefined) {
      joint.enabled = Boolean(options.enabled);
    }

    if (options.broken === false) {
      joint.broken = false;
    }

    resetJointSolverState(joint);

    this.wakeBodies([joint.bodyAId, joint.bodyBId]);
    this.markCollisionStateDirty();
    return this.getJoint(joint.id);
  }

  configureHingeMotor(jointId, options = {}) {
    const joint = this.jointRegistry.getMutable(jointId);
    if (!joint || joint.type !== 'hinge-joint') {
      return null;
    }

    if (options.motorSpeed !== undefined) {
      joint.motorSpeed = Number.isFinite(Number(options.motorSpeed)) ? Number(options.motorSpeed) : (joint.motorSpeed ?? 0);
    }

    if (options.motorMode !== undefined) {
      joint.motorMode = options.motorMode === 'servo' ? 'servo' : 'speed';
    }

    if (options.motorTargetAngle !== undefined) {
      joint.motorTargetAngle = Number.isFinite(Number(options.motorTargetAngle))
        ? Number(options.motorTargetAngle)
        : (joint.motorTargetAngle ?? 0);
    }

    if (options.motorServoGain !== undefined) {
      joint.motorServoGain = toNonNegativeNumber(options.motorServoGain, joint.motorServoGain ?? 8);
    }

    if (options.maxMotorTorque !== undefined) {
      joint.maxMotorTorque = toNonNegativeNumber(options.maxMotorTorque, joint.maxMotorTorque ?? 0);
    }

    if (options.motorEnabled !== undefined) {
      joint.motorEnabled = Boolean(options.motorEnabled) && Number(joint.maxMotorTorque ?? 0) > 0;
    } else if (options.maxMotorTorque !== undefined) {
      joint.motorEnabled = Number(joint.maxMotorTorque ?? 0) > 0;
    }

    joint.broken = false;
    resetJointSolverState(joint);

    this.wakeBodies([joint.bodyAId, joint.bodyBId]);
    this.markCollisionStateDirty();
    return this.getJoint(joint.id);
  }

  configureHingeServo(jointId, options = {}) {
    const joint = this.jointRegistry.getMutable(jointId);
    if (!joint || joint.type !== 'hinge-joint') {
      return null;
    }

    return this.configureHingeMotor(jointId, {
      motorMode: 'servo',
      motorEnabled: options.motorEnabled ?? true,
      motorTargetAngle: options.motorTargetAngle ?? joint.motorTargetAngle ?? 0,
      motorSpeed: options.maxMotorSpeed ?? options.motorSpeed ?? joint.motorSpeed ?? 0,
      maxMotorTorque: options.maxMotorTorque ?? joint.maxMotorTorque ?? 0,
      motorServoGain: options.motorServoGain ?? joint.motorServoGain ?? 8
    });
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
      canSleep: options.canSleep,
      sleeping: options.sleeping,
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
      canSleep: options.canSleep,
      sleeping: options.sleeping,
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
      canSleep: options.canSleep,
      sleeping: options.sleeping,
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
      canSleep: options.canSleep,
      sleeping: options.sleeping,
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

  createStaticConvexHullCollider(options = {}) {
    const resolvedId = String(options.id ?? '').trim() || null;
    const shape = this.createConvexHullShape({
      id: resolvedId ? `${resolvedId}:shape` : null,
      vertices: options.vertices ?? [],
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

  getJoint(id) {
    return this.jointRegistry.get(id);
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

  getBodyJoints(bodyId) {
    return this.jointRegistry.list()
      .filter((joint) => joint.bodyAId === bodyId || joint.bodyBId === bodyId);
  }

  getJointWorldAnchors(jointId) {
    const joint = this.getJoint(jointId);
    if (!joint) {
      return null;
    }

    const bodyA = joint.bodyAId ? this.getBody(joint.bodyAId) : null;
    const bodyB = joint.bodyBId ? this.getBody(joint.bodyBId) : null;
    const anchorA = bodyA
      ? addVec3(bodyA.position, rotateVec3ByQuat(bodyA.rotation ?? createIdentityQuat(), joint.localAnchorA))
      : cloneVec3(joint.localAnchorA);
    const anchorB = bodyB
      ? addVec3(bodyB.position, rotateVec3ByQuat(bodyB.rotation ?? createIdentityQuat(), joint.localAnchorB))
      : cloneVec3(joint.localAnchorB);

    return {
      joint,
      anchorA,
      anchorB
    };
  }

  getJointWorldAxes(jointId) {
    const joint = this.getJoint(jointId);
    if (!joint) {
      return null;
    }

    const bodyA = joint.bodyAId ? this.getBody(joint.bodyAId) : null;
    const bodyB = joint.bodyBId ? this.getBody(joint.bodyBId) : null;
    const axisA = normalizeVec3(
      rotateVec3ByQuat(bodyA?.rotation ?? createIdentityQuat(), joint.localAxisA ?? createVec3(0, 1, 0)),
      createVec3(0, 1, 0)
    );
    const axisB = normalizeVec3(
      rotateVec3ByQuat(bodyB?.rotation ?? createIdentityQuat(), joint.localAxisB ?? createVec3(0, 1, 0)),
      createVec3(0, 1, 0)
    );

    return {
      joint,
      axisA,
      axisB
    };
  }

  getJointWorldReferences(jointId) {
    const joint = this.getJoint(jointId);
    if (!joint) {
      return null;
    }

    const bodyA = joint.bodyAId ? this.getBody(joint.bodyAId) : null;
    const bodyB = joint.bodyBId ? this.getBody(joint.bodyBId) : null;
    const referenceA = normalizeVec3(
      rotateVec3ByQuat(bodyA?.rotation ?? createIdentityQuat(), joint.localReferenceA ?? createVec3(1, 0, 0)),
      createVec3(1, 0, 0)
    );
    const referenceB = normalizeVec3(
      rotateVec3ByQuat(bodyB?.rotation ?? createIdentityQuat(), joint.localReferenceB ?? createVec3(1, 0, 0)),
      createVec3(1, 0, 0)
    );

    return {
      joint,
      referenceA,
      referenceB
    };
  }

  getJointAngle(jointId) {
    const joint = this.getJoint(jointId);
    if (!joint || (joint.type !== 'hinge-joint' && joint.type !== 'fixed-joint')) {
      return null;
    }

    const bodyA = joint.bodyAId ? this.getBody(joint.bodyAId) : null;
    const bodyB = joint.bodyBId ? this.getBody(joint.bodyBId) : null;
    const axes = this.getJointWorldAxes(jointId);
    const hingeAxis = normalizeVec3(addVec3(axes?.axisA ?? createVec3(0, 1, 0), axes?.axisB ?? createVec3(0, 1, 0)), axes?.axisA ?? createVec3(0, 1, 0));
    const fallbackReference = createTangentBasis(hingeAxis).tangentA;
    const referenceA = normalizeVec3(
      rotateVec3ByQuat(bodyA?.rotation ?? createIdentityQuat(), joint.localReferenceA ?? fallbackReference),
      fallbackReference
    );
    const referenceB = normalizeVec3(
      rotateVec3ByQuat(bodyB?.rotation ?? createIdentityQuat(), joint.localReferenceB ?? fallbackReference),
      fallbackReference
    );
    return Math.atan2(
      dotVec3(crossVec3(referenceA, referenceB), hingeAxis),
      dotVec3(referenceA, referenceB)
    );
  }

  applyJointBreakThresholds(jointResults, deltaTime) {
    const brokenJointIds = [];
    if (!Array.isArray(jointResults) || jointResults.length === 0) {
      return brokenJointIds;
    }

    for (const result of jointResults) {
      const joint = this.jointRegistry.getMutable(result.jointId);
      if (!joint) {
        continue;
      }

      const appliedForce = deltaTime > 1e-8
        ? Number(result.linearImpulse ?? 0) / deltaTime
        : 0;
      const appliedTorque = deltaTime > 1e-8
        ? (Number(result.angularImpulse ?? 0) + Number(result.motorImpulse ?? 0)) / deltaTime
        : 0;

      joint.lastAppliedForce = appliedForce;
      joint.lastAppliedTorque = appliedTorque;

      const breakForce = toOptionalNonNegativeNumber(joint.breakForce, null);
      const breakTorque = toOptionalNonNegativeNumber(joint.breakTorque, null);
      const shouldBreak = joint.enabled !== false && (
        (breakForce !== null && appliedForce >= breakForce - 1e-8) ||
        (breakTorque !== null && appliedTorque >= breakTorque - 1e-8)
      );

      if (!shouldBreak) {
        continue;
      }

      joint.enabled = false;
      joint.broken = true;
      resetJointSolverState(joint);
      joint.lastAppliedForce = appliedForce;
      joint.lastAppliedTorque = appliedTorque;
      brokenJointIds.push(joint.id);
    }

    if (brokenJointIds.length > 0) {
      const bodyIds = [];
      for (const jointId of brokenJointIds) {
        const joint = this.jointRegistry.get(jointId);
        if (!joint) {
          continue;
        }

        bodyIds.push(joint.bodyAId, joint.bodyBId);
      }

      this.wakeBodies(bodyIds);
      this.markCollisionStateDirty();
    }

    return brokenJointIds;
  }

  buildIslandState(manifolds) {
    return buildRigidBodyIslandGraph({
      bodyRegistry: this.bodyRegistry,
      manifolds,
      joints: this.jointRegistry.list()
    });
  }

  isBodySleepCandidate(body) {
    return Boolean(body && body.enabled && body.motionType === 'dynamic' && body.canSleep !== false);
  }

  isBodyBelowSleepThresholds(body, thresholdScale = 1) {
    const resolvedScale = Math.max(1, Number(thresholdScale ?? 1));
    const linearThresholdSquared = (this.sleepLinearThreshold * resolvedScale) * (this.sleepLinearThreshold * resolvedScale);
    const angularThresholdSquared = (this.sleepAngularThreshold * resolvedScale) * (this.sleepAngularThreshold * resolvedScale);
    return lengthSquaredVec3(body.linearVelocity) <= linearThresholdSquared &&
      lengthSquaredVec3(body.angularVelocity) <= angularThresholdSquared;
  }

  hasPendingWakeActivity(body) {
    return lengthSquaredVec3(body.forceAccumulator) > 1e-12 ||
      lengthSquaredVec3(body.torqueAccumulator) > 1e-12 ||
      !this.isBodyBelowSleepThresholds(body);
  }

  wakeBody(bodyId) {
    const body = this.bodyRegistry.getMutable(bodyId);
    if (!body || body.motionType !== 'dynamic') {
      return false;
    }

    const changed = body.sleeping || body.sleepTimer > 0;
    body.sleeping = false;
    body.sleepTimer = 0;
    if (changed) {
      this.markCollisionStateDirty();
    }

    return changed;
  }

  wakeBodies(bodyIds) {
    let wokeAnyBody = false;
    for (const bodyId of Array.isArray(bodyIds) ? bodyIds : []) {
      wokeAnyBody = this.wakeBody(bodyId) || wokeAnyBody;
    }

    return wokeAnyBody;
  }

  sleepBodies(bodyIds) {
    let sleptAnyBody = false;
    for (const bodyId of Array.isArray(bodyIds) ? bodyIds : []) {
      const body = this.bodyRegistry.getMutable(bodyId);
      if (!this.isBodySleepCandidate(body)) {
        continue;
      }

      if (!body.sleeping || lengthSquaredVec3(body.linearVelocity) > 1e-12 || lengthSquaredVec3(body.angularVelocity) > 1e-12) {
        sleptAnyBody = true;
      }

      body.sleeping = true;
      body.sleepTimer = this.sleepTimeThreshold;
      zeroBodyMotion(body);
    }

    if (sleptAnyBody) {
      this.markCollisionStateDirty();
    }

    return sleptAnyBody;
  }

  wakeBodiesFromExternalActivity() {
    if (!this.sleepEnabled) {
      return false;
    }

    let wokeAnyBody = false;
    this.bodyRegistry.forEachMutable((body) => {
      if (!body.sleeping || !this.isBodySleepCandidate(body)) {
        return;
      }

      if (this.hasPendingWakeActivity(body)) {
        body.sleeping = false;
        body.sleepTimer = 0;
        wokeAnyBody = true;
      }
    });

    if (wokeAnyBody) {
      this.markCollisionStateDirty();
    }

    return wokeAnyBody;
  }

  wakeBodiesFromIslandInteractions(islandState) {
    if (!this.sleepEnabled) {
      return false;
    }

    let wokeAnyBody = false;
    for (const island of islandState?.islands ?? []) {
      if (island.sleepingBodyCount > 0 && island.awakeBodyCount > 0) {
        wokeAnyBody = this.wakeBodies(island.bodyIds) || wokeAnyBody;
      }
    }

    return wokeAnyBody;
  }

  updateSleepingBodies(deltaTime, manifolds) {
    if (!this.sleepEnabled) {
      const awakeIslands = this.buildIslandState(manifolds);
      this.bodyRegistry.forEachMutable((body) => {
        if (!body || body.motionType !== 'dynamic') {
          return;
        }

        body.sleeping = false;
        body.sleepTimer = 0;
      });
      return awakeIslands;
    }

    const islandState = this.buildIslandState(manifolds);

    for (const island of islandState.islands) {
      const mutableBodies = island.bodyIds
        .map((bodyId) => this.bodyRegistry.getMutable(bodyId))
        .filter(Boolean);

      if (!mutableBodies.length) {
        continue;
      }

      const canIslandSleep = island.canSleep && mutableBodies.every((body) => this.isBodySleepCandidate(body));
      const islandThresholdScale = island.constraintCount > 0 ? 2 : 1;
      const isIslandQuiet = canIslandSleep && mutableBodies.every((body) => (
        this.isBodyBelowSleepThresholds(body, islandThresholdScale) &&
        lengthSquaredVec3(body.forceAccumulator) <= 1e-12 &&
        lengthSquaredVec3(body.torqueAccumulator) <= 1e-12
      ));

      if (!isIslandQuiet) {
        for (const body of mutableBodies) {
          body.sleepTimer = 0;
          body.sleeping = false;
        }
        continue;
      }

      for (const body of mutableBodies) {
        body.sleepTimer += deltaTime;
      }

      const islandSleepTimer = Math.min(...mutableBodies.map((body) => body.sleepTimer));
      if (islandSleepTimer + 1e-12 >= this.sleepTimeThreshold) {
        this.sleepBodies(island.bodyIds);
      }
    }

    return this.buildIslandState(manifolds);
  }

  getWorldSummary() {
    const collisionState = this.getCollisionState();

    return {
      bodyCount: this.bodyRegistry.count(),
      shapeCount: this.shapeRegistry.count(),
      colliderCount: this.colliderRegistry.count(),
      jointCount: this.jointRegistry.count(),
      materialCount: this.materialRegistry.count(),
      broadphaseProxyCount: collisionState.summary.proxyCount,
      broadphasePairCount: collisionState.summary.pairCount,
      contactPairCount: collisionState.summary.contactCount,
      manifoldCount: collisionState.summary.manifoldCount,
      jointConstraintCount: collisionState.summary.jointCount,
      islandCount: collisionState.summary.islandCount,
      sleepingBodyCount: collisionState.summary.sleepingBodyCount,
      awakeBodyCount: collisionState.summary.awakeBodyCount,
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
    let ccdEventCount = 0;

    while (this.accumulatorSeconds + 1e-12 >= this.fixedDeltaTime && performedSubsteps < this.maxSubsteps) {
      this.wakeBodiesFromExternalActivity();
      const ccdEvents = this.integrateRigidBodies(this.fixedDeltaTime);
      ccdEventCount += ccdEvents.length;
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
      remainingAccumulatorSeconds: this.accumulatorSeconds,
      ccdEventCount
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

  buildContinuousCastSpec(body) {
    const primaryCollider = body.primaryColliderId ? this.getCollider(body.primaryColliderId) : null;
    const shape = this.getShape(primaryCollider?.shapeId ?? body.shapeId);
    if (!shape) {
      return null;
    }

    const localPose = primaryCollider?.localPose ?? {
      position: createVec3(),
      rotation: createIdentityQuat()
    };
    const colliderStartPose = composePoses({
      position: body.position,
      rotation: body.rotation ?? createIdentityQuat()
    }, localPose);

    return {
      castType: shape.type,
      queryShape: shape,
      shape,
      radius: shape.type === 'sphere' || shape.type === 'capsule' ? Math.abs(Number(shape.geometry.radius ?? 0)) : computeBoundingSphereRadius(shape),
      halfHeight: shape.type === 'capsule' ? Math.abs(Number(shape.geometry.halfHeight ?? 0)) : 0,
      origin: colliderStartPose.position,
      rotation: colliderStartPose.rotation,
      bodyOffset: rotateVec3ByQuat(body.rotation ?? createIdentityQuat(), localPose.position ?? createVec3())
    };
  }

  runBodyContinuousCollision(body, startPosition, deltaTime, broadphaseSnapshot) {
    if (!this.ccdEnabled) {
      return null;
    }

    const motionDistanceSquared = lengthSquaredVec3(body.linearVelocity) * deltaTime * deltaTime;
    if (motionDistanceSquared <= this.ccdMotionThreshold * this.ccdMotionThreshold) {
      return null;
    }

    const castSpec = this.buildContinuousCastSpec(body);
    if (!castSpec || castSpec.radius <= 0) {
      return null;
    }

    const motionDistance = Math.sqrt(motionDistanceSquared);
    const result = this.shapeCastAgainstWorld({
      castType: castSpec.castType,
      queryShape: castSpec.queryShape,
      origin: castSpec.origin,
      direction: body.linearVelocity,
      maxDistance: motionDistance,
      radius: castSpec.radius,
      halfHeight: castSpec.halfHeight,
      rotation: castSpec.rotation,
      broadphaseProxies: broadphaseSnapshot,
      excludeBodyId: body.id,
      excludeColliderIds: body.colliderIds,
      ignoreDynamicTargets: true,
      storeResult: false
    });

    if (!result.hit || !result.normal) {
      return null;
    }

    const travelDirection = normalizeVec3(body.linearVelocity, createVec3(1, 0, 0));
    const safeDistance = Math.max(0, result.distance - this.ccdSafetyMargin);
    const colliderEndPosition = addScaledVec3(castSpec.origin, travelDirection, safeDistance);
    const bodyEndPosition = subtractVec3(colliderEndPosition, castSpec.bodyOffset);
    const inwardSpeed = dotVec3(body.linearVelocity, result.normal);
    if (inwardSpeed < 0) {
      body.linearVelocity = addScaledVec3(body.linearVelocity, result.normal, -inwardSpeed);
    }

    return createCcdEvent({
      bodyId: body.id,
      colliderId: body.primaryColliderId ?? null,
      castType: castSpec.castType,
      startPosition,
      endPosition: bodyEndPosition,
      hitPosition: result.point,
      normal: result.normal,
      targetColliderId: result.colliderId,
      targetBodyId: result.bodyId,
      distance: result.distance,
      algorithm: result.algorithm
    });
  }

  integrateRigidBodies(deltaTime) {
    let movedAnyDynamicBody = false;
    const ccdEvents = [];
    const broadphaseSnapshot = this.ccdEnabled ? this.buildBroadphaseProxies() : [];

    this.bodyRegistry.forEachMutable((body) => {
      if (!body.enabled || body.sleeping || body.motionType !== 'dynamic') {
        return;
      }

      const startPosition = cloneVec3(body.position);
      const linearAcceleration = addScaledVec3(this.gravity, body.forceAccumulator, body.inverseMass);
      const angularAcceleration = this.applyInverseInertia(body, body.torqueAccumulator);
      body.linearVelocity = addScaledVec3(body.linearVelocity, linearAcceleration, deltaTime);
      body.angularVelocity = addScaledVec3(body.angularVelocity, angularAcceleration, deltaTime);
      const ccdEvent = this.runBodyContinuousCollision(body, startPosition, deltaTime, broadphaseSnapshot);
      if (ccdEvent) {
        body.position = cloneVec3(ccdEvent.endPosition);
        ccdEvents.push(ccdEvent);
      } else {
        body.position = addScaledVec3(body.position, body.linearVelocity, deltaTime);
      }
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

    this.lastCcdEvents = cloneCcdEvents(ccdEvents);
    return cloneCcdEvents(ccdEvents);
  }

  solveRigidContacts(deltaTime, simulationTick) {
    const initialResults = this.buildCollisionResults();
    const manifolds = this.manifoldCache.syncFromContactPairs(initialResults.contactPairs, simulationTick);
    const initialIslandState = this.buildIslandState(manifolds);
    this.wakeBodiesFromIslandInteractions(initialIslandState);
    const solverStats = solveNormalContactConstraints({
      bodyRegistry: this.bodyRegistry,
      manifolds,
      deltaTime,
      iterations: this.solverIterations,
      baumgarte: this.solverBaumgarte,
      allowedPenetration: this.allowedPenetration,
      positionCorrectionPercent: this.positionCorrectionPercent
    });
    const jointSolverStats = solveJointConstraints({
      bodyRegistry: this.bodyRegistry,
      joints: this.jointRegistry.list().filter((joint) => joint.enabled !== false),
      deltaTime,
      iterations: this.solverIterations,
      baumgarte: this.solverBaumgarte,
      allowedStretch: this.allowedPenetration
    });
    this.applyJointBreakThresholds(jointSolverStats.jointResults, deltaTime);
    mergeSolverStats(solverStats, jointSolverStats);

    this.lastSolverStats = solverStats;
    const finalResults = this.buildCollisionResults();
    const finalManifolds = this.manifoldCache.syncFromContactPairs(finalResults.contactPairs, simulationTick);
    const finalIslandState = this.updateSleepingBodies(deltaTime, finalManifolds);
    this.lastIslandState = cloneIslandState(finalIslandState);
    this.commitCollisionState(finalResults, finalManifolds, solverStats, finalIslandState);
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

  commitCollisionState(results, manifolds, solverStats, islandState = this.lastIslandState) {
    const resolvedIslandState = cloneIslandState(islandState);
    const joints = this.jointRegistry.list();
    this.collisionState = {
      broadphaseProxies: results.broadphaseProxies,
      broadphasePairs: results.broadphasePairs,
      contactPairs: results.contactPairs,
      manifolds: Array.isArray(manifolds) ? manifolds.map((manifold) => cloneManifold(manifold)) : [],
      islands: resolvedIslandState.islands,
      joints,
      solverStats: cloneSolverStats(solverStats ?? this.lastSolverStats),
      summary: {
        proxyCount: results.broadphaseProxies.length,
        pairCount: results.broadphasePairs.length,
        contactCount: results.contactPairs.length,
        manifoldCount: Array.isArray(manifolds) ? manifolds.length : 0,
        jointCount: joints.length,
        islandCount: resolvedIslandState.islandCount,
        sleepingBodyCount: resolvedIslandState.sleepingBodyCount,
        awakeBodyCount: resolvedIslandState.awakeBodyCount,
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
      const islandState = this.buildIslandState(manifolds);
      this.lastIslandState = cloneIslandState(islandState);
      this.commitCollisionState(results, manifolds, this.lastSolverStats, islandState);
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

  shapeCastAgainstWorld(options = {}) {
    const queryShape = options.queryShape ?? null;
    const normalizedCastType = String(options.castType ?? 'sphere').trim().toLowerCase();
    const castType = normalizedCastType === 'capsule'
      ? 'capsule'
      : normalizedCastType === 'sphere'
        ? 'sphere'
        : normalizedCastType === 'box' || normalizedCastType === 'convex-hull'
          ? normalizedCastType
          : queryShape?.type ?? 'shape';
    const origin = cloneVec3(options.origin ?? createVec3());
    const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
    const maxDistance = toNonNegativeNumber(options.maxDistance, 100);
    const radius = toNonNegativeNumber(options.radius, 0.5);
    const halfHeight = castType === 'capsule' ? toNonNegativeNumber(options.halfHeight, 0.5) : 0;
    const rotation = cloneQuat(options.rotation ?? createIdentityQuat());
    const broadphaseProxies = Array.isArray(options.broadphaseProxies)
      ? options.broadphaseProxies
      : this.ensureCollisionState().broadphaseProxies;
    const excludeBodyId = String(options.excludeBodyId ?? '').trim() || null;
    const excludeColliderIds = new Set(Array.isArray(options.excludeColliderIds) ? options.excludeColliderIds.map((id) => String(id ?? '').trim()) : []);
    const ignoreDynamicTargets = options.ignoreDynamicTargets === true;
    const ignoreSensors = options.ignoreSensors !== false;
    const resolvedQueryShape = queryShape ?? (castType === 'capsule'
      ? {
        type: 'capsule',
        geometry: {
          radius,
          halfHeight
        },
        localPose: {
          position: createVec3(),
          rotation: createIdentityQuat()
        }
      }
      : {
        type: 'sphere',
        geometry: {
          radius
        },
        localPose: {
          position: createVec3(),
          rotation: createIdentityQuat()
        }
      });
    const sweptAabb = computeSweptShapeAabb({
      shape: resolvedQueryShape,
      origin,
      direction,
      maxDistance,
      rotation
    });
    const sampleOrigins = buildCastSampleOrigins(castType, origin, halfHeight, rotation);
    const sampleEndPoints = sampleOrigins.map((sampleOrigin) => addScaledVec3(sampleOrigin, direction, maxDistance));
    let candidateCount = 0;
    let testedShapeCount = 0;
    let nearestHit = null;

    for (const proxy of broadphaseProxies) {
      if (excludeColliderIds.has(proxy.colliderId)) {
        continue;
      }

      if (excludeBodyId && proxy.bodyId === excludeBodyId) {
        continue;
      }

      if (ignoreDynamicTargets && proxy.motionType === 'dynamic') {
        continue;
      }

      if (ignoreSensors && proxy.isSensor) {
        continue;
      }

      if (sweptAabb && !testAabbOverlap(sweptAabb, proxy.aabb)) {
        continue;
      }

      candidateCount += 1;
      const shape = this.getShape(proxy.shapeId);
      const worldPose = this.getColliderWorldPose(proxy.colliderId);
      if (!shape || !worldPose) {
        continue;
      }

      testedShapeCount += 1;
      const remainingDistance = nearestHit ? Math.min(maxDistance, nearestHit.distance) : maxDistance;
      const shapeHit = queryShape
        ? castConvexShapesWithToi({
          queryShape: resolvedQueryShape,
          targetShape: shape,
          targetPose: worldPose,
          origin,
          rotation,
          direction,
          maxDistance: remainingDistance
        })
        : castType === 'capsule'
          ? capsuleCastShape({
            shape,
            worldPose,
            origin,
            direction,
            maxDistance: remainingDistance,
            radius,
            halfHeight,
            rotation
          })
          : sphereCastShape({
            shape,
            worldPose,
            origin,
            direction,
            maxDistance: remainingDistance,
            radius
          });

      if (!shapeHit) {
        continue;
      }

      if (!nearestHit || shapeHit.distance < nearestHit.distance - 1e-8) {
        nearestHit = {
          ...shapeHit,
          colliderId: proxy.colliderId,
          bodyId: proxy.bodyId,
          shapeId: proxy.shapeId,
          materialId: proxy.materialId,
          shapeType: proxy.shapeType
        };
      }
    }

    const shapeCastResult = nearestHit
      ? createShapeCastResult({
        castType,
        hit: true,
        origin,
        direction,
        maxDistance,
        radius,
        halfHeight,
        rotation,
        sampleOrigins,
        sampleEndPoints,
        distance: nearestHit.distance,
        point: nearestHit.point,
        normal: nearestHit.normal,
        sweepPosition: nearestHit.sweepPosition,
        colliderId: nearestHit.colliderId,
        bodyId: nearestHit.bodyId,
        shapeId: nearestHit.shapeId,
        materialId: nearestHit.materialId,
        shapeType: nearestHit.shapeType,
        algorithm: nearestHit.algorithm,
        featureId: nearestHit.featureId,
        sampleId: nearestHit.sampleId,
        proxyCount: broadphaseProxies.length,
        candidateCount,
        testedShapeCount
      })
      : createShapeCastMiss({
        castType,
        origin,
        direction,
        maxDistance,
        radius,
        halfHeight,
        rotation,
        sampleOrigins,
        sampleEndPoints,
        proxyCount: broadphaseProxies.length,
        candidateCount,
        testedShapeCount
      });

    if (options.storeResult !== false) {
      this.lastShapeCast = shapeCastResult;
    }

    return cloneShapeCastResult(shapeCastResult);
  }

  sphereCast(options = {}) {
    return this.shapeCastAgainstWorld({
      ...options,
      castType: 'sphere'
    });
  }

  capsuleCast(options = {}) {
    return this.shapeCastAgainstWorld({
      ...options,
      castType: 'capsule'
    });
  }

  getLastShapeCast() {
    return cloneShapeCastResult(this.lastShapeCast);
  }

  getLastCcdEvents() {
    return cloneCcdEvents(this.lastCcdEvents);
  }

  raycast(options = {}) {
    const origin = cloneVec3(options.origin ?? createVec3());
    const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
    const maxDistance = toNonNegativeNumber(options.maxDistance, 100);
    const collisionState = this.ensureCollisionState();
    let candidateCount = 0;
    let testedShapeCount = 0;
    let nearestHit = null;

    for (const proxy of collisionState.broadphaseProxies) {
      const broadphaseHit = intersectRayAabb(origin, direction, maxDistance, proxy.aabb);
      if (!broadphaseHit) {
        continue;
      }

      candidateCount += 1;
      if (nearestHit && broadphaseHit.distance > nearestHit.distance + 1e-8) {
        continue;
      }

      const shape = this.getShape(proxy.shapeId);
      const worldPose = this.getColliderWorldPose(proxy.colliderId);
      if (!shape || !worldPose) {
        continue;
      }

      testedShapeCount += 1;
      const shapeHit = raycastShape({
        shape,
        worldPose,
        origin,
        direction,
        maxDistance: nearestHit ? Math.min(maxDistance, nearestHit.distance) : maxDistance
      });

      if (!shapeHit) {
        continue;
      }

      if (!nearestHit || shapeHit.distance < nearestHit.distance - 1e-8) {
        nearestHit = {
          ...shapeHit,
          colliderId: proxy.colliderId,
          bodyId: proxy.bodyId,
          shapeId: proxy.shapeId,
          materialId: proxy.materialId,
          shapeType: proxy.shapeType
        };
      }
    }

    const raycastResult = nearestHit
      ? createRaycastResult({
        hit: true,
        origin,
        direction,
        maxDistance,
        distance: nearestHit.distance,
        point: nearestHit.point,
        normal: nearestHit.normal,
        colliderId: nearestHit.colliderId,
        bodyId: nearestHit.bodyId,
        shapeId: nearestHit.shapeId,
        materialId: nearestHit.materialId,
        shapeType: nearestHit.shapeType,
        algorithm: nearestHit.algorithm,
        featureId: nearestHit.featureId,
        proxyCount: collisionState.broadphaseProxies.length,
        candidateCount,
        testedShapeCount
      })
      : createRaycastMiss({
        origin,
        direction,
        maxDistance,
        proxyCount: collisionState.broadphaseProxies.length,
        candidateCount,
        testedShapeCount
      });

    this.lastRaycast = raycastResult;
    return cloneRaycastResult(raycastResult);
  }

  getLastRaycast() {
    return cloneRaycastResult(this.lastRaycast);
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
      const wireColor = body
        ? (body.sleeping ? DEFAULT_DEBUG_COLORS.sleepingRigidBodyWireframe : DEFAULT_DEBUG_COLORS.rigidBodyWireframe)
        : DEFAULT_DEBUG_COLORS.staticColliderWireframe;
      const wireCategory = body ? 'rigid-body' : 'static-collider';

      if (shape.type === 'convex-hull') {
        primitives.push(...buildConvexHullDebugLines(shape, pose, wireColor, source, wireCategory));
      } else {
        const wireHalfExtents = shape.type === 'box'
          ? shape.geometry.halfExtents
          : proxy.aabb.halfExtents;

        primitives.push(
          createDebugWireBox({
            id: `${proxy.colliderId}:wire-box`,
            category: wireCategory,
            center: pose.position,
            halfExtents: wireHalfExtents,
            rotation: pose.rotation,
            color: wireColor,
            source
          })
        );
      }

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

    for (const joint of collisionState.joints) {
      const anchors = this.getJointWorldAnchors(joint.id);
      if (!anchors) {
        continue;
      }

      const debugAxes = (joint.type === 'hinge-joint' || joint.type === 'fixed-joint') ? this.getJointWorldAxes(joint.id) : null;

      const bodyA = joint.bodyAId ? this.getBody(joint.bodyAId) : null;
      const bodyB = joint.bodyBId ? this.getBody(joint.bodyBId) : null;
      const sleeping = Boolean(bodyA?.sleeping) && Boolean(bodyB?.sleeping);
      const jointColor = joint.broken === true
        ? DEFAULT_DEBUG_COLORS.brokenJoint
        : joint.type === 'fixed-joint'
        ? (sleeping ? DEFAULT_DEBUG_COLORS.sleepingFixedJoint : DEFAULT_DEBUG_COLORS.fixedJoint)
        : joint.type === 'hinge-joint'
        ? (sleeping ? DEFAULT_DEBUG_COLORS.sleepingHingeJoint : DEFAULT_DEBUG_COLORS.hingeJoint)
        : joint.type === 'point-to-point-joint'
          ? (sleeping ? DEFAULT_DEBUG_COLORS.sleepingPointToPointJoint : DEFAULT_DEBUG_COLORS.pointToPointJoint)
          : (sleeping ? DEFAULT_DEBUG_COLORS.sleepingDistanceJoint : DEFAULT_DEBUG_COLORS.distanceJoint);
      const source = {
        jointId: joint.id,
        bodyAId: joint.bodyAId,
        bodyBId: joint.bodyBId,
        distance: joint.distance,
        jointType: joint.type
      };

      primitives.push(
        createDebugLine({
          id: `${joint.id}:distance-line`,
          category: joint.type,
          start: anchors.anchorA,
          end: anchors.anchorB,
          color: jointColor,
          source
        })
      );

      primitives.push(
        createDebugPoint({
          id: `${joint.id}:anchor-a`,
          category: `${joint.type}-anchor`,
          position: anchors.anchorA,
          color: jointColor,
          size: 4,
          source: {
            ...source,
            anchor: 'a'
          }
        })
      );

      primitives.push(
        createDebugPoint({
          id: `${joint.id}:anchor-b`,
          category: `${joint.type}-anchor`,
          position: anchors.anchorB,
          color: jointColor,
          size: 4,
          source: {
            ...source,
            anchor: 'b'
          }
        })
      );

      if (debugAxes) {
        const axisLength = 18;
        primitives.push(
          createDebugLine({
            id: `${joint.id}:axis-a`,
            category: joint.type === 'hinge-joint' ? 'hinge-axis-a' : 'fixed-axis-a',
            start: anchors.anchorA,
            end: addScaledVec3(anchors.anchorA, debugAxes.axisA, axisLength),
            color: DEFAULT_DEBUG_COLORS.hingeAxisA,
            source
          })
        );

        primitives.push(
          createDebugLine({
            id: `${joint.id}:axis-b`,
            category: joint.type === 'hinge-joint' ? 'hinge-axis-b' : 'fixed-axis-b',
            start: anchors.anchorB,
            end: addScaledVec3(anchors.anchorB, debugAxes.axisB, axisLength),
            color: DEFAULT_DEBUG_COLORS.hingeAxisB,
            source
          })
        );

        if (joint.type === 'hinge-joint' && joint.motorEnabled && Number(joint.maxMotorTorque ?? 0) > 0) {
          const motorSign = Number(joint.motorSpeed ?? 0) >= 0 ? 1 : -1;
          primitives.push(
          createDebugLine({
            id: `${joint.id}:motor`,
            category: 'hinge-motor',
            start: anchors.anchorB,
            end: addScaledVec3(anchors.anchorB, debugAxes.axisB, axisLength * motorSign),
            color: DEFAULT_DEBUG_COLORS.hingeMotor,
            source: {
              ...source,
              motorMode: joint.motorMode ?? 'speed',
              motorTargetAngle: joint.motorTargetAngle ?? 0,
              motorSpeed: joint.motorSpeed,
              maxMotorTorque: joint.maxMotorTorque
            }
            })
          );
        }
      }
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

    if (this.lastRaycast) {
      primitives.push(
        createDebugLine({
          id: 'raycast:last:line',
          category: 'raycast-line',
          start: this.lastRaycast.origin,
          end: this.lastRaycast.endPoint,
          color: this.lastRaycast.hit
            ? DEFAULT_DEBUG_COLORS.raycastHitLine
            : DEFAULT_DEBUG_COLORS.raycastMissLine,
          source: {
            hit: this.lastRaycast.hit,
            colliderId: this.lastRaycast.colliderId,
            bodyId: this.lastRaycast.bodyId,
            shapeType: this.lastRaycast.shapeType
          }
        })
      );

      if (this.lastRaycast.hit && this.lastRaycast.point && this.lastRaycast.normal) {
        primitives.push(
          createDebugPoint({
            id: 'raycast:last:hit-point',
            category: 'raycast-hit-point',
            position: this.lastRaycast.point,
            color: DEFAULT_DEBUG_COLORS.raycastHitPoint,
            size: 6,
            source: {
              colliderId: this.lastRaycast.colliderId,
              bodyId: this.lastRaycast.bodyId,
              distance: this.lastRaycast.distance
            }
          })
        );

        primitives.push(
          createDebugLine({
            id: 'raycast:last:hit-normal',
            category: 'raycast-hit-normal',
            start: this.lastRaycast.point,
            end: addScaledVec3(this.lastRaycast.point, this.lastRaycast.normal, 16),
            color: DEFAULT_DEBUG_COLORS.raycastHitNormal,
            source: {
              colliderId: this.lastRaycast.colliderId,
              bodyId: this.lastRaycast.bodyId
            }
          })
        );
      }
    }

    if (this.lastShapeCast) {
      this.lastShapeCast.sampleOrigins.forEach((sampleOrigin, sampleIndex) => {
        primitives.push(
          createDebugLine({
            id: `shape-cast:last:line:${sampleIndex}`,
            category: 'shape-cast-line',
            start: sampleOrigin,
            end: this.lastShapeCast.sampleEndPoints[sampleIndex] ?? addScaledVec3(sampleOrigin, this.lastShapeCast.direction, this.lastShapeCast.maxDistance),
            color: this.lastShapeCast.hit
              ? DEFAULT_DEBUG_COLORS.shapeCastHitLine
              : DEFAULT_DEBUG_COLORS.shapeCastMissLine,
            source: {
              castType: this.lastShapeCast.castType,
              sampleIndex,
              hit: this.lastShapeCast.hit
            }
          })
        );
      });

      if (this.lastShapeCast.hit && this.lastShapeCast.point && this.lastShapeCast.normal) {
        primitives.push(
          createDebugPoint({
            id: 'shape-cast:last:hit-point',
            category: 'shape-cast-hit-point',
            position: this.lastShapeCast.point,
            color: DEFAULT_DEBUG_COLORS.shapeCastHitPoint,
            size: 6,
            source: {
              castType: this.lastShapeCast.castType,
              colliderId: this.lastShapeCast.colliderId,
              sampleId: this.lastShapeCast.sampleId
            }
          })
        );

        primitives.push(
          createDebugLine({
            id: 'shape-cast:last:hit-normal',
            category: 'shape-cast-hit-normal',
            start: this.lastShapeCast.point,
            end: addScaledVec3(this.lastShapeCast.point, this.lastShapeCast.normal, 16),
            color: DEFAULT_DEBUG_COLORS.shapeCastHitNormal,
            source: {
              castType: this.lastShapeCast.castType,
              colliderId: this.lastShapeCast.colliderId
            }
          })
        );
      }
    }

    for (const [eventIndex, ccdEvent] of this.lastCcdEvents.entries()) {
      primitives.push(
        createDebugLine({
          id: `ccd:last:path:${eventIndex}`,
          category: 'ccd-path',
          start: ccdEvent.startPosition,
          end: ccdEvent.endPosition,
          color: DEFAULT_DEBUG_COLORS.ccdPath,
          source: {
            bodyId: ccdEvent.bodyId,
            targetColliderId: ccdEvent.targetColliderId,
            castType: ccdEvent.castType
          }
        })
      );

      if (ccdEvent.hitPosition && ccdEvent.normal) {
        primitives.push(
          createDebugPoint({
            id: `ccd:last:hit-point:${eventIndex}`,
            category: 'ccd-hit-point',
            position: ccdEvent.hitPosition,
            color: DEFAULT_DEBUG_COLORS.ccdHitPoint,
            size: 6,
            source: {
              bodyId: ccdEvent.bodyId,
              targetColliderId: ccdEvent.targetColliderId
            }
          })
        );

        primitives.push(
          createDebugLine({
            id: `ccd:last:hit-normal:${eventIndex}`,
            category: 'ccd-hit-normal',
            start: ccdEvent.hitPosition,
            end: addScaledVec3(ccdEvent.hitPosition, ccdEvent.normal, 16),
            color: DEFAULT_DEBUG_COLORS.ccdHitNormal,
            source: {
              bodyId: ccdEvent.bodyId,
              targetColliderId: ccdEvent.targetColliderId
            }
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
        jointCount: this.jointRegistry.count(),
        materialCount: this.materialRegistry.count(),
        broadphaseProxyCount: collisionState.summary.proxyCount,
        broadphasePairCount: collisionState.summary.pairCount,
        contactPairCount: collisionState.summary.contactCount,
        manifoldCount: collisionState.summary.manifoldCount,
        islandCount: collisionState.summary.islandCount,
        sleepingBodyCount: collisionState.summary.sleepingBodyCount,
        awakeBodyCount: collisionState.summary.awakeBodyCount,
        solverIterations: collisionState.solverStats.iterations,
        solvedContactCount: collisionState.solverStats.solvedContactCount,
        solvedTangentContactCount: collisionState.solverStats.solvedTangentContactCount,
        restitutionContactCount: collisionState.solverStats.restitutionContactCount,
        warmStartedContactCount: collisionState.solverStats.warmStartedContactCount,
        warmStartedJointCount: collisionState.solverStats.warmStartedJointCount,
        solvedJointCount: collisionState.solverStats.solvedJointCount,
        raycastHit: this.lastRaycast?.hit ?? false,
        shapeCastHit: this.lastShapeCast?.hit ?? false,
        ccdEventCount: this.lastCcdEvents.length,
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
      jointCount: this.jointRegistry.count(),
      materialCount: this.materialRegistry.count(),
      bodies: this.bodyRegistry.list(),
      shapes: this.shapeRegistry.list(),
      colliders: this.colliderRegistry.list(),
      joints: this.jointRegistry.list(),
      materials: this.materialRegistry.list(),
      collision: collisionState,
      simulationTick: this.simulationTick,
      renderFrameCount: this.renderFrameCount,
      fixedDeltaTime: this.fixedDeltaTime,
      gravity: cloneVec3(this.gravity),
      accumulatorSeconds: this.accumulatorSeconds,
      lastRaycast: cloneRaycastResult(this.lastRaycast),
      lastShapeCast: cloneShapeCastResult(this.lastShapeCast),
      lastCcdEvents: cloneCcdEvents(this.lastCcdEvents),
      lastStepStats: {
        ...this.lastStepStats
      },
      lastSolverStats: cloneSolverStats(this.lastSolverStats)
    };
  }
}
