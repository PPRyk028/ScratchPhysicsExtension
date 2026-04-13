import { cloneAabb, computeLocalShapeAabb, computeShapeWorldAabb, createAabbFromCenterHalfExtents, createAabbFromMinMax, expandAabb, intersectRayAabb, testAabbOverlap, testPointInAabb } from '../collision/aabb.js';
import { buildBroadphasePairs, cloneBroadphasePair, cloneBroadphaseProxy, createBroadphaseProxy } from '../collision/broadphase.js';
import { cloneContactPair, runNarrowphase } from '../collision/narrowphase.js';
import { capsuleCastShape, castConvexShapesWithToi, cloneRaycastResult, cloneShapeCastResult, computeSweptShapeAabb, createRaycastResult, createShapeCastResult, raycastShape, sphereCastShape } from '../collision/raycast.js';
import { composePoses, createTangentBasis, getClosestPointOnSegment, getShapeSupportFeature, transformPointByPose } from '../collision/support.js';
import { DEFAULT_DEBUG_COLORS, createDebugFrame, createDebugLine, createDebugPoint, createDebugWireBox } from '../debug/debug-primitives.js';
import { cloneManifold, ManifoldCache } from '../manifold/manifold-cache.js';
import { cloneQuat, createIdentityQuat, integrateQuat, inverseRotateVec3ByQuat, rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, addVec3, cloneVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';
import { buildRigidBodyIslandGraph, cloneRigidBodyIslandGraph, cloneRigidBodyIsland } from '../rigid/islands.js';
import { solveJointConstraints } from '../rigid/distance-joint-solver.js';
import { solveNormalContactConstraints } from '../rigid/normal-contact-solver.js';
import { ParticleWorld } from '../xpbd/particle-world.js';
import { BodyRegistry } from './body-registry.js';
import { CharacterRegistry } from './character-registry.js';
import { ColliderRegistry } from './collider-registry.js';
import { JointRegistry } from './joint-registry.js';
import { MaterialRegistry } from './material-registry.js';
import { ShapeRegistry } from './shape-registry.js';

const DEFAULT_MATERIAL_ID = 'material-default';
const DEFAULT_COLLISION_LAYER = 1;
const DEFAULT_COLLISION_MASK = 0x7fffffff;
const KINEMATIC_CHARACTER_CAST_MAX_DEPTH = 12;
const KINEMATIC_CHARACTER_CAST_DISTANCE_TOLERANCE = 0.05;
const KINEMATIC_FACE_QUERY_EPSILON = 1e-4;
const KINEMATIC_MOTION_FACE_APPROACH_EPSILON = 1e-3;
const KINEMATIC_MOTION_FACE_INSIDE_TOLERANCE = 0.15;
const KINEMATIC_ONE_WAY_MIN_UP = 0.1;
const KINEMATIC_ONE_WAY_RECOVERY_DEPTH = 0.35;

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

function normalizeCollisionBits(value, fallback) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) {
    return fallback;
  }

  return Math.max(0, Math.min(DEFAULT_COLLISION_MASK, Math.trunc(parsed)));
}

function shouldCollisionLayersInteract(layerA, maskA, layerB, maskB) {
  const resolvedLayerA = normalizeCollisionBits(layerA, DEFAULT_COLLISION_LAYER);
  const resolvedMaskA = normalizeCollisionBits(maskA, DEFAULT_COLLISION_MASK);
  const resolvedLayerB = normalizeCollisionBits(layerB, DEFAULT_COLLISION_LAYER);
  const resolvedMaskB = normalizeCollisionBits(maskB, DEFAULT_COLLISION_MASK);

  return (resolvedMaskA & resolvedLayerB) !== 0 && (resolvedMaskB & resolvedLayerA) !== 0;
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

function cloneSceneSettings(world) {
  return {
    fixedDeltaTime: Number(world.fixedDeltaTime),
    maxSubsteps: Number(world.maxSubsteps),
    solverIterations: Number(world.solverIterations),
    solverBaumgarte: Number(world.solverBaumgarte),
    allowedPenetration: Number(world.allowedPenetration),
    positionCorrectionPercent: Number(world.positionCorrectionPercent),
    sleepEnabled: world.sleepEnabled !== false,
    sleepLinearThreshold: Number(world.sleepLinearThreshold),
    sleepAngularThreshold: Number(world.sleepAngularThreshold),
    sleepTimeThreshold: Number(world.sleepTimeThreshold),
    ccdEnabled: world.ccdEnabled !== false,
    ccdMotionThreshold: Number(world.ccdMotionThreshold),
    ccdSafetyMargin: Number(world.ccdSafetyMargin),
    xpbdIterations: Number(world.particleWorld.iterations),
    xpbdSubsteps: Number(world.particleWorld.substeps),
    xpbdDefaultDamping: Number(world.particleWorld.defaultDamping)
  };
}

function clampUnitInterval(value) {
  return Math.max(-1, Math.min(1, Number(value ?? 0)));
}

function radiansToDegrees(value) {
  return Number(value ?? 0) * (180 / Math.PI);
}

function createDefaultGroundState() {
  return {
    grounded: false,
    walkable: false,
    distance: null,
    angleDegrees: null,
    normal: createVec3(0, 1, 0),
    point: createVec3(),
    colliderId: null,
    bodyId: null
  };
}

function getBodyVelocityAtPoint(body, contactPosition) {
  if (!body) {
    return createVec3();
  }

  const offset = subtractVec3(contactPosition, body.position);
  return addVec3(body.linearVelocity, crossVec3(body.angularVelocity, offset));
}

function getRelativeVelocityAtPoint(bodyA, bodyB, contactPosition) {
  return subtractVec3(
    getBodyVelocityAtPoint(bodyB, contactPosition),
    getBodyVelocityAtPoint(bodyA, contactPosition)
  );
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

function createBodyPairKey(bodyAId, bodyBId) {
  const left = String(bodyAId ?? '').trim();
  const right = String(bodyBId ?? '').trim();
  if (!left || !right) {
    return null;
  }

  return [left, right].sort().join('|');
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

function createContactEvent(options = {}) {
  return {
    id: String(options.id ?? `${options.eventType ?? 'contact'}:${options.phase ?? 'stay'}:${options.pairKey ?? 'pair'}`).trim(),
    eventType: String(options.eventType ?? 'contact').trim() || 'contact',
    phase: String(options.phase ?? 'stay').trim() || 'stay',
    pairKey: String(options.pairKey ?? '').trim() || null,
    pairKind: String(options.pairKind ?? '').trim() || 'dynamic-dynamic',
    colliderAId: String(options.colliderAId ?? '').trim() || null,
    colliderBId: String(options.colliderBId ?? '').trim() || null,
    bodyAId: String(options.bodyAId ?? '').trim() || null,
    bodyBId: String(options.bodyBId ?? '').trim() || null,
    shapeAType: String(options.shapeAType ?? '').trim() || 'unknown',
    shapeBType: String(options.shapeBType ?? '').trim() || 'unknown',
    algorithm: String(options.algorithm ?? '').trim() || 'unknown',
    isSensorPair: options.isSensorPair === true,
    contactCount: Number(options.contactCount ?? 0),
    penetration: Number(options.penetration ?? 0),
    normal: cloneVec3(options.normal ?? createVec3()),
    contacts: Array.isArray(options.contacts)
      ? options.contacts.map((contact) => ({
        id: String(contact.id ?? '').trim() || null,
        featureId: String(contact.featureId ?? '').trim() || null,
        position: cloneVec3(contact.position ?? createVec3()),
        penetration: Number(contact.penetration ?? 0),
        separation: Number(contact.separation ?? 0)
      }))
      : []
  };
}

function cloneContactEvent(contactEvent) {
  return createContactEvent(contactEvent);
}

function createEmptyContactEventState() {
  return {
    contactPairsByKey: new Map(),
    triggerPairsByKey: new Map(),
    contactEvents: [],
    triggerEvents: []
  };
}

function cloneContactEventState(contactEventState) {
  return {
    contactPairsByKey: new Map(Array.from(contactEventState.contactPairsByKey.entries(), ([pairKey, pair]) => [pairKey, cloneContactPair(pair)])),
    triggerPairsByKey: new Map(Array.from(contactEventState.triggerPairsByKey.entries(), ([pairKey, pair]) => [pairKey, cloneContactPair(pair)])),
    contactEvents: Array.isArray(contactEventState.contactEvents)
      ? contactEventState.contactEvents.map((event) => cloneContactEvent(event))
      : [],
    triggerEvents: Array.isArray(contactEventState.triggerEvents)
      ? contactEventState.triggerEvents.map((event) => cloneContactEvent(event))
      : []
  };
}

function buildPairEventState(currentPairs, previousPairsByKey, eventType) {
  const nextPairsByKey = new Map();
  const events = [];

  for (const pair of Array.isArray(currentPairs) ? currentPairs : []) {
    nextPairsByKey.set(pair.pairKey, cloneContactPair(pair));
    const phase = previousPairsByKey.has(pair.pairKey) ? 'stay' : 'enter';
    events.push(createContactEvent({
      ...pair,
      eventType,
      phase,
      isSensorPair: eventType === 'trigger'
    }));
  }

  for (const [pairKey, previousPair] of previousPairsByKey.entries()) {
    if (nextPairsByKey.has(pairKey)) {
      continue;
    }

    events.push(createContactEvent({
      ...previousPair,
      eventType,
      phase: 'exit',
      isSensorPair: eventType === 'trigger'
    }));
  }

  return {
    nextPairsByKey,
    events
  };
}

function createEmptyCollisionState(iterations = 0) {
  return {
    broadphaseProxies: [],
    broadphasePairs: [],
    contactPairs: [],
    sensorPairs: [],
    manifolds: [],
    islands: [],
    joints: [],
    contactEvents: [],
    triggerEvents: [],
    solverStats: createEmptySolverStats(iterations),
    summary: {
      proxyCount: 0,
      pairCount: 0,
      contactCount: 0,
      sensorPairCount: 0,
      manifoldCount: 0,
      jointCount: 0,
      islandCount: 0,
      sleepingBodyCount: 0,
      awakeBodyCount: 0,
      contactEnterCount: 0,
      contactStayCount: 0,
      contactExitCount: 0,
      triggerEnterCount: 0,
      triggerStayCount: 0,
      triggerExitCount: 0,
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
    sensorPairCount: Number(summary.sensorPairCount ?? 0),
    manifoldCount: summary.manifoldCount,
    jointCount: Number(summary.jointCount ?? 0),
    islandCount: summary.islandCount,
    sleepingBodyCount: summary.sleepingBodyCount,
    awakeBodyCount: summary.awakeBodyCount,
    contactEnterCount: Number(summary.contactEnterCount ?? 0),
    contactStayCount: Number(summary.contactStayCount ?? 0),
    contactExitCount: Number(summary.contactExitCount ?? 0),
    triggerEnterCount: Number(summary.triggerEnterCount ?? 0),
    triggerStayCount: Number(summary.triggerStayCount ?? 0),
    triggerExitCount: Number(summary.triggerExitCount ?? 0),
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
    sensorPairs: Array.isArray(collisionState.sensorPairs)
      ? collisionState.sensorPairs.map((contactPair) => cloneContactPair(contactPair))
      : [],
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
    contactEvents: Array.isArray(collisionState.contactEvents)
      ? collisionState.contactEvents.map((event) => cloneContactEvent(event))
      : [],
    triggerEvents: Array.isArray(collisionState.triggerEvents)
      ? collisionState.triggerEvents.map((event) => cloneContactEvent(event))
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

function isBlockingKinematicHit(direction, normal) {
  if (!normal) {
    return true;
  }

  return dotVec3(direction, normal) < -1e-4;
}

function isWalkableKinematicNormal(normal, maxGroundAngleDegrees) {
  if (!normal) {
    return false;
  }

  const up = createVec3(0, 1, 0);
  const normalizedNormal = normalizeVec3(normal, up);
  const maxSlopeRadians = toNonNegativeNumber(maxGroundAngleDegrees, 55) * (Math.PI / 180);
  return dotVec3(normalizedNormal, up) >= Math.cos(maxSlopeRadians + 1e-6);
}

function projectOntoGroundPlane(move, groundNormal) {
  if (!groundNormal) {
    return cloneVec3(move ?? createVec3());
  }

  return subtractVec3(move, scaleVec3(groundNormal, dotVec3(move, groundNormal)));
}

function getForwardProgress(move, actualMove) {
  const moveLengthSquared = lengthSquaredVec3(move);
  if (moveLengthSquared <= 1e-8) {
    return 0;
  }

  const direction = normalizeVec3(move, createVec3(1, 0, 0));
  return dotVec3(actualMove, direction);
}

function getBoxLocalCorners(halfExtents) {
  return [
    createVec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
    createVec3(halfExtents.x, -halfExtents.y, -halfExtents.z),
    createVec3(halfExtents.x, halfExtents.y, -halfExtents.z),
    createVec3(-halfExtents.x, halfExtents.y, -halfExtents.z),
    createVec3(-halfExtents.x, -halfExtents.y, halfExtents.z),
    createVec3(halfExtents.x, -halfExtents.y, halfExtents.z),
    createVec3(halfExtents.x, halfExtents.y, halfExtents.z),
    createVec3(-halfExtents.x, halfExtents.y, halfExtents.z)
  ];
}

function createWorldFace(id, normal, points) {
  if (!Array.isArray(points) || points.length < 3) {
    return null;
  }

  const resolvedNormal = normalizeVec3(normal, createVec3(0, 1, 0));
  return {
    id,
    normal: resolvedNormal,
    planeOffset: dotVec3(points[0], resolvedNormal),
    points: points.map((point) => cloneVec3(point))
  };
}

function getStaticPolygonalWorldFaces(shape, pose) {
  if (shape?.type === 'box') {
    const worldPose = composePoses({
      position: createVec3(),
      rotation: createIdentityQuat()
    }, pose ?? {
      position: createVec3(),
      rotation: createIdentityQuat()
    });
    const localCorners = getBoxLocalCorners(shape.geometry.halfExtents);
    const faceDefinitions = [
      { id: 'face:x+', normal: createVec3(1, 0, 0), indices: [1, 5, 6, 2] },
      { id: 'face:x-', normal: createVec3(-1, 0, 0), indices: [4, 0, 3, 7] },
      { id: 'face:y+', normal: createVec3(0, 1, 0), indices: [3, 2, 6, 7] },
      { id: 'face:y-', normal: createVec3(0, -1, 0), indices: [0, 4, 5, 1] },
      { id: 'face:z+', normal: createVec3(0, 0, 1), indices: [4, 7, 6, 5] },
      { id: 'face:z-', normal: createVec3(0, 0, -1), indices: [0, 1, 2, 3] }
    ];

    return faceDefinitions
      .map((definition) => createWorldFace(
        definition.id,
        rotateVec3ByQuat(worldPose.rotation, definition.normal),
        definition.indices.map((index) => transformPointByPose(worldPose, localCorners[index]))
      ))
      .filter(Boolean);
  }

  if (shape?.type === 'convex-hull') {
    const worldPose = composePoses({
      position: createVec3(),
      rotation: createIdentityQuat()
    }, pose ?? {
      position: createVec3(),
      rotation: createIdentityQuat()
    });
    const localVertices = Array.isArray(shape.geometry?.vertices) ? shape.geometry.vertices : [];
    const faces = Array.isArray(shape.geometry?.faces) ? shape.geometry.faces : [];

    return faces
      .map((face, faceIndex) => {
        const boundaryIndices = Array.isArray(face.boundaryIndices) ? face.boundaryIndices : [];
        const points = boundaryIndices
          .map((vertexIndex) => localVertices[vertexIndex])
          .filter(Boolean)
          .map((localPoint) => transformPointByPose(worldPose, localPoint));
        return createWorldFace(
          `face:${faceIndex}`,
          rotateVec3ByQuat(worldPose.rotation, face.normal ?? createVec3(0, 1, 0)),
          points
        );
      })
      .filter(Boolean);
  }

  return [];
}

function isPointInsideWorldFace(point, face, tolerance = KINEMATIC_FACE_QUERY_EPSILON) {
  if (!face || !Array.isArray(face.points) || face.points.length < 3) {
    return false;
  }

  let windingSign = 0;
  for (let index = 0; index < face.points.length; index += 1) {
    const start = face.points[index];
    const end = face.points[(index + 1) % face.points.length];
    const edge = subtractVec3(end, start);
    const toPoint = subtractVec3(point, start);
    const side = dotVec3(crossVec3(edge, toPoint), face.normal);
    if (Math.abs(side) <= tolerance) {
      continue;
    }

    if (windingSign === 0) {
      windingSign = Math.sign(side);
      continue;
    }

    if (side * windingSign < -tolerance) {
      return false;
    }
  }

  return true;
}

function getPointDistanceSquaredToWorldFaceBoundary(point, face) {
  if (!face || !Array.isArray(face.points) || face.points.length < 2) {
    return Number.POSITIVE_INFINITY;
  }

  let bestDistanceSquared = Number.POSITIVE_INFINITY;
  for (let index = 0; index < face.points.length; index += 1) {
    const start = face.points[index];
    const end = face.points[(index + 1) % face.points.length];
    const closestPoint = getClosestPointOnSegment(point, start, end);
    const delta = subtractVec3(point, closestPoint);
    bestDistanceSquared = Math.min(bestDistanceSquared, lengthSquaredVec3(delta));
  }

  return bestDistanceSquared;
}

function isOneWayPlatformCollider(colliderLike) {
  return colliderLike?.isOneWay === true && colliderLike?.isSensor !== true;
}

function isUpwardOneWayFace(face) {
  return Number(face?.normal?.y ?? 0) > KINEMATIC_ONE_WAY_MIN_UP;
}

function shouldIgnoreKinematicOneWayMotionFace(faceContext, direction) {
  if (!isOneWayPlatformCollider(faceContext)) {
    return false;
  }

  if (!isUpwardOneWayFace(faceContext.face)) {
    return true;
  }

  return Number(direction?.y ?? 0) >= -1e-4;
}

function shouldIgnoreKinematicOneWayCastHit(collider, hit, direction) {
  if (!isOneWayPlatformCollider(collider)) {
    return false;
  }

  if (Number(hit?.normal?.y ?? 0) <= KINEMATIC_ONE_WAY_MIN_UP) {
    return true;
  }

  return Number(direction?.y ?? 0) >= -1e-4;
}

function ensureKinematicFaceCache(character, cacheKey) {
  if (!character[cacheKey] || typeof character[cacheKey] !== 'object') {
    character[cacheKey] = {
      colliderId: null,
      bodyId: null,
      faceId: null,
      normal: createVec3(0, 1, 0),
      point: createVec3(),
      distance: null,
      stableFrames: 0
    };
  }

  return character[cacheKey];
}

function clearKinematicFaceCache(character, cacheKey) {
  const cache = ensureKinematicFaceCache(character, cacheKey);
  cache.colliderId = null;
  cache.bodyId = null;
  cache.faceId = null;
  cache.normal = createVec3(0, 1, 0);
  cache.point = createVec3();
  cache.distance = null;
  cache.stableFrames = 0;
}

function updateKinematicFaceCache(character, cacheKey, hit) {
  const cache = ensureKinematicFaceCache(character, cacheKey);
  if (!hit?.hit || !hit.colliderId || !hit.featureId) {
    clearKinematicFaceCache(character, cacheKey);
    return cache;
  }

  const isSameFace = cache.colliderId === hit.colliderId && cache.faceId === hit.featureId;
  cache.colliderId = hit.colliderId;
  cache.bodyId = hit.bodyId ?? null;
  cache.faceId = hit.featureId ?? null;
  cache.normal = cloneVec3(hit.normal ?? createVec3(0, 1, 0));
  cache.point = cloneVec3(hit.point ?? createVec3());
  cache.distance = hit.distance ?? null;
  cache.stableFrames = isSameFace ? cache.stableFrames + 1 : 1;
  return cache;
}

function getPrioritizedStaticProxyList(proxies, prioritizedColliderIds = []) {
  if (!Array.isArray(proxies) || proxies.length <= 1 || !Array.isArray(prioritizedColliderIds) || prioritizedColliderIds.length === 0) {
    return Array.isArray(proxies) ? proxies : [];
  }

  const priorities = prioritizedColliderIds
    .map((id) => String(id ?? '').trim())
    .filter(Boolean);
  if (priorities.length === 0) {
    return proxies;
  }

  const prioritySet = new Set(priorities);
  const prioritized = [];
  const remainder = [];

  for (const proxy of proxies) {
    if (prioritySet.has(proxy.colliderId)) {
      prioritized.push(proxy);
    } else {
      remainder.push(proxy);
    }
  }

  prioritized.sort((left, right) => priorities.indexOf(left.colliderId) - priorities.indexOf(right.colliderId));
  return prioritized.concat(remainder);
}

function ensureKinematicCandidateCache(character, cacheKey) {
  if (!character[cacheKey] || typeof character[cacheKey] !== 'object') {
    character[cacheKey] = {
      bounds: null,
      colliderIds: [],
      stableFrames: 0
    };
  }

  if (!Array.isArray(character[cacheKey].colliderIds)) {
    character[cacheKey].colliderIds = [];
  }

  return character[cacheKey];
}

function clearKinematicCandidateCache(character, cacheKey) {
  const cache = ensureKinematicCandidateCache(character, cacheKey);
  cache.bounds = null;
  cache.colliderIds = [];
  cache.stableFrames = 0;
}

function cloneOptionalAabb(aabb) {
  if (!aabb) {
    return null;
  }

  if (aabb.center && aabb.halfExtents) {
    return cloneAabb(aabb);
  }

  if (aabb.min && aabb.max) {
    return createAabbFromMinMax(aabb.min, aabb.max);
  }

  return null;
}

function areColliderIdListsEqual(left, right) {
  if (!Array.isArray(left) || !Array.isArray(right) || left.length !== right.length) {
    return false;
  }

  for (let index = 0; index < left.length; index += 1) {
    if (left[index] !== right[index]) {
      return false;
    }
  }

  return true;
}

function isAabbContainedWithin(inner, outer, epsilon = 1e-6) {
  if (!inner || !outer) {
    return false;
  }

  const resolvedInner = cloneOptionalAabb(inner);
  const resolvedOuter = cloneOptionalAabb(outer);
  if (!resolvedInner || !resolvedOuter) {
    return false;
  }

  return (
    resolvedInner.min.x >= resolvedOuter.min.x - epsilon &&
    resolvedInner.min.y >= resolvedOuter.min.y - epsilon &&
    resolvedInner.min.z >= resolvedOuter.min.z - epsilon &&
    resolvedInner.max.x <= resolvedOuter.max.x + epsilon &&
    resolvedInner.max.y <= resolvedOuter.max.y + epsilon &&
    resolvedInner.max.z <= resolvedOuter.max.z + epsilon
  );
}

function updateKinematicCandidateCache(character, cacheKey, queryAabb, broadphaseProxies, options = {}) {
  const cache = ensureKinematicCandidateCache(character, cacheKey);
  const resolvedQueryAabb = cloneOptionalAabb(queryAabb);
  if (!resolvedQueryAabb) {
    clearKinematicCandidateCache(character, cacheKey);
    return cache;
  }

  const padding = Math.max(0, Number(options.padding ?? 0));
  const expandedBounds = padding > 0
    ? expandAabb(resolvedQueryAabb, createVec3(padding, padding, padding))
    : resolvedQueryAabb;
  const nextColliderIds = [];

  for (const proxy of Array.isArray(broadphaseProxies) ? broadphaseProxies : []) {
    if (proxy.isSensor || (proxy.shapeType !== 'box' && proxy.shapeType !== 'convex-hull')) {
      continue;
    }

    if (!testAabbOverlap(expandedBounds, proxy.aabb)) {
      continue;
    }

    nextColliderIds.push(proxy.colliderId);
  }

  const isSameSet = areColliderIdListsEqual(cache.colliderIds, nextColliderIds);
  cache.bounds = expandedBounds;
  cache.colliderIds = nextColliderIds;
  cache.stableFrames = isSameSet ? cache.stableFrames + 1 : 1;
  return cache;
}

function getReusableKinematicCandidateProxies(character, cacheKey, queryAabb, broadphaseProxies, prioritizedColliderIds = []) {
  const cache = ensureKinematicCandidateCache(character, cacheKey);
  const resolvedQueryAabb = cloneOptionalAabb(queryAabb);
  if (!resolvedQueryAabb || !cache.bounds || !isAabbContainedWithin(resolvedQueryAabb, cache.bounds)) {
    return null;
  }

  const proxyByColliderId = new Map();
  for (const proxy of Array.isArray(broadphaseProxies) ? broadphaseProxies : []) {
    proxyByColliderId.set(proxy.colliderId, proxy);
  }

  const proxies = [];
  for (const colliderId of cache.colliderIds) {
    const proxy = proxyByColliderId.get(colliderId);
    if (proxy) {
      proxies.push(proxy);
    }
  }

  return {
    bounds: cache.bounds,
    proxies: getPrioritizedStaticProxyList(proxies, prioritizedColliderIds),
    reused: true,
    stableFrames: cache.stableFrames
  };
}

function hasKinematicFallbackBroadphaseOverlap(queryAabb, broadphaseProxies, options = {}) {
  const resolvedQueryAabb = cloneOptionalAabb(queryAabb);
  if (!resolvedQueryAabb || !Array.isArray(broadphaseProxies) || broadphaseProxies.length === 0) {
    return false;
  }

  const excludeBodyId = String(options.excludeBodyId ?? '').trim() || null;
  const excludeColliderIds = new Set(Array.isArray(options.excludeColliderIds) ? options.excludeColliderIds.map((id) => String(id ?? '').trim()) : []);
  const ignoreSensors = options.ignoreSensors !== false;

  for (const proxy of broadphaseProxies) {
    if (excludeColliderIds.has(proxy.colliderId)) {
      continue;
    }

    if (excludeBodyId && proxy.bodyId === excludeBodyId) {
      continue;
    }

    if (ignoreSensors && proxy.isSensor) {
      continue;
    }

    if (!testAabbOverlap(resolvedQueryAabb, proxy.aabb)) {
      continue;
    }

    if (proxy.motionType !== 'static' || (proxy.shapeType !== 'box' && proxy.shapeType !== 'convex-hull')) {
      return true;
    }
  }

  return false;
}

function findWorldFaceById(faces, faceId) {
  if (!Array.isArray(faces) || !faceId) {
    return null;
  }

  return faces.find((face) => face?.id === faceId) ?? null;
}

function createKinematicBottomSphereCenter(character, position) {
  return addVec3(position, createVec3(0, -toPositiveNumber(character?.halfHeight, 0), 0));
}

function createSweptBottomSphereAabb(center, radius, maxDistance) {
  const resolvedRadius = Math.max(0, Number(radius ?? 0));
  const endCenter = addVec3(center, createVec3(0, -Math.max(0, Number(maxDistance ?? 0)), 0));
  const minY = Math.min(center.y, endCenter.y) - resolvedRadius;
  const maxY = Math.max(center.y, endCenter.y) + resolvedRadius;
  return createAabbFromMinMax(
    createVec3(
      Math.min(center.x, endCenter.x) - resolvedRadius,
      minY,
      Math.min(center.z, endCenter.z) - resolvedRadius
    ),
    createVec3(
      Math.max(center.x, endCenter.x) + resolvedRadius,
      maxY,
      Math.max(center.z, endCenter.z) + resolvedRadius
    )
  );
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
    this.characterRegistry = new CharacterRegistry();
    this.colliderRegistry = new ColliderRegistry();
    this.jointRegistry = new JointRegistry();
    this.materialRegistry = new MaterialRegistry();
    this.particleWorld = new ParticleWorld({
      gravity: this.gravity,
      iterations: options.xpbdIterations,
      substeps: options.xpbdSubsteps,
      defaultDamping: options.xpbdDefaultDamping
    });
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
    this.lastXpbdStats = this.particleWorld.getSnapshot().lastStepStats;
    this.lastRaycast = null;
    this.lastShapeCast = null;
    this.lastCcdEvents = [];
    this.lastIslandState = createEmptyIslandState();
    this.lastContactEventState = createEmptyContactEventState();
    this.collisionStateDirty = true;
    this.collisionState = createEmptyCollisionState(this.solverIterations);
    this.manifoldCache.clear();
  }

  reset() {
    this.shapeRegistry.clear();
    this.bodyRegistry.clear();
    this.characterRegistry.clear();
    this.colliderRegistry.clear();
    this.jointRegistry.clear();
    this.materialRegistry.clear();
    this.particleWorld.reset();
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
    this.particleWorld.setGravity(this.gravity);
  }

  exportSceneDefinition() {
    return {
      schemaVersion: 'physics-scene@1',
      settings: cloneSceneSettings(this),
      gravity: cloneVec3(this.gravity),
      debugCamera: cloneCamera(this.debugCamera),
      materials: this.materialRegistry.list(),
      shapes: this.shapeRegistry.list(),
      bodies: this.bodyRegistry.list(),
      characters: this.characterRegistry.list(),
      colliders: this.colliderRegistry.list(),
      joints: this.jointRegistry.list(),
      xpbd: this.particleWorld.exportSceneDefinition()
    };
  }

  importSceneDefinition(sceneDefinition = {}, options = {}) {
    const shouldReset = options.reset !== false;
    const scene = typeof sceneDefinition === 'object' && sceneDefinition !== null ? sceneDefinition : {};

    if (shouldReset) {
      this.reset();
    }

    const settings = typeof scene.settings === 'object' && scene.settings !== null ? scene.settings : {};
    this.fixedDeltaTime = toPositiveNumber(settings.fixedDeltaTime, this.fixedDeltaTime);
    this.maxSubsteps = Math.max(1, Math.floor(toPositiveNumber(settings.maxSubsteps, this.maxSubsteps)));
    this.solverIterations = Math.max(1, Math.floor(toPositiveNumber(settings.solverIterations, this.solverIterations)));
    this.solverBaumgarte = toNonNegativeNumber(settings.solverBaumgarte, this.solverBaumgarte);
    this.allowedPenetration = toNonNegativeNumber(settings.allowedPenetration, this.allowedPenetration);
    this.positionCorrectionPercent = toNonNegativeNumber(settings.positionCorrectionPercent, this.positionCorrectionPercent);
    this.sleepEnabled = settings.sleepEnabled !== undefined ? settings.sleepEnabled !== false : this.sleepEnabled;
    this.sleepLinearThreshold = toNonNegativeNumber(settings.sleepLinearThreshold, this.sleepLinearThreshold);
    this.sleepAngularThreshold = toNonNegativeNumber(settings.sleepAngularThreshold, this.sleepAngularThreshold);
    this.sleepTimeThreshold = toNonNegativeNumber(settings.sleepTimeThreshold, this.sleepTimeThreshold);
    this.ccdEnabled = settings.ccdEnabled !== undefined ? settings.ccdEnabled !== false : this.ccdEnabled;
    this.ccdMotionThreshold = toNonNegativeNumber(settings.ccdMotionThreshold, this.ccdMotionThreshold);
    this.ccdSafetyMargin = toNonNegativeNumber(settings.ccdSafetyMargin, this.ccdSafetyMargin);
    this.particleWorld.iterations = Math.max(1, Math.floor(toPositiveNumber(settings.xpbdIterations, this.particleWorld.iterations)));
    this.particleWorld.substeps = Math.max(1, Math.floor(toPositiveNumber(settings.xpbdSubsteps, this.particleWorld.substeps)));
    this.particleWorld.defaultDamping = toNonNegativeNumber(settings.xpbdDefaultDamping, this.particleWorld.defaultDamping);

    if (scene.gravity) {
      this.setGravity(scene.gravity);
    }

    if (scene.debugCamera?.position) {
      this.setDebugCameraPosition(scene.debugCamera.position);
    }

    if (scene.debugCamera?.target) {
      this.setDebugCameraTarget(scene.debugCamera.target);
    }

    for (const material of Array.isArray(scene.materials) ? scene.materials : []) {
      this.createMaterial(material);
    }

    for (const shape of Array.isArray(scene.shapes) ? scene.shapes : []) {
      if (!shape?.id || !shape?.type) {
        continue;
      }

      if (shape.type === 'box') {
        this.createBoxShape({
          id: shape.id,
          halfExtents: shape.geometry?.halfExtents,
          localPose: shape.localPose,
          userData: shape.userData
        });
        continue;
      }

      if (shape.type === 'sphere') {
        this.createSphereShape({
          id: shape.id,
          radius: shape.geometry?.radius,
          localPose: shape.localPose,
          userData: shape.userData
        });
        continue;
      }

      if (shape.type === 'capsule') {
        this.createCapsuleShape({
          id: shape.id,
          radius: shape.geometry?.radius,
          halfHeight: shape.geometry?.halfHeight,
          localPose: shape.localPose,
          userData: shape.userData
        });
        continue;
      }

      if (shape.type === 'convex-hull') {
        this.createConvexHullShape({
          id: shape.id,
          vertices: shape.geometry?.vertices,
          localPose: shape.localPose,
          userData: shape.userData
        });
      }
    }

    for (const body of Array.isArray(scene.bodies) ? scene.bodies : []) {
      if (!body?.id) {
        continue;
      }

      this.createRigidBody({
        id: body.id,
        motionType: body.motionType,
        shapeId: body.shapeId,
        primaryColliderId: body.primaryColliderId,
        colliderIds: body.colliderIds,
        position: body.position,
        rotation: body.rotation,
        linearVelocity: body.linearVelocity,
        angularVelocity: body.angularVelocity,
        forceAccumulator: body.forceAccumulator,
        torqueAccumulator: body.torqueAccumulator,
        mass: body.mass,
        canSleep: body.canSleep,
        sleeping: body.sleeping,
        sleepTimer: body.sleepTimer,
        enabled: body.enabled,
        userData: body.userData
      });
    }

    for (const character of Array.isArray(scene.characters) ? scene.characters : []) {
      if (!character?.id) {
        continue;
      }

      this.characterRegistry.createKinematicCapsule(character);
    }

    for (const collider of Array.isArray(scene.colliders) ? scene.colliders : []) {
      if (!collider?.id) {
        continue;
      }

      this.createCollider({
        id: collider.id,
        shapeId: collider.shapeId,
        bodyId: collider.bodyId,
        materialId: collider.materialId,
        enabled: collider.enabled,
        isSensor: collider.isSensor,
        collisionLayer: collider.collisionLayer,
        collisionMask: collider.collisionMask,
        localPose: collider.localPose,
        userData: collider.userData
      });
    }

    for (const joint of Array.isArray(scene.joints) ? scene.joints : []) {
      if (!joint?.id || !joint?.type) {
        continue;
      }

      let createdJoint = null;
      if (joint.type === 'distance-joint') {
        createdJoint = this.jointRegistry.createDistanceJoint(joint);
      } else if (joint.type === 'point-to-point-joint') {
        createdJoint = this.jointRegistry.createPointToPointJoint(joint);
      } else if (joint.type === 'hinge-joint') {
        createdJoint = this.jointRegistry.createHingeJoint(joint);
      } else if (joint.type === 'fixed-joint') {
        createdJoint = this.jointRegistry.createFixedJoint(joint);
      }

      if (createdJoint) {
        const mutableJoint = this.jointRegistry.getMutable(createdJoint.id);
        if (mutableJoint) {
          mutableJoint.enabled = joint.enabled !== false;
          mutableJoint.broken = joint.broken === true;
          mutableJoint.lastAppliedForce = 0;
          mutableJoint.lastAppliedTorque = 0;
          mutableJoint.accumulatedImpulse = 0;
          mutableJoint.accumulatedLinearImpulse = createVec3();
          mutableJoint.accumulatedAngularImpulse = createVec3();
          mutableJoint.accumulatedMotorImpulse = 0;
        }
      }
    }

    if (scene.xpbd) {
      this.particleWorld.importSceneDefinition(scene.xpbd);
      this.particleWorld.setGravity(this.gravity);
    }

    const cameraPosition = scene.debugCamera?.position ? cloneVec3(scene.debugCamera.position) : cloneVec3(this.debugCamera.position);
    const cameraTarget = scene.debugCamera?.target ? cloneVec3(scene.debugCamera.target) : cloneVec3(this.debugCamera.target);
    this.resetRuntimeState();
    this.debugCamera.position = cameraPosition;
    this.debugCamera.target = cameraTarget;
    this.markCollisionStateDirty();
    this.refreshKinematicCapsules();

    return {
      bodyCount: this.bodyRegistry.count(),
      characterCount: this.characterRegistry.count(),
      colliderCount: this.colliderRegistry.count(),
      jointCount: this.jointRegistry.count(),
      materialCount: this.materialRegistry.count(),
      shapeCount: this.shapeRegistry.count(),
      clothCount: this.particleWorld.countCloths(),
      softBodyCount: this.particleWorld.countSoftBodies(),
      particleCount: this.particleWorld.countParticles()
    };
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
      materialId: this.resolveMaterialId(options.materialId),
      isOneWay: options.isOneWay,
      collisionLayer: normalizeCollisionBits(options.collisionLayer, DEFAULT_COLLISION_LAYER),
      collisionMask: normalizeCollisionBits(options.collisionMask, DEFAULT_COLLISION_MASK)
    });

    if (collider.bodyId) {
      this.bodyRegistry.attachCollider(collider.bodyId, collider.id, collider.shapeId);
      this.updateBodyMassProperties(collider.bodyId);
    }

    this.markCollisionStateDirty();
    return collider;
  }

  configureColliderCollision(colliderId, options = {}) {
    const collider = this.colliderRegistry.getMutable(colliderId);
    if (!collider) {
      return null;
    }

    if (options.collisionLayer !== undefined) {
      collider.collisionLayer = normalizeCollisionBits(options.collisionLayer, collider.collisionLayer ?? DEFAULT_COLLISION_LAYER);
    }

    if (options.collisionMask !== undefined) {
      collider.collisionMask = normalizeCollisionBits(options.collisionMask, collider.collisionMask ?? DEFAULT_COLLISION_MASK);
    }

    if (options.isSensor !== undefined) {
      collider.isSensor = Boolean(options.isSensor);
      if (collider.isSensor) {
        collider.isOneWay = false;
      }
    }

    if (options.enabled !== undefined) {
      collider.enabled = Boolean(options.enabled);
    }

    if (options.isOneWay !== undefined && !collider.isSensor) {
      collider.isOneWay = Boolean(options.isOneWay);
    }

    if (collider.bodyId) {
      this.wakeBody(collider.bodyId);
    }

    this.markCollisionStateDirty();
    return this.getCollider(collider.id);
  }

  configureColliderOneWay(colliderId, enabled) {
    return this.configureColliderCollision(colliderId, {
      isOneWay: enabled
    });
  }

  configureBodyCollision(bodyId, options = {}) {
    const body = this.bodyRegistry.getMutable(bodyId);
    if (!body) {
      return null;
    }

    for (const colliderId of body.colliderIds) {
      this.configureColliderCollision(colliderId, options);
    }

    return this.getBody(body.id);
  }

  getKinematicCapsule(characterId) {
    return this.characterRegistry.get(characterId);
  }

  listKinematicCapsules() {
    return this.characterRegistry.list();
  }

  configureKinematicCapsule(characterId, options = {}) {
    const character = this.characterRegistry.getMutable(characterId);
    if (!character) {
      return null;
    }

    if (options.skinWidth !== undefined) {
      character.skinWidth = toNonNegativeNumber(options.skinWidth, character.skinWidth ?? 0.5);
    }

    if (options.groundProbeDistance !== undefined) {
      character.groundProbeDistance = toNonNegativeNumber(options.groundProbeDistance, character.groundProbeDistance ?? 4);
    }

    if (options.maxGroundAngleDegrees !== undefined) {
      character.maxGroundAngleDegrees = toNonNegativeNumber(options.maxGroundAngleDegrees, character.maxGroundAngleDegrees ?? 55);
    }

    if (options.enabled !== undefined) {
      character.enabled = Boolean(options.enabled);
    }

    this.updateKinematicGroundState(character.id);
    return this.getKinematicCapsule(character.id);
  }

  configureKinematicController(characterId, options = {}) {
    const character = this.characterRegistry.getMutable(characterId);
    if (!character) {
      return null;
    }

    if (options.jumpSpeed !== undefined) {
      character.jumpSpeed = toOptionalNumber(options.jumpSpeed, character.jumpSpeed ?? 8) ?? (character.jumpSpeed ?? 8);
    }

    if (options.gravityScale !== undefined) {
      character.gravityScale = toNonNegativeNumber(options.gravityScale, character.gravityScale ?? 1);
    }

    if (options.stepOffset !== undefined) {
      character.stepOffset = toNonNegativeNumber(options.stepOffset, character.stepOffset ?? 6);
    }

    if (options.groundSnapDistance !== undefined) {
      character.groundSnapDistance = toNonNegativeNumber(options.groundSnapDistance, character.groundSnapDistance ?? 2);
    }

    if (options.airControlFactor !== undefined) {
      character.airControlFactor = Math.max(0, Number.isFinite(Number(options.airControlFactor))
        ? Number(options.airControlFactor)
        : (character.airControlFactor ?? 1));
    }

    if (options.coyoteTimeSeconds !== undefined) {
      character.coyoteTimeSeconds = toNonNegativeNumber(options.coyoteTimeSeconds, character.coyoteTimeSeconds ?? 0.1);
      character.coyoteTimer = Math.min(
        toNonNegativeNumber(character.coyoteTimer, 0),
        character.coyoteTimeSeconds
      );
    }

    if (options.jumpBufferSeconds !== undefined) {
      character.jumpBufferSeconds = toNonNegativeNumber(options.jumpBufferSeconds, character.jumpBufferSeconds ?? 0.1);
      character.jumpBufferTimer = Math.min(
        toNonNegativeNumber(character.jumpBufferTimer, 0),
        character.jumpBufferSeconds
      );
    }

    if (options.rideMovingPlatforms !== undefined) {
      character.rideMovingPlatforms = Boolean(options.rideMovingPlatforms);
    }

    return this.getKinematicCapsule(character.id);
  }

  setKinematicCapsuleMoveIntent(characterId, moveIntent = createVec3()) {
    const character = this.characterRegistry.getMutable(characterId);
    if (!character) {
      return null;
    }

    character.moveIntent = cloneVec3(moveIntent);
    return this.getKinematicCapsule(character.id);
  }

  jumpKinematicCapsule(characterId) {
    const character = this.characterRegistry.getMutable(characterId);
    if (!character) {
      return null;
    }

    character.jumpRequested = true;
    return this.getKinematicCapsule(character.id);
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
      worldAnchor: options.localAnchorA !== undefined
        ? undefined
        : (options.worldAnchorA ?? sharedWorldAnchor)
    });
    const localAnchorB = this.resolveBodyLocalAnchor(bodyBId, {
      localAnchor: options.localAnchorB,
      worldAnchor: options.localAnchorB !== undefined
        ? undefined
        : (options.worldAnchorB ?? sharedWorldAnchor)
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
      worldAnchor: options.localAnchorA !== undefined
        ? undefined
        : (options.worldAnchorA ?? sharedWorldAnchor)
    });
    const localAnchorB = this.resolveBodyLocalAnchor(bodyBId, {
      localAnchor: options.localAnchorB,
      worldAnchor: options.localAnchorB !== undefined
        ? undefined
        : (options.worldAnchorB ?? sharedWorldAnchor)
    });
    const localAxisA = this.resolveBodyLocalAxis(bodyAId, {
      localAxis: options.localAxisA,
      worldAxis: options.localAxisA !== undefined
        ? undefined
        : (options.worldAxisA ?? sharedWorldAxis)
    });
    const localAxisB = this.resolveBodyLocalAxis(bodyBId, {
      localAxis: options.localAxisB,
      worldAxis: options.localAxisB !== undefined
        ? undefined
        : (options.worldAxisB ?? sharedWorldAxis)
    });
    const tangentBasis = createTangentBasis(sharedWorldAxis);
    const localReferenceA = this.resolveBodyLocalAxis(bodyAId, {
      localAxis: options.localReferenceA,
      worldAxis: options.localReferenceA !== undefined
        ? undefined
        : (options.worldReferenceA ?? tangentBasis.tangentA)
    });
    const localReferenceB = this.resolveBodyLocalAxis(bodyBId, {
      localAxis: options.localReferenceB,
      worldAxis: options.localReferenceB !== undefined
        ? undefined
        : (options.worldReferenceB ?? tangentBasis.tangentA)
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
      worldAnchor: options.localAnchorA !== undefined
        ? undefined
        : (options.worldAnchorA ?? sharedWorldAnchor)
    });
    const localAnchorB = this.resolveBodyLocalAnchor(bodyBId, {
      localAnchor: options.localAnchorB,
      worldAnchor: options.localAnchorB !== undefined
        ? undefined
        : (options.worldAnchorB ?? sharedWorldAnchor)
    });
    const localAxisA = this.resolveBodyLocalAxis(bodyAId, {
      localAxis: options.localAxisA,
      worldAxis: options.localAxisA !== undefined
        ? undefined
        : (options.worldAxisA ?? sharedWorldAxis)
    });
    const localAxisB = this.resolveBodyLocalAxis(bodyBId, {
      localAxis: options.localAxisB,
      worldAxis: options.localAxisB !== undefined
        ? undefined
        : (options.worldAxisB ?? sharedWorldAxis)
    });
    const localReferenceA = this.resolveBodyLocalAxis(bodyAId, {
      localAxis: options.localReferenceA,
      worldAxis: options.localReferenceA !== undefined
        ? undefined
        : (options.worldReferenceA ?? sharedWorldReference)
    });
    const localReferenceB = this.resolveBodyLocalAxis(bodyBId, {
      localAxis: options.localReferenceB,
      worldAxis: options.localReferenceB !== undefined
        ? undefined
        : (options.worldReferenceB ?? sharedWorldReference)
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
      isSensor: options.isSensor,
      collisionLayer: options.collisionLayer,
      collisionMask: options.collisionMask,
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
      isSensor: options.isSensor,
      collisionLayer: options.collisionLayer,
      collisionMask: options.collisionMask,
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
      isSensor: options.isSensor,
      collisionLayer: options.collisionLayer,
      collisionMask: options.collisionMask,
      userData: options.colliderUserData ?? null
    });

    return {
      shape,
      body: this.getBody(body.id),
      collider
    };
  }

  createKinematicCapsule(options = {}) {
    const resolvedId = String(options.id ?? '').trim() || null;
    const radius = toPositiveNumber(options.radius, 12);
    const halfHeight = toPositiveNumber(options.halfHeight, 24);
    const { shape, body, collider } = this.createCapsuleBody({
      id: resolvedId,
      motionType: 'kinematic',
      position: options.position ?? createVec3(),
      rotation: options.rotation ?? createIdentityQuat(),
      radius,
      halfHeight,
      materialId: options.materialId,
      collisionLayer: options.collisionLayer,
      collisionMask: options.collisionMask,
      canSleep: false,
      bodyUserData: options.bodyUserData ?? null,
      shapeUserData: options.shapeUserData ?? null,
      colliderUserData: options.colliderUserData ?? null
    });

    const character = this.characterRegistry.createKinematicCapsule({
      id: resolvedId ?? body.id,
      bodyId: body.id,
      colliderId: collider.id,
      shapeId: shape.id,
      radius,
      halfHeight,
      skinWidth: options.skinWidth,
      groundProbeDistance: options.groundProbeDistance,
      maxGroundAngleDegrees: options.maxGroundAngleDegrees,
      gravityScale: options.gravityScale,
      jumpSpeed: options.jumpSpeed,
      stepOffset: options.stepOffset,
      groundSnapDistance: options.groundSnapDistance,
      airControlFactor: options.airControlFactor,
      coyoteTimeSeconds: options.coyoteTimeSeconds,
      jumpBufferSeconds: options.jumpBufferSeconds,
      rideMovingPlatforms: options.rideMovingPlatforms,
      coyoteTimer: options.coyoteTimer,
      jumpBufferTimer: options.jumpBufferTimer,
      moveIntent: options.moveIntent,
      verticalVelocity: options.verticalVelocity,
      jumpRequested: options.jumpRequested,
      enabled: options.enabled,
      userData: options.userData ?? null
    });

    this.updateKinematicGroundState(character.id);
    return this.getKinematicCapsule(character.id);
  }

  buildKinematicMoveResult(characterId, options = {}) {
    return {
      characterId: String(characterId ?? '').trim() || null,
      startPosition: cloneVec3(options.startPosition ?? createVec3()),
      endPosition: cloneVec3(options.endPosition ?? createVec3()),
      requestedMove: cloneVec3(options.requestedMove ?? createVec3()),
      actualMove: cloneVec3(options.actualMove ?? createVec3()),
      blocked: options.blocked === true,
      hitColliderId: String(options.hitColliderId ?? '').trim() || null,
      hitBodyId: String(options.hitBodyId ?? '').trim() || null,
      hitNormal: cloneVec3(options.hitNormal ?? createVec3()),
      hitDistance: options.hitDistance ?? null,
      hitPoint: cloneOptionalVec3(options.hitPoint ?? null),
      hitFeatureId: String(options.hitFeatureId ?? '').trim() || null,
      hitAlgorithm: String(options.hitAlgorithm ?? '').trim() || null
    };
  }

  getCachedStaticKinematicFace(cache) {
    const colliderId = String(cache?.colliderId ?? '').trim();
    const faceId = String(cache?.faceId ?? '').trim();
    if (!colliderId || !faceId) {
      return null;
    }

    const collider = this.getCollider(colliderId);
    if (!collider || collider.isSensor) {
      return null;
    }

    const shape = this.getShape(collider.shapeId);
    if (!shape || (shape.type !== 'box' && shape.type !== 'convex-hull')) {
      return null;
    }

    const worldPose = this.getColliderWorldPose(collider.id);
    if (!worldPose) {
      return null;
    }

    const faces = getStaticPolygonalWorldFaces(shape, worldPose);
    const face = findWorldFaceById(faces, faceId);
    if (!face) {
      return null;
    }

    return {
      colliderId: collider.id,
      bodyId: collider.bodyId ?? null,
      shapeId: collider.shapeId,
      materialId: collider.materialId ?? null,
      isOneWay: collider.isOneWay === true,
      shapeType: shape.type,
      face
    };
  }

  evaluateKinematicGroundFaceHit(character, body, faceContext, maxDistance, options = {}) {
    if (!faceContext?.face) {
      return null;
    }

    const queryDistance = Math.max(0, Number(maxDistance ?? 0));
    const radius = Math.max(0, Number(character?.radius ?? 0));
    const origin = cloneVec3(options.origin ?? body.position);
    const bottomSphereCenter = createKinematicBottomSphereCenter(character, origin);
    const face = faceContext.face;
    if (isOneWayPlatformCollider(faceContext) && !isUpwardOneWayFace(face)) {
      return null;
    }

    if (!isWalkableKinematicNormal(face.normal, character.maxGroundAngleDegrees)) {
      return null;
    }

    const verticalSupport = Number(face.normal?.y ?? 0);
    if (verticalSupport <= 1e-6) {
      return null;
    }

    const signedDistance = dotVec3(bottomSphereCenter, face.normal) - face.planeOffset;
    if (signedDistance < radius - character.skinWidth - 1e-3) {
      return null;
    }

    const distance = Math.max(0, (signedDistance - radius) / verticalSupport);
    if (distance > queryDistance + character.skinWidth + 1e-4) {
      return null;
    }

    const sphereCenterAtHit = addVec3(bottomSphereCenter, createVec3(0, -distance, 0));
    const contactOffset = signedDistance <= radius
      ? Math.max(0, signedDistance)
      : radius;
    const contactPoint = subtractVec3(sphereCenterAtHit, scaleVec3(face.normal, contactOffset));
    if (!isPointInsideWorldFace(contactPoint, face, Math.max(character.skinWidth, 0.15))) {
      return null;
    }

    return createShapeCastResult({
      castType: 'capsule',
      hit: true,
      origin,
      direction: createVec3(0, -1, 0),
      maxDistance: queryDistance,
      radius,
      halfHeight: character.halfHeight,
      rotation: cloneQuat(body.rotation ?? createIdentityQuat()),
      sampleOrigins: [cloneVec3(origin)],
      sampleEndPoints: [addVec3(origin, createVec3(0, -queryDistance, 0))],
      distance,
      point: contactPoint,
      normal: face.normal,
      sweepPosition: addVec3(origin, createVec3(0, -distance, 0)),
      colliderId: faceContext.colliderId,
      bodyId: faceContext.bodyId,
      shapeId: faceContext.shapeId,
      materialId: faceContext.materialId,
      shapeType: faceContext.shapeType,
      algorithm: 'character-ground-face-v1',
      featureId: face.id,
      proxyCount: options.proxyCount ?? 0,
      candidateCount: options.candidateCount ?? 1,
      testedShapeCount: options.testedShapeCount ?? 1
    });
  }

  evaluateKinematicMotionFaceHit(character, body, shape, faceContext, direction, maxDistance, options = {}) {
    if (!faceContext?.face) {
      return null;
    }

    const queryDistance = Math.max(0, Number(maxDistance ?? 0));
    const queryDirection = normalizeVec3(direction ?? createVec3(1, 0, 0), createVec3(1, 0, 0));
    const rotation = cloneQuat(options.rotation ?? body.rotation ?? createIdentityQuat());
    const origin = cloneVec3(options.origin ?? body.position);
    const face = faceContext.face;
    if (shouldIgnoreKinematicOneWayMotionFace(faceContext, queryDirection)) {
      return createShapeCastMiss({
        castType: 'capsule',
        origin,
        direction: queryDirection,
        maxDistance: queryDistance,
        radius: character.radius,
        halfHeight: character.halfHeight,
        rotation,
        sampleOrigins: [cloneVec3(origin)],
        sampleEndPoints: [addScaledVec3(origin, queryDirection, queryDistance)],
        proxyCount: options.proxyCount ?? 0,
        candidateCount: options.candidateCount ?? 1,
        testedShapeCount: options.testedShapeCount ?? 1
      });
    }

    const approachSpeed = -dotVec3(queryDirection, face.normal);
    if (approachSpeed <= KINEMATIC_MOTION_FACE_APPROACH_EPSILON) {
      return null;
    }

    const supportFeature = getShapeSupportFeature(shape, {
      position: origin,
      rotation
    }, scaleVec3(face.normal, -1));
    const supportPoint = cloneVec3(supportFeature.worldPoint);
    const signedDistance = dotVec3(supportPoint, face.normal) - face.planeOffset;
    if (signedDistance <= KINEMATIC_MOTION_FACE_INSIDE_TOLERANCE) {
      return null;
    }

    const planeDistance = signedDistance / approachSpeed;
    if (planeDistance < 0 || planeDistance > queryDistance + Math.max(character.skinWidth, 0.25)) {
      return null;
    }

    const contactPoint = addScaledVec3(supportPoint, queryDirection, planeDistance);
    if (!isPointInsideWorldFace(contactPoint, face, Math.max(character.skinWidth, 0.1))) {
      const boundaryDistanceSquared = getPointDistanceSquaredToWorldFaceBoundary(contactPoint, face);
      const edgeFallbackDistance = Math.max(character.radius + character.skinWidth, 0.75);
      if (boundaryDistanceSquared <= edgeFallbackDistance * edgeFallbackDistance) {
        return null;
      }
      return createShapeCastMiss({
        castType: 'capsule',
        origin,
        direction: queryDirection,
        maxDistance: queryDistance,
        radius: character.radius,
        halfHeight: character.halfHeight,
        rotation,
        sampleOrigins: [cloneVec3(origin)],
        sampleEndPoints: [addScaledVec3(origin, queryDirection, queryDistance)],
        proxyCount: options.proxyCount ?? 0,
        candidateCount: options.candidateCount ?? 1,
        testedShapeCount: options.testedShapeCount ?? 1
      });
    }

    return createShapeCastResult({
      castType: 'capsule',
      hit: true,
      origin,
      direction: queryDirection,
      maxDistance: queryDistance,
      radius: character.radius,
      halfHeight: character.halfHeight,
      rotation,
      sampleOrigins: [cloneVec3(origin)],
      sampleEndPoints: [addScaledVec3(origin, queryDirection, queryDistance)],
      distance: planeDistance,
      point: contactPoint,
      normal: face.normal,
      sweepPosition: addScaledVec3(origin, queryDirection, planeDistance),
      colliderId: faceContext.colliderId,
      bodyId: faceContext.bodyId,
      shapeId: faceContext.shapeId,
      materialId: faceContext.materialId,
      shapeType: faceContext.shapeType,
      algorithm: 'character-motion-face-v1',
      featureId: face.id,
      proxyCount: options.proxyCount ?? 0,
      candidateCount: options.candidateCount ?? 1,
      testedShapeCount: options.testedShapeCount ?? 1
    });
  }

  queryFastStaticKinematicGround(character, body, maxDistance, options = {}) {
    const broadphaseProxies = Array.isArray(options.broadphaseProxies)
      ? options.broadphaseProxies
      : this.ensureCollisionState().broadphaseProxies;
    const queryDistance = Math.max(0, Number(maxDistance ?? 0));
    const radius = Math.max(0, Number(character?.radius ?? 0));
    const origin = cloneVec3(options.origin ?? body.position);
    const bottomSphereCenter = createKinematicBottomSphereCenter(character, origin);
    const sweptAabb = createSweptBottomSphereAabb(bottomSphereCenter, radius, queryDistance);
    const prioritizedColliderIds = [
      character?.groundFaceCache?.colliderId,
      character?.motionFaceCache?.colliderId
    ];
    const cachedGroundContext = character?.groundFaceCache?.stableFrames >= 2
      ? this.getCachedStaticKinematicFace(character.groundFaceCache)
      : null;
    if (cachedGroundContext) {
      const cachedGroundHit = this.evaluateKinematicGroundFaceHit(character, body, cachedGroundContext, maxDistance, {
        origin: options.origin,
        proxyCount: broadphaseProxies.length,
        candidateCount: 1,
        testedShapeCount: 1
      });
      if (cachedGroundHit) {
        return cachedGroundHit;
      }
    }
    const candidateReuse = getReusableKinematicCandidateProxies(
      character,
      'groundCandidateCache',
      sweptAabb,
      broadphaseProxies,
      prioritizedColliderIds
    );
    const prioritizedProxies = candidateReuse
      ? candidateReuse.proxies
      : getPrioritizedStaticProxyList(broadphaseProxies, prioritizedColliderIds);
    const excludeBodyId = String(options.excludeBodyId ?? body.id ?? '').trim() || null;
    const excludeColliderIds = new Set(Array.isArray(options.excludeColliderIds) ? options.excludeColliderIds.map((id) => String(id ?? '').trim()) : []);
    const up = createVec3(0, 1, 0);
    let nearestHit = null;
    let candidateCount = 0;
    let testedShapeCount = 0;
    let fallbackRequired = false;
    let hasNonStaticPolygonalCandidate = false;

    for (const proxy of prioritizedProxies) {
      if (excludeColliderIds.has(proxy.colliderId)) {
        continue;
      }

      if (excludeBodyId && proxy.bodyId === excludeBodyId) {
        continue;
      }

      if (proxy.isSensor) {
        continue;
      }

      if (!testAabbOverlap(sweptAabb, proxy.aabb)) {
        continue;
      }

      candidateCount += 1;

      if (proxy.shapeType !== 'box' && proxy.shapeType !== 'convex-hull') {
        fallbackRequired = true;
        continue;
      }

      if (proxy.motionType !== 'static') {
        hasNonStaticPolygonalCandidate = true;
      }

      const shape = this.getShape(proxy.shapeId);
      const worldPose = this.getColliderWorldPose(proxy.colliderId);
      if (!shape || !worldPose) {
        fallbackRequired = true;
        continue;
      }

      testedShapeCount += 1;
      const faces = getStaticPolygonalWorldFaces(shape, worldPose);
      if (faces.length === 0) {
        fallbackRequired = true;
        continue;
      }

      const cachedFace = findWorldFaceById(
        faces,
        character?.groundFaceCache?.colliderId === proxy.colliderId
          ? character?.groundFaceCache?.faceId
          : null
      );
      const faceQueue = cachedFace
        ? [cachedFace, ...faces.filter((face) => face !== cachedFace)]
        : faces;

      let cachedFaceHit = null;
      for (const face of faceQueue) {
        if (proxy.isOneWay === true && !isUpwardOneWayFace(face)) {
          continue;
        }

        if (!isWalkableKinematicNormal(face.normal, character.maxGroundAngleDegrees)) {
          continue;
        }

        const verticalSupport = Number(face.normal?.y ?? 0);
        if (verticalSupport <= 1e-6) {
          continue;
        }

        const signedDistance = dotVec3(bottomSphereCenter, face.normal) - face.planeOffset;
        if (signedDistance < radius - character.skinWidth - 1e-3) {
          continue;
        }

        const distance = Math.max(0, (signedDistance - radius) / verticalSupport);
        if (distance > queryDistance + character.skinWidth + 1e-4) {
          continue;
        }

        const sphereCenterAtHit = addVec3(bottomSphereCenter, createVec3(0, -distance, 0));
        const contactPoint = subtractVec3(sphereCenterAtHit, scaleVec3(face.normal, radius));
        if (!isPointInsideWorldFace(contactPoint, face, Math.max(character.skinWidth, 0.15))) {
          continue;
        }

        const candidateHit = createShapeCastResult({
            castType: 'capsule',
            hit: true,
            origin,
            direction: createVec3(0, -1, 0),
            maxDistance: queryDistance,
            radius,
            halfHeight: character.halfHeight,
            rotation: cloneQuat(body.rotation ?? createIdentityQuat()),
            sampleOrigins: [cloneVec3(origin)],
            sampleEndPoints: [addVec3(origin, createVec3(0, -queryDistance, 0))],
            distance,
            point: contactPoint,
            normal: face.normal,
            sweepPosition: addVec3(origin, createVec3(0, -distance, 0)),
            colliderId: proxy.colliderId,
            bodyId: proxy.bodyId,
            shapeId: proxy.shapeId,
            materialId: proxy.materialId,
            shapeType: proxy.shapeType,
            algorithm: 'character-ground-face-v1',
            featureId: face.id,
            proxyCount: broadphaseProxies.length,
            candidateCount,
            testedShapeCount
          });
        if (!nearestHit || distance < nearestHit.distance - 1e-8) {
          nearestHit = candidateHit;
        }
        if (cachedFace && face === cachedFace) {
          cachedFaceHit = candidateHit;
          break;
        }
      }

      if (cachedFaceHit) {
        continue;
      }
    }

    updateKinematicCandidateCache(character, 'groundCandidateCache', sweptAabb, broadphaseProxies, {
      padding: Math.max(character.skinWidth + 0.5, radius * 0.25, 1)
    });

    if (nearestHit) {
      return nearestHit;
    }

    if (fallbackRequired) {
      return null;
    }

    if (candidateCount > 0 && hasNonStaticPolygonalCandidate) {
      return createShapeCastMiss({
        castType: 'capsule',
        origin,
        direction: createVec3(0, -1, 0),
        maxDistance: queryDistance,
        radius,
        halfHeight: character.halfHeight,
        rotation: cloneQuat(body.rotation ?? createIdentityQuat()),
        sampleOrigins: [cloneVec3(origin)],
        sampleEndPoints: [addVec3(origin, createVec3(0, -queryDistance, 0))],
        proxyCount: broadphaseProxies.length,
        candidateCount,
        testedShapeCount
      });
    }

    return null;
  }

  queryFastStaticKinematicMotion(character, body, shape, direction, maxDistance, options = {}) {
    const broadphaseProxies = Array.isArray(options.broadphaseProxies)
      ? options.broadphaseProxies
      : this.ensureCollisionState().broadphaseProxies;
    const queryDistance = Math.max(0, Number(maxDistance ?? 0));
    const queryDirection = normalizeVec3(direction ?? createVec3(1, 0, 0), createVec3(1, 0, 0));
    const rotation = cloneQuat(options.rotation ?? body.rotation ?? createIdentityQuat());
    const origin = cloneVec3(options.origin ?? body.position);
    const sweptAabb = computeSweptShapeAabb({
      shape,
      origin,
      direction: queryDirection,
      maxDistance: queryDistance,
      rotation
    });
    const prioritizedColliderIds = [
      character?.motionFaceCache?.colliderId,
      character?.groundFaceCache?.colliderId
    ];
    const cachedMotionContexts = [
      character?.motionFaceCache?.stableFrames >= 2 ? this.getCachedStaticKinematicFace(character.motionFaceCache) : null,
      character?.groundFaceCache?.stableFrames >= 2 ? this.getCachedStaticKinematicFace(character.groundFaceCache) : null
    ].filter(Boolean);
    for (const cachedMotionContext of cachedMotionContexts) {
      if (shouldIgnoreKinematicOneWayMotionFace(cachedMotionContext, direction)) {
        continue;
      }

      const cachedMotionHit = this.evaluateKinematicMotionFaceHit(character, body, shape, cachedMotionContext, direction, maxDistance, {
        origin: options.origin,
        rotation: options.rotation,
        proxyCount: broadphaseProxies.length,
        candidateCount: 1,
        testedShapeCount: 1
      });
      if (cachedMotionHit?.hit) {
        return cachedMotionHit;
      }
    }
    const candidateReuse = getReusableKinematicCandidateProxies(
      character,
      'motionCandidateCache',
      sweptAabb,
      broadphaseProxies,
      prioritizedColliderIds
    );
    const prioritizedProxies = candidateReuse
      ? candidateReuse.proxies
      : getPrioritizedStaticProxyList(broadphaseProxies, prioritizedColliderIds);
    const excludeBodyId = String(options.excludeBodyId ?? body.id ?? '').trim() || null;
    const excludeColliderIds = new Set(Array.isArray(options.excludeColliderIds) ? options.excludeColliderIds.map((id) => String(id ?? '').trim()) : []);
    const ignoreSensors = options.ignoreSensors !== false;
    let nearestHit = null;
    let candidateCount = 0;
    let testedShapeCount = 0;
    let fallbackRequired = false;
    let nearestPlaneDistance = Number.POSITIVE_INFINITY;

    for (const proxy of prioritizedProxies) {
      if (excludeColliderIds.has(proxy.colliderId)) {
        continue;
      }

      if (excludeBodyId && proxy.bodyId === excludeBodyId) {
        continue;
      }

      if (ignoreSensors && proxy.isSensor) {
        continue;
      }

      if (proxy.motionType !== 'static' || (proxy.shapeType !== 'box' && proxy.shapeType !== 'convex-hull')) {
        fallbackRequired = true;
        continue;
      }

      if (sweptAabb && !testAabbOverlap(sweptAabb, proxy.aabb)) {
        continue;
      }

      candidateCount += 1;

      const targetShape = this.getShape(proxy.shapeId);
      const targetPose = this.getColliderWorldPose(proxy.colliderId);
      if (!targetShape || !targetPose) {
        fallbackRequired = true;
        continue;
      }

      testedShapeCount += 1;
      const faces = getStaticPolygonalWorldFaces(targetShape, targetPose);
      if (faces.length === 0) {
        fallbackRequired = true;
        continue;
      }

      const cachedMotionFace = findWorldFaceById(
        faces,
        character?.motionFaceCache?.colliderId === proxy.colliderId
          ? character?.motionFaceCache?.faceId
          : null
      );
      const cachedGroundFace = !cachedMotionFace
        ? findWorldFaceById(
          faces,
          character?.groundFaceCache?.colliderId === proxy.colliderId
            ? character?.groundFaceCache?.faceId
            : null
        )
        : null;
      const cachedFaces = [cachedMotionFace, cachedGroundFace].filter(Boolean);
      const faceQueue = cachedFaces.length > 0
        ? cachedFaces.concat(faces.filter((face) => !cachedFaces.includes(face)))
        : faces;

      for (const face of faceQueue) {
        const faceContext = {
          colliderId: proxy.colliderId,
          bodyId: proxy.bodyId,
          shapeId: proxy.shapeId,
          materialId: proxy.materialId,
          isOneWay: proxy.isOneWay === true,
          shapeType: proxy.shapeType,
          face
        };
        if (shouldIgnoreKinematicOneWayMotionFace(faceContext, queryDirection)) {
          continue;
        }

        const evaluated = this.evaluateKinematicMotionFaceHit(character, body, shape, {
          ...faceContext
        }, queryDirection, queryDistance, {
          origin,
          rotation,
          proxyCount: broadphaseProxies.length,
          candidateCount,
          testedShapeCount
        });

        if (!evaluated) {
          const supportFeature = getShapeSupportFeature(shape, {
            position: origin,
            rotation
          }, scaleVec3(face.normal, -1));
          const supportPoint = cloneVec3(supportFeature.worldPoint);
          const signedDistance = dotVec3(supportPoint, face.normal) - face.planeOffset;
          if (signedDistance > KINEMATIC_MOTION_FACE_INSIDE_TOLERANCE) {
            const approachSpeed = -dotVec3(queryDirection, face.normal);
            if (approachSpeed > KINEMATIC_MOTION_FACE_APPROACH_EPSILON) {
              const planeDistance = signedDistance / approachSpeed;
              if (Number.isFinite(planeDistance)) {
                nearestPlaneDistance = Math.min(nearestPlaneDistance, planeDistance);
              }
            }
          }
          const projectedPoint = supportFeature?.worldPoint
            ? addScaledVec3(supportPoint, queryDirection, Math.max(0, signedDistance / Math.max(1e-8, -dotVec3(queryDirection, face.normal))))
            : null;
          if (projectedPoint) {
            const boundaryDistanceSquared = getPointDistanceSquaredToWorldFaceBoundary(projectedPoint, face);
            const edgeFallbackDistance = Math.max(character.radius + character.skinWidth, 0.75);
            if (boundaryDistanceSquared <= edgeFallbackDistance * edgeFallbackDistance) {
              fallbackRequired = true;
            }
          }
          continue;
        }

        if (!evaluated.hit) {
          const boundaryDistanceSquared = getPointDistanceSquaredToWorldFaceBoundary(evaluated.endPoint ?? origin, face);
          const edgeFallbackDistance = Math.max(character.radius + character.skinWidth, 0.75);
          if (boundaryDistanceSquared <= edgeFallbackDistance * edgeFallbackDistance) {
            fallbackRequired = true;
          }
          continue;
        }

        if (Number.isFinite(evaluated.distance)) {
          nearestPlaneDistance = Math.min(nearestPlaneDistance, evaluated.distance);
        }
        if (!nearestHit || evaluated.distance < nearestHit.distance - 1e-8) {
          nearestHit = evaluated;
        }
        if (cachedFaces.includes(face) && evaluated.hit) {
          nearestHit = evaluated;
          break;
        }
      }
    }

    updateKinematicCandidateCache(character, 'motionCandidateCache', sweptAabb, broadphaseProxies, {
      padding: Math.max(character.skinWidth + 0.5, queryDistance, Math.max(character.radius * 0.25, 1))
    });

    if (nearestHit) {
      return nearestHit;
    }

    if (fallbackRequired) {
      return null;
    }

    const canShortCircuitMotionMiss = candidateReuse &&
      candidateReuse.proxies.length > 0 &&
      candidateReuse.proxies.every((proxy) => proxy.shapeType === 'convex-hull');

    if (canShortCircuitMotionMiss && !hasKinematicFallbackBroadphaseOverlap(sweptAabb, broadphaseProxies, {
      excludeBodyId,
      excludeColliderIds: Array.from(excludeColliderIds),
      ignoreSensors
    })) {
      return createShapeCastMiss({
        castType: 'capsule',
        origin,
        direction: queryDirection,
        maxDistance: queryDistance,
        radius: character.radius,
        halfHeight: character.halfHeight,
        rotation,
        sampleOrigins: [cloneVec3(origin)],
        sampleEndPoints: [addScaledVec3(origin, queryDirection, queryDistance)],
        proxyCount: broadphaseProxies.length,
        candidateCount,
        testedShapeCount
      });
    }

    if (!Number.isFinite(nearestPlaneDistance) || nearestPlaneDistance > queryDistance + Math.max(character.radius, character.skinWidth, 0.5)) {
      if (candidateReuse && !canShortCircuitMotionMiss) {
        return null;
      }

      return createShapeCastMiss({
        castType: 'capsule',
        origin,
        direction: queryDirection,
        maxDistance: queryDistance,
        radius: character.radius,
        halfHeight: character.halfHeight,
        rotation,
        sampleOrigins: [cloneVec3(origin)],
        sampleEndPoints: [addScaledVec3(origin, queryDirection, queryDistance)],
        proxyCount: broadphaseProxies.length,
        candidateCount,
        testedShapeCount
      });
    }

    return null;
  }

  recoverKinematicStaticOverlaps(character, body, shape, options = {}) {
    const broadphaseProxies = Array.isArray(options.broadphaseProxies)
      ? options.broadphaseProxies
      : this.ensureCollisionState().broadphaseProxies;
    const prioritizedProxies = getPrioritizedStaticProxyList(broadphaseProxies, [
      character?.groundFaceCache?.colliderId,
      character?.motionFaceCache?.colliderId
    ]);
    const rotation = cloneQuat(options.rotation ?? body.rotation ?? createIdentityQuat());
    const maxIterations = Math.max(1, Math.floor(toPositiveNumber(options.maxIterations, 2)));
    const recoveryPadding = Math.max(1e-4, toNonNegativeNumber(options.padding, 0.02));
    const maxRecoveryDepth = Math.max(
      recoveryPadding,
      toNonNegativeNumber(
        options.maxRecoveryDepth,
        Math.max(character.skinWidth * 2, character.groundSnapDistance + character.skinWidth + 0.5)
      )
    );
    let totalRecovery = createVec3();
    let recovered = false;
    let lastRecoveryHit = null;

    for (let iteration = 0; iteration < maxIterations; iteration += 1) {
      const origin = cloneVec3(body.position);
      const overlapAabb = computeShapeWorldAabb(shape, {
        position: origin,
        rotation
      });
      let bestRecovery = null;

      for (const proxy of prioritizedProxies) {
        if (proxy.bodyId === body.id || proxy.isSensor || proxy.motionType !== 'static' || (proxy.shapeType !== 'box' && proxy.shapeType !== 'convex-hull')) {
          continue;
        }

        if (overlapAabb && !testAabbOverlap(overlapAabb, proxy.aabb)) {
          continue;
        }

        const targetShape = this.getShape(proxy.shapeId);
        const targetPose = this.getColliderWorldPose(proxy.colliderId);
        if (!targetShape || !targetPose) {
          continue;
        }

        const faces = getStaticPolygonalWorldFaces(targetShape, targetPose);
        if (faces.length === 0) {
          continue;
        }
        const isOneWay = proxy.isOneWay === true;

        const cachedFaces = [
          findWorldFaceById(faces, character?.groundFaceCache?.colliderId === proxy.colliderId ? character?.groundFaceCache?.faceId : null),
          findWorldFaceById(faces, character?.motionFaceCache?.colliderId === proxy.colliderId ? character?.motionFaceCache?.faceId : null)
        ].filter(Boolean);
        const faceQueue = cachedFaces.length > 0
          ? cachedFaces.concat(faces.filter((face) => !cachedFaces.includes(face)))
          : faces;

        for (const face of faceQueue) {
          if (isOneWay && !isUpwardOneWayFace(face)) {
            continue;
          }

          const supportFeature = getShapeSupportFeature(shape, {
            position: origin,
            rotation
          }, scaleVec3(face.normal, -1));
          const supportPoint = cloneVec3(supportFeature.worldPoint);
          const signedDistance = dotVec3(supportPoint, face.normal) - face.planeOffset;
          if (isOneWay && signedDistance < -Math.max(character.skinWidth + recoveryPadding, KINEMATIC_ONE_WAY_RECOVERY_DEPTH)) {
            continue;
          }

          if (signedDistance >= -1e-4 || signedDistance < -maxRecoveryDepth) {
            continue;
          }

          const projectedPoint = subtractVec3(supportPoint, scaleVec3(face.normal, signedDistance));
          const insideFace = isPointInsideWorldFace(projectedPoint, face, Math.max(character.skinWidth, 0.1));
          const boundaryDistanceSquared = insideFace ? 0 : getPointDistanceSquaredToWorldFaceBoundary(projectedPoint, face);
          const recoveryBoundaryDistance = Math.max(character.radius + character.skinWidth, 1);
          if (!insideFace && boundaryDistanceSquared > recoveryBoundaryDistance * recoveryBoundaryDistance) {
            continue;
          }

          const recoveryDistance = -signedDistance + recoveryPadding;
          if (!bestRecovery || recoveryDistance < bestRecovery.distance - 1e-8) {
            bestRecovery = {
              distance: recoveryDistance,
              normal: cloneVec3(face.normal),
              point: projectedPoint,
              colliderId: proxy.colliderId,
              bodyId: proxy.bodyId ?? null,
              faceId: face.id
            };
          }
        }
      }

      if (!bestRecovery || bestRecovery.distance <= 1e-6) {
        break;
      }

      const recoveryMove = scaleVec3(bestRecovery.normal, bestRecovery.distance);
      body.position = addVec3(body.position, recoveryMove);
      totalRecovery = addVec3(totalRecovery, recoveryMove);
      recovered = true;
      lastRecoveryHit = bestRecovery;
      updateKinematicFaceCache(character, 'motionFaceCache', {
        hit: true,
        colliderId: bestRecovery.colliderId,
        bodyId: bestRecovery.bodyId,
        featureId: bestRecovery.faceId,
        normal: bestRecovery.normal,
        point: bestRecovery.point,
        distance: 0
      });
      if (isWalkableKinematicNormal(bestRecovery.normal, character.maxGroundAngleDegrees)) {
        updateKinematicFaceCache(character, 'groundFaceCache', {
          hit: true,
          colliderId: bestRecovery.colliderId,
          bodyId: bestRecovery.bodyId,
          featureId: bestRecovery.faceId,
          normal: bestRecovery.normal,
          point: bestRecovery.point,
          distance: 0
        });
      }
    }

    character.lastRecoveryNormal = recovered && lastRecoveryHit
      ? cloneVec3(lastRecoveryHit.normal)
      : createVec3();
    character.lastRecoveryDistance = recovered ? Math.sqrt(lengthSquaredVec3(totalRecovery)) : 0;

    if (recovered) {
      this.markCollisionStateDirty();
    }

    return {
      recovered,
      move: totalRecovery,
      colliderId: lastRecoveryHit?.colliderId ?? null,
      bodyId: lastRecoveryHit?.bodyId ?? null,
      faceId: lastRecoveryHit?.faceId ?? null,
      distance: character.lastRecoveryDistance
    };
  }

  characterShapeCastAgainstWorld(character, body, shape, options = {}) {
    const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
    const maxDistance = toNonNegativeNumber(options.maxDistance, 0);
    const isGroundQuery = options.queryMode === 'ground' &&
      Math.abs(direction.x) <= 1e-6 &&
      Math.abs(direction.z) <= 1e-6 &&
      direction.y < -0.999;

    if (isGroundQuery) {
      const fastGroundHit = this.queryFastStaticKinematicGround(character, body, maxDistance, options);
      if (fastGroundHit) {
        if (options.storeResult !== false) {
          this.lastShapeCast = cloneShapeCastResult(fastGroundHit);
        }
        return fastGroundHit;
      }
    }

    if (options.queryMode === 'motion') {
      const fastMotionHit = this.queryFastStaticKinematicMotion(character, body, shape, direction, maxDistance, options);
      if (fastMotionHit) {
        if (options.storeResult !== false) {
          this.lastShapeCast = cloneShapeCastResult(fastMotionHit);
        }
        return fastMotionHit;
      }
    }

    const ignoredColliderIds = new Set(Array.isArray(options.excludeColliderIds) ? options.excludeColliderIds.map((id) => String(id ?? '').trim()).filter(Boolean) : []);
    const maxIgnoreAttempts = 8;

    for (let attempt = 0; attempt < maxIgnoreAttempts; attempt += 1) {
      const result = this.shapeCastAgainstWorld({
        ...options,
        castType: 'capsule',
        queryShape: shape,
        radius: character.radius,
        halfHeight: character.halfHeight,
        rotation: cloneQuat(options.rotation ?? body.rotation ?? createIdentityQuat()),
        excludeColliderIds: Array.from(ignoredColliderIds),
        distanceTolerance: options.distanceTolerance ?? Math.max(KINEMATIC_CHARACTER_CAST_DISTANCE_TOLERANCE, toNonNegativeNumber(character.skinWidth, 0.5) * 0.25),
        maxDepth: options.maxDepth ?? KINEMATIC_CHARACTER_CAST_MAX_DEPTH
      });

      if (!result.hit) {
        return result;
      }

      const hitCollider = result.colliderId ? this.getCollider(result.colliderId) : null;
      if (!shouldIgnoreKinematicOneWayCastHit(hitCollider, result, direction)) {
        return result;
      }

      if (!result.colliderId || ignoredColliderIds.has(result.colliderId)) {
        return result;
      }

      ignoredColliderIds.add(result.colliderId);
    }

    return this.shapeCastAgainstWorld({
      ...options,
      castType: 'capsule',
      queryShape: shape,
      radius: character.radius,
      halfHeight: character.halfHeight,
      rotation: cloneQuat(options.rotation ?? body.rotation ?? createIdentityQuat()),
      excludeColliderIds: Array.from(ignoredColliderIds),
      distanceTolerance: options.distanceTolerance ?? Math.max(KINEMATIC_CHARACTER_CAST_DISTANCE_TOLERANCE, toNonNegativeNumber(character.skinWidth, 0.5) * 0.25),
      maxDepth: options.maxDepth ?? KINEMATIC_CHARACTER_CAST_MAX_DEPTH
    });
  }

  traceKinematicCapsuleMotion(character, body, shape, requestedMove = createVec3(), options = {}) {
    const startPosition = cloneVec3(options.startPosition ?? body.position);
    const requestedMoveVector = cloneVec3(requestedMove ?? createVec3());
    const actualMove = createVec3();
    let currentPosition = cloneVec3(startPosition);
    let remainingMove = cloneVec3(requestedMoveVector);
    let hitColliderId = null;
    let hitBodyId = null;
    let hitDistance = null;
    let hitNormal = createVec3();
    let hitPoint = null;
    let hitFeatureId = null;
    let hitAlgorithm = null;
    let blocked = false;
    const maxPasses = Math.max(1, Math.floor(toPositiveNumber(options.maxPasses, 2)));
    const maxHitAttempts = Math.max(1, Math.floor(toPositiveNumber(options.maxHitAttempts, 6)));

    for (let passIndex = 0; passIndex < maxPasses; passIndex += 1) {
      const remainingLengthSquared = lengthSquaredVec3(remainingMove);
      if (remainingLengthSquared <= 1e-8) {
        break;
      }

      const remainingLength = Math.sqrt(remainingLengthSquared);
      const direction = scaleVec3(remainingMove, 1 / remainingLength);
      const ignoredColliderIds = new Set([
        ...(Array.isArray(body.colliderIds) ? body.colliderIds : []),
        ...(Array.isArray(options.excludeColliderIds) ? options.excludeColliderIds : [])
      ]);
      let hit = { hit: false };
      for (let hitAttempt = 0; hitAttempt < maxHitAttempts; hitAttempt += 1) {
        const candidateHit = this.characterShapeCastAgainstWorld(character, body, shape, {
          origin: currentPosition,
          direction,
          maxDistance: remainingLength,
          rotation: body.rotation ?? createIdentityQuat(),
          excludeBodyId: options.excludeBodyId ?? body.id,
          excludeColliderIds: Array.from(ignoredColliderIds),
          ignoreDynamicTargets: options.ignoreDynamicTargets === true,
          ignoreSensors: options.ignoreSensors !== false,
          storeResult: false,
          queryMode: Math.abs(direction.x) <= 1e-6 && Math.abs(direction.z) <= 1e-6 && direction.y < -0.999
            ? 'ground'
            : 'motion'
        });

        if (!candidateHit.hit) {
          hit = candidateHit;
          break;
        }

        if (isBlockingKinematicHit(direction, candidateHit.normal)) {
          hit = candidateHit;
          break;
        }

        if (!candidateHit.colliderId) {
          hit = candidateHit;
          break;
        }

        ignoredColliderIds.add(candidateHit.colliderId);
      }

      if (!hit.hit) {
        currentPosition = addVec3(currentPosition, remainingMove);
        actualMove.x += remainingMove.x;
        actualMove.y += remainingMove.y;
        actualMove.z += remainingMove.z;
        remainingMove = createVec3();
        break;
      }

      blocked = true;
      hitColliderId = hit.colliderId ?? hitColliderId;
      hitBodyId = hit.bodyId ?? hitBodyId;
      hitDistance = hit.distance ?? hitDistance;
      hitNormal = cloneVec3(hit.normal ?? hitNormal);
      hitPoint = cloneOptionalVec3(hit.point ?? hitPoint);
      hitFeatureId = String(hit.featureId ?? '').trim() || hitFeatureId;
      hitAlgorithm = String(hit.algorithm ?? '').trim() || hitAlgorithm;

      const safeDistance = Math.max(0, Math.min(remainingLength, Number(hit.distance ?? 0) - character.skinWidth));
      if (safeDistance > 1e-8) {
        const movedSegment = scaleVec3(direction, safeDistance);
        currentPosition = addVec3(currentPosition, movedSegment);
        actualMove.x += movedSegment.x;
        actualMove.y += movedSegment.y;
        actualMove.z += movedSegment.z;
      }

      const leftoverDistance = Math.max(0, remainingLength - safeDistance);
      if (leftoverDistance <= 1e-8 || !hit.normal) {
        remainingMove = createVec3();
        break;
      }

      const leftoverMove = scaleVec3(direction, leftoverDistance);
      const slideMove = subtractVec3(leftoverMove, scaleVec3(hit.normal, dotVec3(leftoverMove, hit.normal)));
      if (lengthSquaredVec3(slideMove) <= 1e-8) {
        remainingMove = createVec3();
        break;
      }

      remainingMove = slideMove;
    }

    const moveResult = this.buildKinematicMoveResult(character.id, {
      startPosition,
      endPosition: currentPosition,
      requestedMove: requestedMoveVector,
      actualMove,
      blocked,
      hitColliderId,
      hitBodyId,
      hitNormal,
      hitDistance,
      hitPoint,
      hitFeatureId,
      hitAlgorithm
    });

    if (moveResult.blocked && moveResult.hitAlgorithm === 'character-motion-face-v1' && moveResult.hitFeatureId) {
      updateKinematicFaceCache(character, 'motionFaceCache', {
        hit: true,
        colliderId: moveResult.hitColliderId,
        bodyId: moveResult.hitBodyId,
        featureId: moveResult.hitFeatureId,
        normal: moveResult.hitNormal,
        point: moveResult.hitPoint ?? createVec3(),
        distance: moveResult.hitDistance
      });
    } else if (!moveResult.blocked || moveResult.hitAlgorithm !== 'character-motion-face-v1') {
      clearKinematicFaceCache(character, 'motionFaceCache');
    }

    return moveResult;
  }

  applyKinematicMoveResult(character, body, moveResult, options = {}) {
    body.position = cloneVec3(moveResult?.endPosition ?? body.position);
    zeroBodyMotion(body);

    character.lastRequestedMove = cloneVec3(moveResult?.requestedMove ?? createVec3());
    character.lastActualMove = cloneVec3(moveResult?.actualMove ?? createVec3());
    character.lastHitColliderId = moveResult?.hitColliderId ?? null;
    character.lastHitBodyId = moveResult?.hitBodyId ?? null;
    character.lastHitDistance = moveResult?.hitDistance ?? null;
    character.lastHitNormal = cloneVec3(moveResult?.hitNormal ?? createVec3());

    this.markCollisionStateDirty();
    if (options.updateGroundState !== false) {
      this.updateKinematicGroundState(character.id);
    }

    return this.buildKinematicMoveResult(character.id, {
      startPosition: moveResult?.startPosition,
      endPosition: moveResult?.endPosition,
      requestedMove: moveResult?.requestedMove,
      actualMove: moveResult?.actualMove,
      blocked: moveResult?.blocked,
      hitColliderId: moveResult?.hitColliderId,
      hitBodyId: moveResult?.hitBodyId,
      hitNormal: moveResult?.hitNormal,
      hitDistance: moveResult?.hitDistance,
      hitPoint: moveResult?.hitPoint,
      hitFeatureId: moveResult?.hitFeatureId,
      hitAlgorithm: moveResult?.hitAlgorithm
    });
  }

  applyKinematicPlatformCarry(character, body, shape, deltaTime) {
    if (!character || !body || !shape) {
      return null;
    }

    character.lastPlatformCarry = createVec3();
    character.lastPlatformBodyId = null;
    character.platformVelocity = createVec3();

    if (
      character.rideMovingPlatforms === false ||
      character.grounded !== true ||
      character.walkable !== true ||
      !character.groundBodyId ||
      character.groundBodyLocalPoint?.valid !== true
    ) {
      return null;
    }

    const groundBody = this.bodyRegistry.getMutable(character.groundBodyId);
    if (!groundBody || groundBody.enabled === false) {
      return null;
    }

    const localPoint = cloneVec3(character.groundBodyLocalPoint.point ?? createVec3());
    const currentGroundPoint = addVec3(
      groundBody.position,
      rotateVec3ByQuat(groundBody.rotation ?? createIdentityQuat(), localPoint)
    );
    const previousGroundPoint = cloneVec3(character.groundPoint ?? createVec3());
    const platformCarry = subtractVec3(currentGroundPoint, previousGroundPoint);

    if (lengthSquaredVec3(platformCarry) <= 1e-10) {
      return null;
    }

    const carryResult = this.traceKinematicCapsuleMotion(character, body, shape, platformCarry, {
      startPosition: body.position,
      maxPasses: 1
    });

    body.position = cloneVec3(carryResult.endPosition);
    character.lastPlatformCarry = cloneVec3(carryResult.actualMove ?? createVec3());
    character.lastPlatformBodyId = groundBody.id;
    if (deltaTime > 1e-8) {
      character.platformVelocity = scaleVec3(character.lastPlatformCarry, 1 / deltaTime);
    }
    return {
      requestedMove: platformCarry,
      moveResult: carryResult
    };
  }

  tryKinematicCapsuleStepMove(character, body, shape, horizontalMove = createVec3()) {
    const stepOffset = toNonNegativeNumber(character.stepOffset, 0);
    if (stepOffset <= 1e-8 || lengthSquaredVec3(horizontalMove) <= 1e-8) {
      return null;
    }

    const startPosition = cloneVec3(body.position);
    const upResult = this.traceKinematicCapsuleMotion(character, body, shape, createVec3(0, stepOffset, 0), {
      startPosition,
      maxPasses: 1
    });
    const liftedPosition = cloneVec3(upResult.endPosition);
    if (liftedPosition.y <= startPosition.y + 1e-6) {
      return null;
    }

    const horizontalResult = this.traceKinematicCapsuleMotion(character, body, shape, horizontalMove, {
      startPosition: liftedPosition
    });
    if (getForwardProgress(horizontalMove, horizontalResult.actualMove) <= 1e-4) {
      return null;
    }

    const downDistance = stepOffset + toNonNegativeNumber(character.groundSnapDistance, 2) + character.skinWidth;
    const downResult = this.traceKinematicCapsuleMotion(character, body, shape, createVec3(0, -downDistance, 0), {
      startPosition: horizontalResult.endPosition,
      maxPasses: 1
    });

    if (!downResult.blocked || !isWalkableKinematicNormal(downResult.hitNormal, character.maxGroundAngleDegrees)) {
      return null;
    }

    return this.buildKinematicMoveResult(character.id, {
      startPosition,
      endPosition: downResult.endPosition,
      requestedMove: addVec3(createVec3(0, stepOffset, 0), addVec3(horizontalMove, createVec3(0, -downDistance, 0))),
      actualMove: subtractVec3(downResult.endPosition, startPosition),
      blocked: horizontalResult.blocked || downResult.blocked,
      hitColliderId: horizontalResult.hitColliderId ?? downResult.hitColliderId,
      hitBodyId: horizontalResult.hitBodyId ?? downResult.hitBodyId,
      hitNormal: downResult.hitNormal ?? horizontalResult.hitNormal,
      hitDistance: downResult.hitDistance ?? horizontalResult.hitDistance
    });
  }

  moveKinematicCapsule(characterId, motion = createVec3()) {
    const character = this.characterRegistry.getMutable(characterId);
    if (!character || character.enabled === false) {
      return null;
    }

    const body = this.bodyRegistry.getMutable(character.bodyId);
    const shape = this.getShape(character.shapeId);
    if (!body || !shape) {
      return null;
    }

    const moveResult = this.traceKinematicCapsuleMotion(character, body, shape, motion);
    return this.applyKinematicMoveResult(character, body, moveResult, {
      updateGroundState: true
    });
  }

  stepKinematicCapsules(deltaTime) {
    let movedAnyCharacter = false;
    this.characterRegistry.forEachMutable((character) => {
      if (!character || character.enabled === false) {
        return;
      }

      const body = this.bodyRegistry.getMutable(character.bodyId);
      const shape = this.getShape(character.shapeId);
      if (!body || !shape) {
        return;
      }

      const startPosition = cloneVec3(body.position);
      let blocked = false;
      let lastHitColliderId = null;
      let lastHitBodyId = null;
      let lastHitDistance = null;
      let lastHitNormal = createVec3();
      const totalRequestedMove = createVec3();
      const totalActualMove = createVec3();

      const initialRecovery = this.recoverKinematicStaticOverlaps(character, body, shape, {
        maxIterations: 2,
        padding: 0.02
      });
      let totalRecoveryDistance = 0;
      let lastRecoveryNormal = createVec3();
      if (initialRecovery.recovered) {
        totalRecoveryDistance += initialRecovery.distance;
        lastRecoveryNormal = cloneVec3(character.lastRecoveryNormal ?? createVec3());
      }

      const platformCarry = this.applyKinematicPlatformCarry(character, body, shape, deltaTime);
      if (platformCarry) {
        totalRequestedMove.x += platformCarry.requestedMove.x;
        totalRequestedMove.y += platformCarry.requestedMove.y;
        totalRequestedMove.z += platformCarry.requestedMove.z;
        totalActualMove.x += platformCarry.moveResult.actualMove.x;
        totalActualMove.y += platformCarry.moveResult.actualMove.y;
        totalActualMove.z += platformCarry.moveResult.actualMove.z;
        blocked = blocked || platformCarry.moveResult.blocked;
        if (platformCarry.moveResult.hitColliderId) {
          lastHitColliderId = platformCarry.moveResult.hitColliderId;
          lastHitBodyId = platformCarry.moveResult.hitBodyId;
          lastHitDistance = platformCarry.moveResult.hitDistance;
          lastHitNormal = cloneVec3(platformCarry.moveResult.hitNormal);
        }
      }

      this.updateKinematicGroundState(character.id);
      const wasGrounded = character.grounded === true && character.walkable === true;
      const supportVelocity = cloneVec3(character.platformVelocity ?? createVec3());
      let verticalVelocity = toOptionalNumber(character.verticalVelocity, 0) ?? 0;
      if (wasGrounded) {
        character.coyoteTimer = toNonNegativeNumber(character.coyoteTimeSeconds, 0.1);
      } else {
        character.coyoteTimer = Math.max(0, toNonNegativeNumber(character.coyoteTimer, 0) - deltaTime);
      }

      if (character.jumpRequested === true) {
        character.jumpBufferTimer = Math.max(
          toNonNegativeNumber(character.jumpBufferTimer, 0),
          toNonNegativeNumber(character.jumpBufferSeconds, 0.1),
          deltaTime
        );
      }
      character.jumpRequested = false;

      const wantsJump = toNonNegativeNumber(character.jumpBufferTimer, 0) > 1e-8;
      const canGroundJump = wasGrounded || toNonNegativeNumber(character.coyoteTimer, 0) > 1e-8;
      let jumpedThisFrame = false;
      if (wantsJump && canGroundJump) {
        verticalVelocity = (toOptionalNumber(character.jumpSpeed, 8) ?? 8) + Number(supportVelocity.y ?? 0);
        character.jumpBufferTimer = 0;
        character.coyoteTimer = 0;
        character.grounded = false;
        character.walkable = false;
        character.inheritedVelocity = createVec3(Number(supportVelocity.x ?? 0), 0, Number(supportVelocity.z ?? 0));
        jumpedThisFrame = true;
      } else {
        character.jumpBufferTimer = Math.max(0, toNonNegativeNumber(character.jumpBufferTimer, 0) - deltaTime);
      }

      const moveIntent = cloneVec3(character.moveIntent ?? createVec3());
      let horizontalMove = scaleVec3(createVec3(moveIntent.x, 0, moveIntent.z), deltaTime);
      if (wasGrounded && character.walkable && !jumpedThisFrame && lengthSquaredVec3(horizontalMove) > 1e-8) {
        horizontalMove = projectOntoGroundPlane(horizontalMove, character.groundNormal ?? createVec3(0, 1, 0));
      } else if ((!wasGrounded || jumpedThisFrame) && lengthSquaredVec3(horizontalMove) > 1e-8) {
        horizontalMove = scaleVec3(horizontalMove, Math.max(0, toNonNegativeNumber(character.airControlFactor, 1)));
      }
      if (!wasGrounded || jumpedThisFrame) {
        horizontalMove = addVec3(
          horizontalMove,
          scaleVec3(createVec3(
            Number(character.inheritedVelocity?.x ?? 0),
            0,
            Number(character.inheritedVelocity?.z ?? 0)
          ), deltaTime)
        );
      }

      if (!wasGrounded || verticalVelocity > 0) {
        verticalVelocity += Number(this.gravity.y ?? -9.81) * toNonNegativeNumber(character.gravityScale, 1) * deltaTime;
      } else if (verticalVelocity < 0) {
        verticalVelocity = 0;
      }

      if (verticalVelocity > 1e-8) {
        const upwardMove = createVec3(0, verticalVelocity * deltaTime, 0);
        const upwardResult = this.traceKinematicCapsuleMotion(character, body, shape, upwardMove, {
          startPosition: body.position,
          maxPasses: 1
        });
        body.position = cloneVec3(upwardResult.endPosition);
        totalRequestedMove.x += upwardMove.x;
        totalRequestedMove.y += upwardMove.y;
        totalRequestedMove.z += upwardMove.z;
        totalActualMove.x += upwardResult.actualMove.x;
        totalActualMove.y += upwardResult.actualMove.y;
        totalActualMove.z += upwardResult.actualMove.z;
        blocked = blocked || upwardResult.blocked;
        if (upwardResult.hitColliderId) {
          lastHitColliderId = upwardResult.hitColliderId;
          lastHitBodyId = upwardResult.hitBodyId;
          lastHitDistance = upwardResult.hitDistance;
          lastHitNormal = cloneVec3(upwardResult.hitNormal);
        }
        if (upwardResult.blocked && (upwardResult.hitNormal?.y ?? 0) < -0.2) {
          verticalVelocity = 0;
        }
      }

      if (lengthSquaredVec3(horizontalMove) > 1e-8) {
        let horizontalResult = this.traceKinematicCapsuleMotion(character, body, shape, horizontalMove, {
          startPosition: body.position
        });
        const stepCandidate = wasGrounded && character.walkable && horizontalResult.blocked && (horizontalResult.hitNormal?.y ?? 1) < 0.35
          ? this.tryKinematicCapsuleStepMove(character, body, shape, horizontalMove)
          : null;
        const horizontalForwardProgress = getForwardProgress(horizontalMove, horizontalResult.actualMove);
        if (
          stepCandidate &&
          (
            getForwardProgress(horizontalMove, stepCandidate.actualMove) > horizontalForwardProgress + 1e-4 ||
            (
              getForwardProgress(horizontalMove, stepCandidate.actualMove) >= horizontalForwardProgress - 1e-4 &&
              stepCandidate.endPosition.y > horizontalResult.endPosition.y + Math.max(character.skinWidth, 0.25)
            )
          )
        ) {
          horizontalResult = stepCandidate;
        }

        body.position = cloneVec3(horizontalResult.endPosition);
        totalRequestedMove.x += horizontalMove.x;
        totalRequestedMove.y += horizontalMove.y;
        totalRequestedMove.z += horizontalMove.z;
        totalActualMove.x += horizontalResult.actualMove.x;
        totalActualMove.y += horizontalResult.actualMove.y;
        totalActualMove.z += horizontalResult.actualMove.z;
        blocked = blocked || horizontalResult.blocked;
        if (horizontalResult.hitColliderId) {
          lastHitColliderId = horizontalResult.hitColliderId;
          lastHitBodyId = horizontalResult.hitBodyId;
          lastHitDistance = horizontalResult.hitDistance;
          lastHitNormal = cloneVec3(horizontalResult.hitNormal);
        }
      }

      const shouldSnapToGround = verticalVelocity <= 0;
      const downwardDistance = Math.max(0, -verticalVelocity * deltaTime) + (shouldSnapToGround ? toNonNegativeNumber(character.groundSnapDistance, 2) : 0);
      if (downwardDistance > 1e-8) {
        const downResult = this.traceKinematicCapsuleMotion(character, body, shape, createVec3(0, -downwardDistance, 0), {
          startPosition: body.position,
          maxPasses: 1
        });
        body.position = cloneVec3(downResult.endPosition);
        totalRequestedMove.y -= downwardDistance;
        totalActualMove.x += downResult.actualMove.x;
        totalActualMove.y += downResult.actualMove.y;
        totalActualMove.z += downResult.actualMove.z;
        blocked = blocked || downResult.blocked;
        if (downResult.hitColliderId) {
          lastHitColliderId = downResult.hitColliderId;
          lastHitBodyId = downResult.hitBodyId;
          lastHitDistance = downResult.hitDistance;
          lastHitNormal = cloneVec3(downResult.hitNormal);
        }
        if (downResult.blocked && isWalkableKinematicNormal(downResult.hitNormal, character.maxGroundAngleDegrees)) {
          verticalVelocity = 0;
        }
      }

      zeroBodyMotion(body);
      character.verticalVelocity = verticalVelocity;
      character.lastRequestedMove = cloneVec3(totalRequestedMove);
      character.lastActualMove = cloneVec3(totalActualMove);
      character.lastHitColliderId = lastHitColliderId;
      character.lastHitBodyId = lastHitBodyId;
      character.lastHitDistance = lastHitDistance;
      character.lastHitNormal = cloneVec3(lastHitNormal);

      const finalRecovery = this.recoverKinematicStaticOverlaps(character, body, shape, {
        maxIterations: 1,
        padding: 0.01
      });
      if (finalRecovery.recovered) {
        totalRecoveryDistance += finalRecovery.distance;
        lastRecoveryNormal = cloneVec3(character.lastRecoveryNormal ?? createVec3());
      }
      character.lastRecoveryDistance = totalRecoveryDistance;
      character.lastRecoveryNormal = totalRecoveryDistance > 0
        ? cloneVec3(lastRecoveryNormal)
        : createVec3();
      this.updateKinematicGroundState(character.id);
      if (verticalVelocity > 1e-6) {
        character.grounded = false;
        character.walkable = false;
      } else if (character.grounded === true && verticalVelocity <= 0) {
        character.verticalVelocity = 0;
        character.coyoteTimer = toNonNegativeNumber(character.coyoteTimeSeconds, 0.1);
      } else {
        character.coyoteTimer = Math.max(0, toNonNegativeNumber(character.coyoteTimer, 0));
      }

      if (character.grounded === true && character.walkable === true) {
        character.inheritedVelocity = createVec3();
      } else if (wasGrounded && !jumpedThisFrame) {
        character.inheritedVelocity = createVec3(Number(supportVelocity.x ?? 0), 0, Number(supportVelocity.z ?? 0));
        if (Math.abs(verticalVelocity) <= 1e-6) {
          character.verticalVelocity = Number(supportVelocity.y ?? 0);
        }
      }

      if (
        character.grounded === true &&
        character.walkable === true &&
        toNonNegativeNumber(character.jumpBufferTimer, 0) > 1e-8 &&
        character.verticalVelocity <= 0
      ) {
        character.verticalVelocity = toOptionalNumber(character.jumpSpeed, 8) ?? 8;
        character.jumpBufferTimer = 0;
        character.coyoteTimer = 0;
        character.grounded = false;
        character.walkable = false;
      }

      movedAnyCharacter = movedAnyCharacter ||
        lengthSquaredVec3(subtractVec3(body.position, startPosition)) > 1e-8 ||
        Math.abs(verticalVelocity) > 1e-8;
      if (blocked) {
        movedAnyCharacter = true;
      }
    });

    if (movedAnyCharacter) {
      this.markCollisionStateDirty();
    }

    return movedAnyCharacter;
  }

  updateKinematicGroundState(characterId) {
    const character = this.characterRegistry.getMutable(characterId);
    if (!character) {
      return null;
    }

    const body = this.bodyRegistry.getMutable(character.bodyId);
    const shape = this.getShape(character.shapeId);
    if (!body || !shape) {
      return this.getKinematicCapsule(character.id);
    }

    const up = createVec3(0, 1, 0);
    const groundProbeDistance = toNonNegativeNumber(character.groundProbeDistance, 4);
    const hit = this.characterShapeCastAgainstWorld(character, body, shape, {
      origin: body.position,
      direction: createVec3(0, -1, 0),
      maxDistance: groundProbeDistance + character.skinWidth + 1e-4,
      rotation: body.rotation ?? createIdentityQuat(),
      excludeBodyId: body.id,
      excludeColliderIds: body.colliderIds,
      ignoreDynamicTargets: false,
      ignoreSensors: true,
      storeResult: false,
      queryMode: 'ground'
    });

    const nextGroundState = createDefaultGroundState();
    if (hit.hit && hit.normal) {
      const normalizedNormal = normalizeVec3(hit.normal, up);
      const angleDegrees = radiansToDegrees(Math.acos(clampUnitInterval(dotVec3(normalizedNormal, up))));
      const walkable = angleDegrees <= character.maxGroundAngleDegrees + 1e-6;

      nextGroundState.grounded = walkable && Number(hit.distance ?? Number.POSITIVE_INFINITY) <= groundProbeDistance + character.skinWidth + 1e-4;
      nextGroundState.walkable = walkable;
      nextGroundState.distance = Number(hit.distance ?? 0);
      nextGroundState.angleDegrees = angleDegrees;
      nextGroundState.normal = normalizedNormal;
      nextGroundState.point = cloneVec3(hit.point ?? createVec3());
      nextGroundState.colliderId = hit.colliderId ?? null;
      nextGroundState.bodyId = hit.bodyId ?? null;
    }

    if ((toOptionalNumber(character.verticalVelocity, 0) ?? 0) > 1e-6) {
      nextGroundState.grounded = false;
      nextGroundState.walkable = false;
    }

    character.grounded = nextGroundState.grounded;
    character.walkable = nextGroundState.walkable;
    character.groundDistance = nextGroundState.distance;
    character.groundAngleDegrees = nextGroundState.angleDegrees;
    character.groundNormal = cloneVec3(nextGroundState.normal);
    character.groundPoint = cloneVec3(nextGroundState.point);
    character.groundColliderId = nextGroundState.colliderId;
    character.groundBodyId = nextGroundState.bodyId;
    if (nextGroundState.grounded && nextGroundState.bodyId) {
      const groundBody = this.bodyRegistry.get(nextGroundState.bodyId);
      if (groundBody) {
        const localGroundPoint = inverseRotateVec3ByQuat(
          groundBody.rotation ?? createIdentityQuat(),
          subtractVec3(nextGroundState.point, groundBody.position)
        );
        character.groundBodyLocalPoint = {
          valid: true,
          point: cloneVec3(localGroundPoint)
        };
      } else {
        character.groundBodyLocalPoint = {
          valid: false,
          point: createVec3()
        };
      }
    } else {
      character.groundBodyLocalPoint = {
        valid: false,
        point: createVec3()
      };
    }

    if (hit.hit && hit.algorithm === 'character-ground-face-v1' && hit.featureId) {
      updateKinematicFaceCache(character, 'groundFaceCache', hit);
    } else if (!nextGroundState.grounded) {
      clearKinematicFaceCache(character, 'groundFaceCache');
    }

    return this.getKinematicCapsule(character.id);
  }

  getKinematicGroundState(characterId) {
    const character = this.getKinematicCapsule(characterId);
    if (!character) {
      return null;
    }

    return {
      grounded: character.grounded === true,
      walkable: character.walkable === true,
      distance: character.groundDistance ?? null,
      angleDegrees: character.groundAngleDegrees ?? null,
      normal: cloneVec3(character.groundNormal ?? createVec3(0, 1, 0)),
      point: cloneVec3(character.groundPoint ?? createVec3()),
      colliderId: character.groundColliderId ?? null,
      bodyId: character.groundBodyId ?? null
    };
  }

  isKinematicCapsuleGrounded(characterId) {
    const character = this.getKinematicCapsule(characterId);
    return Boolean(character?.grounded);
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
      isSensor: options.isSensor,
      collisionLayer: options.collisionLayer,
      collisionMask: options.collisionMask,
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
      isSensor: options.isSensor,
      isOneWay: options.isOneWay,
      collisionLayer: options.collisionLayer,
      collisionMask: options.collisionMask,
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
      isSensor: options.isSensor,
      isOneWay: options.isOneWay,
      collisionLayer: options.collisionLayer,
      collisionMask: options.collisionMask,
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

  createStaticBoxSensor(options = {}) {
    return this.createStaticBoxCollider({
      ...options,
      isOneWay: false,
      isSensor: true
    });
  }

  createStaticConvexHullSensor(options = {}) {
    return this.createStaticConvexHullCollider({
      ...options,
      isOneWay: false,
      isSensor: true
    });
  }

  createClothSheet(options = {}) {
    return this.particleWorld.createClothSheet({
      id: options.id,
      rows: options.rows,
      columns: options.columns ?? options.cols,
      spacing: options.spacing,
      position: options.position ?? createVec3(),
      pinMode: options.pinMode,
      particleMass: options.particleMass ?? options.mass,
      damping: options.damping,
      collisionMargin: options.collisionMargin,
      stretchCompliance: options.stretchCompliance,
      shearCompliance: options.shearCompliance,
      bendCompliance: options.bendCompliance,
      selfCollisionEnabled: options.selfCollisionEnabled,
      selfCollisionDistance: options.selfCollisionDistance
    });
  }

  createSoftBodyCube(options = {}) {
    return this.particleWorld.createSoftBodyCube({
      id: options.id,
      rows: options.rows,
      columns: options.columns ?? options.cols,
      layers: options.layers ?? options.depth,
      spacing: options.spacing,
      position: options.position ?? createVec3(),
      pinMode: options.pinMode,
      particleMass: options.particleMass ?? options.mass,
      damping: options.damping,
      collisionMargin: options.collisionMargin,
      stretchCompliance: options.stretchCompliance,
      shearCompliance: options.shearCompliance,
      bendCompliance: options.bendCompliance,
      volumeCompliance: options.volumeCompliance
    });
  }

  configureCloth(id, options = {}) {
    return this.particleWorld.configureCloth(id, {
      damping: options.damping,
      collisionMargin: options.collisionMargin,
      stretchCompliance: options.stretchCompliance,
      shearCompliance: options.shearCompliance,
      bendCompliance: options.bendCompliance,
      selfCollisionEnabled: options.selfCollisionEnabled,
      selfCollisionDistance: options.selfCollisionDistance
    });
  }

  configureSoftBody(id, options = {}) {
    return this.particleWorld.configureSoftBody(id, {
      damping: options.damping,
      collisionMargin: options.collisionMargin,
      stretchCompliance: options.stretchCompliance,
      shearCompliance: options.shearCompliance,
      bendCompliance: options.bendCompliance,
      volumeCompliance: options.volumeCompliance
    });
  }

  getCloth(id) {
    return this.particleWorld.getCloth(id);
  }

  getSoftBody(id) {
    return this.particleWorld.getSoftBody(id);
  }

  listCloths() {
    return this.particleWorld.listCloths();
  }

  listSoftBodies() {
    return this.particleWorld.listSoftBodies();
  }

  listStaticClothColliders() {
    const colliders = [];

    for (const collider of this.colliderRegistry.listMutable()) {
      if (!collider?.enabled || collider.isSensor) {
        continue;
      }

      const body = collider.bodyId ? this.bodyRegistry.getMutable(collider.bodyId) : null;
      const isStatic = !body || body.motionType === 'static';
      if (!isStatic) {
        continue;
      }

      const shape = this.shapeRegistry.getMutable(collider.shapeId);
      if (!shape || (shape.type !== 'box' && shape.type !== 'convex-hull')) {
        continue;
      }

      const pose = this.getColliderWorldPose(collider.id);
      if (!pose) {
        continue;
      }

      colliders.push({
        colliderId: collider.id,
        bodyId: collider.bodyId ?? null,
        materialId: collider.materialId,
        material: this.getEffectiveMaterialForCollider(collider.id),
        shape,
        pose
      });
    }

    return colliders;
  }

  listDynamicClothColliders() {
    const colliders = [];

    for (const collider of this.colliderRegistry.listMutable()) {
      if (!collider?.enabled || collider.isSensor || !collider.bodyId) {
        continue;
      }

      const body = this.bodyRegistry.getMutable(collider.bodyId);
      if (!body || !body.enabled || body.motionType !== 'dynamic') {
        continue;
      }

      const shape = this.shapeRegistry.getMutable(collider.shapeId);
      if (!shape || (shape.type !== 'box' && shape.type !== 'convex-hull')) {
        continue;
      }

      colliders.push({
        colliderId: collider.id,
        bodyId: collider.bodyId,
        materialId: collider.materialId,
        material: this.getEffectiveMaterialForCollider(collider.id),
        shape,
        body,
        motionType: body.motionType,
        getPose: () => this.getColliderWorldPose(collider.id)
      });
    }

    return colliders;
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
      clothCount: this.particleWorld.countCloths(),
      softBodyCount: this.particleWorld.countSoftBodies(),
      particleCount: this.particleWorld.countParticles(),
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
    const staticClothColliders = this.listStaticClothColliders();

    while (this.accumulatorSeconds + 1e-12 >= this.fixedDeltaTime && performedSubsteps < this.maxSubsteps) {
      this.wakeBodiesFromExternalActivity();
      const ccdEvents = this.integrateRigidBodies(this.fixedDeltaTime);
      ccdEventCount += ccdEvents.length;
      const solverStats = this.solveRigidContacts(this.fixedDeltaTime, this.simulationTick + 1);
      this.stepKinematicCapsules(this.fixedDeltaTime);
      const dynamicClothColliders = this.listDynamicClothColliders();
      this.lastXpbdStats = this.particleWorld.step(this.fixedDeltaTime, {
        staticColliders: staticClothColliders,
        dynamicColliders: dynamicClothColliders
      });
      if ((this.lastXpbdStats.solvedDynamicCollisions ?? 0) > 0) {
        this.markCollisionStateDirty();
      }
      mergeSolverStats(aggregatedSolverStats, solverStats);
      this.accumulatorSeconds -= this.fixedDeltaTime;
      this.simulationTick += 1;
      performedSubsteps += 1;
    }

    if (performedSubsteps === this.maxSubsteps && this.accumulatorSeconds > this.fixedDeltaTime) {
      this.accumulatorSeconds = this.fixedDeltaTime;
    }

    if (this.characterRegistry.count() > 0) {
      this.refreshKinematicCapsules();
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

  refreshKinematicCapsules() {
    this.characterRegistry.forEachMutable((character) => {
      if (!character || character.enabled === false) {
        return;
      }

      this.updateKinematicGroundState(character.id);
    });
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
    const initialResults = this.buildCollisionResults({
      trackEvents: false
    });
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
      joints: this.jointRegistry.listMutable().filter((joint) => joint.enabled !== false),
      deltaTime,
      iterations: this.solverIterations,
      baumgarte: this.solverBaumgarte,
      allowedStretch: this.allowedPenetration
    });
    this.applyJointBreakThresholds(jointSolverStats.jointResults, deltaTime);
    mergeSolverStats(solverStats, jointSolverStats);

    this.lastSolverStats = solverStats;
    const finalResults = this.buildCollisionResults({
      trackEvents: true
    });
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
        isOneWay: mutableCollider.isOneWay,
        collisionLayer: mutableCollider.collisionLayer,
        collisionMask: mutableCollider.collisionMask,
        aabb
      }));
    });

    return broadphaseProxies;
  }

  shouldCullTransientContact(contactPair, contact) {
    const penetration = Number(contact?.penetration ?? contactPair?.penetration ?? 0);
    const transientPenetration = Math.max(Number(this.allowedPenetration ?? 0.01) * 0.6, 0.004);
    if (penetration > transientPenetration) {
      return false;
    }

    const bodyA = contactPair?.bodyAId ? this.getBody(contactPair.bodyAId) : null;
    const bodyB = contactPair?.bodyBId ? this.getBody(contactPair.bodyBId) : null;
    const relativeVelocity = getRelativeVelocityAtPoint(bodyA, bodyB, contact.position ?? createVec3());
    const relativeNormalVelocity = dotVec3(relativeVelocity, contactPair?.normal ?? createVec3(0, 1, 0));
    return relativeNormalVelocity > 0.05;
  }

  filterTransientContactPairs(contactPairs) {
    const filteredPairs = [];

    for (const contactPair of Array.isArray(contactPairs) ? contactPairs : []) {
      const sourceContacts = Array.isArray(contactPair.contacts) ? contactPair.contacts : [];
      const survivingContacts = sourceContacts.filter((contact) => !this.shouldCullTransientContact(contactPair, contact));
      const contacts = sourceContacts.length > 1
        ? (survivingContacts.length === 0 ? [] : sourceContacts)
        : survivingContacts;
      if (contacts.length === 0) {
        continue;
      }

      filteredPairs.push(cloneContactPair({
        ...contactPair,
        contactCount: contacts.length,
        contacts,
        penetration: Math.max(...contacts.map((contact) => Number(contact.penetration ?? 0)), Number(contactPair.penetration ?? 0))
      }));
    }

    return filteredPairs;
  }

  buildNonCollidingJointBodyPairSet() {
    const suppressedPairs = new Set();

    this.jointRegistry.forEachMutable((joint) => {
      if (!joint || joint.enabled === false || joint.collideConnected === true) {
        return;
      }

      const pairKey = createBodyPairKey(joint.bodyAId, joint.bodyBId);
      if (pairKey) {
        suppressedPairs.add(pairKey);
      }
    });

    return suppressedPairs;
  }

  shouldCollidersInteract(leftProxy, rightProxy) {
    if (!leftProxy || !rightProxy) {
      return false;
    }

    return shouldCollisionLayersInteract(
      leftProxy.collisionLayer,
      leftProxy.collisionMask,
      rightProxy.collisionLayer,
      rightProxy.collisionMask
    );
  }

  buildContactEventState(contactPairs, sensorPairs) {
    const previousState = this.lastContactEventState ?? createEmptyContactEventState();
    const contactState = buildPairEventState(contactPairs, previousState.contactPairsByKey, 'contact');
    const triggerState = buildPairEventState(sensorPairs, previousState.triggerPairsByKey, 'trigger');
    const nextState = {
      contactPairsByKey: contactState.nextPairsByKey,
      triggerPairsByKey: triggerState.nextPairsByKey,
      contactEvents: contactState.events,
      triggerEvents: triggerState.events
    };

    this.lastContactEventState = cloneContactEventState(nextState);
    return nextState;
  }

  buildCollisionResults(options = {}) {
    const trackEvents = options.trackEvents !== false;
    const broadphaseProxies = this.buildBroadphaseProxies();
    const broadphaseProxyByColliderId = new Map(broadphaseProxies.map((proxy) => [proxy.colliderId, proxy]));
    const suppressedJointPairs = this.buildNonCollidingJointBodyPairSet();
    const broadphasePairs = buildBroadphasePairs(broadphaseProxies).filter((pair) => {
      const pairKey = createBodyPairKey(pair.bodyAId, pair.bodyBId);
      if (pairKey && suppressedJointPairs.has(pairKey)) {
        return false;
      }

      return this.shouldCollidersInteract(
        broadphaseProxyByColliderId.get(pair.colliderAId),
        broadphaseProxyByColliderId.get(pair.colliderBId)
      );
    });
    const narrowphase = runNarrowphase(broadphasePairs, {
      getShape: (shapeId) => this.getShape(shapeId),
      getPose: (colliderId) => this.getColliderWorldPose(colliderId)
    });
    const resolvedContactPairs = narrowphase.contactPairs.map((contactPair) => {
      const materialA = this.getEffectiveMaterialForCollider(contactPair.colliderAId);
      const materialB = this.getEffectiveMaterialForCollider(contactPair.colliderBId);
      const materialProperties = combineMaterialProperties(materialA, materialB);

      return cloneContactPair({
        ...contactPair,
        ...materialProperties
      });
    });
    const sensorPairs = [];
    const solidPairs = [];

    for (const contactPair of resolvedContactPairs) {
      const proxyA = broadphaseProxyByColliderId.get(contactPair.colliderAId);
      const proxyB = broadphaseProxyByColliderId.get(contactPair.colliderBId);
      if (proxyA?.isSensor || proxyB?.isSensor) {
        sensorPairs.push(cloneContactPair({
          ...contactPair,
          status: 'sensor-overlap'
        }));
        continue;
      }

      solidPairs.push(contactPair);
    }

    const contactPairs = this.filterTransientContactPairs(solidPairs);
    const eventState = trackEvents
      ? this.buildContactEventState(contactPairs, sensorPairs)
      : {
        contactEvents: [],
        triggerEvents: []
      };

    return {
      broadphaseProxies,
      broadphasePairs,
      contactPairs,
      sensorPairs,
      contactEvents: eventState.contactEvents,
      triggerEvents: eventState.triggerEvents,
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
    const contactEvents = Array.isArray(results.contactEvents) ? results.contactEvents : [];
    const triggerEvents = Array.isArray(results.triggerEvents) ? results.triggerEvents : [];
    this.collisionState = {
      broadphaseProxies: results.broadphaseProxies,
      broadphasePairs: results.broadphasePairs,
      contactPairs: results.contactPairs,
      sensorPairs: Array.isArray(results.sensorPairs) ? results.sensorPairs.map((pair) => cloneContactPair(pair)) : [],
      manifolds: Array.isArray(manifolds) ? manifolds.map((manifold) => cloneManifold(manifold)) : [],
      islands: resolvedIslandState.islands,
      joints,
      contactEvents: contactEvents.map((event) => cloneContactEvent(event)),
      triggerEvents: triggerEvents.map((event) => cloneContactEvent(event)),
      solverStats: cloneSolverStats(solverStats ?? this.lastSolverStats),
      summary: {
        proxyCount: results.broadphaseProxies.length,
        pairCount: results.broadphasePairs.length,
        contactCount: results.contactPairs.length,
        sensorPairCount: Array.isArray(results.sensorPairs) ? results.sensorPairs.length : 0,
        manifoldCount: Array.isArray(manifolds) ? manifolds.length : 0,
        jointCount: joints.length,
        islandCount: resolvedIslandState.islandCount,
        sleepingBodyCount: resolvedIslandState.sleepingBodyCount,
        awakeBodyCount: resolvedIslandState.awakeBodyCount,
        contactEnterCount: contactEvents.filter((event) => event.phase === 'enter').length,
        contactStayCount: contactEvents.filter((event) => event.phase === 'stay').length,
        contactExitCount: contactEvents.filter((event) => event.phase === 'exit').length,
        triggerEnterCount: triggerEvents.filter((event) => event.phase === 'enter').length,
        triggerStayCount: triggerEvents.filter((event) => event.phase === 'stay').length,
        triggerExitCount: triggerEvents.filter((event) => event.phase === 'exit').length,
        unsupportedPairCount: results.unsupportedPairCount,
        pairKinds: { ...results.pairKinds },
        algorithms: { ...results.algorithms }
      }
    };
  }

  ensureCollisionState() {
    if (this.collisionStateDirty) {
      const results = this.buildCollisionResults({
        trackEvents: true
      });
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

  getContactPairsForBody(bodyId) {
    const resolvedBodyId = String(bodyId ?? '').trim();
    if (!resolvedBodyId) {
      return [];
    }

    return this.ensureCollisionState().contactPairs
      .filter((contactPair) => contactPair.bodyAId === resolvedBodyId || contactPair.bodyBId === resolvedBodyId)
      .map((contactPair) => cloneContactPair(contactPair));
  }

  getBodiesTouchingBody(bodyId) {
    const resolvedBodyId = String(bodyId ?? '').trim();
    const pairs = this.getContactPairsForBody(resolvedBodyId);
    const touchedBodyIds = new Set();

    for (const pair of pairs) {
      const otherBodyId = pair.bodyAId === resolvedBodyId ? pair.bodyBId : pair.bodyAId;
      if (otherBodyId) {
        touchedBodyIds.add(otherBodyId);
      }
    }

    return {
      bodyId: resolvedBodyId,
      pairs,
      bodies: Array.from(touchedBodyIds)
        .map((otherBodyId) => this.getBody(otherBodyId))
        .filter(Boolean),
      count: touchedBodyIds.size
    };
  }

  getContactPairsForCollider(colliderId) {
    const resolvedColliderId = String(colliderId ?? '').trim();
    if (!resolvedColliderId) {
      return [];
    }

    return this.ensureCollisionState().contactPairs
      .filter((contactPair) => contactPair.colliderAId === resolvedColliderId || contactPair.colliderBId === resolvedColliderId)
      .map((contactPair) => cloneContactPair(contactPair));
  }

  getCollidersTouchingCollider(colliderId) {
    const resolvedColliderId = String(colliderId ?? '').trim();
    const pairs = this.getContactPairsForCollider(resolvedColliderId);
    const touchedColliderIds = new Set();

    for (const pair of pairs) {
      const otherColliderId = pair.colliderAId === resolvedColliderId ? pair.colliderBId : pair.colliderAId;
      if (otherColliderId) {
        touchedColliderIds.add(otherColliderId);
      }
    }

    return {
      colliderId: resolvedColliderId,
      pairs,
      colliders: Array.from(touchedColliderIds)
        .map((otherColliderId) => this.getCollider(otherColliderId))
        .filter(Boolean),
      count: touchedColliderIds.size
    };
  }

  getContactEvents(phase = null) {
    const resolvedPhase = phase ? String(phase).trim().toLowerCase() : null;
    const events = Array.isArray(this.ensureCollisionState().contactEvents)
      ? this.ensureCollisionState().contactEvents
      : [];
    return events
      .filter((event) => !resolvedPhase || event.phase === resolvedPhase)
      .map((event) => cloneContactEvent(event));
  }

  getTriggerEvents(phase = null) {
    const resolvedPhase = phase ? String(phase).trim().toLowerCase() : null;
    const events = Array.isArray(this.ensureCollisionState().triggerEvents)
      ? this.ensureCollisionState().triggerEvents
      : [];
    return events
      .filter((event) => !resolvedPhase || event.phase === resolvedPhase)
      .map((event) => cloneContactEvent(event));
  }

  getBodyContactEvents(bodyId, phase = null) {
    const resolvedBodyId = String(bodyId ?? '').trim();
    const events = this.getContactEvents(phase).filter((event) => event.bodyAId === resolvedBodyId || event.bodyBId === resolvedBodyId);
    const touchedBodyIds = new Set();

    for (const event of events) {
      const otherBodyId = event.bodyAId === resolvedBodyId ? event.bodyBId : event.bodyAId;
      if (otherBodyId) {
        touchedBodyIds.add(otherBodyId);
      }
    }

    return {
      bodyId: resolvedBodyId,
      phase: phase ? String(phase).trim().toLowerCase() : null,
      events,
      bodies: Array.from(touchedBodyIds)
        .map((otherBodyId) => this.getBody(otherBodyId))
        .filter(Boolean),
      count: touchedBodyIds.size
    };
  }

  getBodyTriggerEvents(bodyId, phase = null) {
    const resolvedBodyId = String(bodyId ?? '').trim();
    const events = this.getTriggerEvents(phase).filter((event) => event.bodyAId === resolvedBodyId || event.bodyBId === resolvedBodyId);
    const touchedBodyIds = new Set();

    for (const event of events) {
      const otherBodyId = event.bodyAId === resolvedBodyId ? event.bodyBId : event.bodyAId;
      if (otherBodyId) {
        touchedBodyIds.add(otherBodyId);
      }
    }

    return {
      bodyId: resolvedBodyId,
      phase: phase ? String(phase).trim().toLowerCase() : null,
      events,
      bodies: Array.from(touchedBodyIds)
        .map((otherBodyId) => this.getBody(otherBodyId))
        .filter(Boolean),
      count: touchedBodyIds.size
    };
  }

  getColliderContactEvents(colliderId, phase = null) {
    const resolvedColliderId = String(colliderId ?? '').trim();
    const events = this.getContactEvents(phase).filter((event) => event.colliderAId === resolvedColliderId || event.colliderBId === resolvedColliderId);
    const touchedColliderIds = new Set();

    for (const event of events) {
      const otherColliderId = event.colliderAId === resolvedColliderId ? event.colliderBId : event.colliderAId;
      if (otherColliderId) {
        touchedColliderIds.add(otherColliderId);
      }
    }

    return {
      colliderId: resolvedColliderId,
      phase: phase ? String(phase).trim().toLowerCase() : null,
      events,
      colliders: Array.from(touchedColliderIds)
        .map((otherColliderId) => this.getCollider(otherColliderId))
        .filter(Boolean),
      count: touchedColliderIds.size
    };
  }

  getColliderTriggerEvents(colliderId, phase = null) {
    const resolvedColliderId = String(colliderId ?? '').trim();
    const events = this.getTriggerEvents(phase).filter((event) => event.colliderAId === resolvedColliderId || event.colliderBId === resolvedColliderId);
    const touchedColliderIds = new Set();

    for (const event of events) {
      const otherColliderId = event.colliderAId === resolvedColliderId ? event.colliderBId : event.colliderAId;
      if (otherColliderId) {
        touchedColliderIds.add(otherColliderId);
      }
    }

    return {
      colliderId: resolvedColliderId,
      phase: phase ? String(phase).trim().toLowerCase() : null,
      events,
      colliders: Array.from(touchedColliderIds)
        .map((otherColliderId) => this.getCollider(otherColliderId))
        .filter(Boolean),
      count: touchedColliderIds.size
    };
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
          maxDistance: remainingDistance,
          distanceTolerance: options.distanceTolerance,
          maxDepth: options.maxDepth
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
            rotation,
            distanceTolerance: options.distanceTolerance,
            maxDepth: options.maxDepth
          })
          : sphereCastShape({
            shape,
            worldPose,
            origin,
            direction,
            maxDistance: remainingDistance,
            radius,
            distanceTolerance: options.distanceTolerance,
            maxDepth: options.maxDepth
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
        materialId: collider.materialId,
        collisionLayer: collider.collisionLayer,
        collisionMask: collider.collisionMask,
        isSensor: collider.isSensor,
        isOneWay: collider.isOneWay === true
      };
      const wireColor = collider.isSensor
        ? DEFAULT_DEBUG_COLORS.sensorColliderWireframe
        : body
          ? (body.sleeping ? DEFAULT_DEBUG_COLORS.sleepingRigidBodyWireframe : DEFAULT_DEBUG_COLORS.rigidBodyWireframe)
          : collider.isOneWay === true
            ? DEFAULT_DEBUG_COLORS.oneWayPlatformWireframe
            : DEFAULT_DEBUG_COLORS.staticColliderWireframe;
      const wireCategory = collider.isSensor
        ? 'sensor-collider'
        : body
          ? 'rigid-body'
          : 'static-collider';

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
          category: collider.isSensor
            ? 'sensor-origin'
            : body
              ? 'center-of-mass'
              : 'collider-origin',
          position: body?.position ?? pose.position,
          color: collider.isSensor ? DEFAULT_DEBUG_COLORS.sensorOrigin : DEFAULT_DEBUG_COLORS.rigidBodyCenter,
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

    for (const character of this.characterRegistry.list()) {
      const body = character.bodyId ? this.getBody(character.bodyId) : null;
      if (!body) {
        continue;
      }

      const source = {
        characterId: character.id,
        bodyId: character.bodyId,
        colliderId: character.colliderId,
        grounded: character.grounded === true,
        walkable: character.walkable === true
      };

      primitives.push(
        createDebugPoint({
          id: `${character.id}:controller`,
          category: 'character-controller',
          position: body.position,
          color: DEFAULT_DEBUG_COLORS.characterController,
          size: 6,
          source
        })
      );

      if (lengthSquaredVec3(character.lastActualMove ?? createVec3()) > 1e-8) {
        primitives.push(
          createDebugLine({
            id: `${character.id}:move-path`,
            category: 'character-move-path',
            start: body.position,
            end: addVec3(body.position, character.lastActualMove),
            color: DEFAULT_DEBUG_COLORS.characterMovePath,
            source
          })
        );
      }

      if (character.groundColliderId && character.groundPoint && character.groundNormal) {
        primitives.push(
          createDebugPoint({
            id: `${character.id}:ground-point`,
            category: 'character-ground-point',
            position: character.groundPoint,
            color: DEFAULT_DEBUG_COLORS.characterGroundPoint,
            size: 5,
            source: {
              ...source,
              groundColliderId: character.groundColliderId,
              groundBodyId: character.groundBodyId
            }
          })
        );

        primitives.push(
          createDebugLine({
            id: `${character.id}:ground-normal`,
            category: 'character-ground-normal',
            start: character.groundPoint,
            end: addScaledVec3(character.groundPoint, character.groundNormal, Math.max(10, (character.groundProbeDistance ?? 4) * 4)),
            color: DEFAULT_DEBUG_COLORS.characterGroundNormal,
            source: {
              ...source,
              groundColliderId: character.groundColliderId,
              groundBodyId: character.groundBodyId,
              groundAngleDegrees: character.groundAngleDegrees
            }
          })
        );
      }
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

    for (const sensorPair of Array.isArray(collisionState.sensorPairs) ? collisionState.sensorPairs : []) {
      for (const contact of sensorPair.contacts) {
        const source = {
          pairKey: sensorPair.pairKey,
          colliderAId: sensorPair.colliderAId,
          colliderBId: sensorPair.colliderBId,
          bodyAId: sensorPair.bodyAId,
          bodyBId: sensorPair.bodyBId,
          isSensorPair: true
        };

        primitives.push(
          createDebugPoint({
            id: `${contact.id}:trigger-point`,
            category: 'trigger-contact-point',
            position: contact.position,
            color: DEFAULT_DEBUG_COLORS.triggerContactPoint,
            size: 5,
            source
          })
        );

        primitives.push(
          createDebugLine({
            id: `${contact.id}:trigger-normal`,
            category: 'trigger-contact-normal',
            start: contact.position,
            end: addScaledVec3(contact.position, sensorPair.normal, Math.max(10, Number(contact.penetration ?? sensorPair.penetration ?? 0) * 20)),
            color: DEFAULT_DEBUG_COLORS.triggerContactNormal,
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

    primitives.push(...this.particleWorld.buildDebugPrimitives());

    const xpbdSnapshot = this.particleWorld.getSnapshot();

    return createDebugFrame({
      frameNumber: this.renderFrameCount,
      simulationTick: this.simulationTick,
      camera: cloneCamera(this.debugCamera),
      primitives,
      stats: {
      bodyCount: this.bodyRegistry.count(),
      characterCount: this.characterRegistry.count(),
      groundedCharacterCount: this.characterRegistry.list().filter((character) => character.grounded === true).length,
      clothCount: xpbdSnapshot.clothCount,
      softBodyCount: xpbdSnapshot.softBodyCount ?? 0,
      particleCount: xpbdSnapshot.particleCount,
        shapeCount: this.shapeRegistry.count(),
        colliderCount: this.colliderRegistry.count(),
        jointCount: this.jointRegistry.count(),
        materialCount: this.materialRegistry.count(),
        broadphaseProxyCount: collisionState.summary.proxyCount,
        broadphasePairCount: collisionState.summary.pairCount,
        contactPairCount: collisionState.summary.contactCount,
        sensorPairCount: collisionState.summary.sensorPairCount,
        manifoldCount: collisionState.summary.manifoldCount,
        islandCount: collisionState.summary.islandCount,
        sleepingBodyCount: collisionState.summary.sleepingBodyCount,
        awakeBodyCount: collisionState.summary.awakeBodyCount,
        contactEnterCount: collisionState.summary.contactEnterCount,
        contactStayCount: collisionState.summary.contactStayCount,
        contactExitCount: collisionState.summary.contactExitCount,
        triggerEnterCount: collisionState.summary.triggerEnterCount,
        triggerStayCount: collisionState.summary.triggerStayCount,
        triggerExitCount: collisionState.summary.triggerExitCount,
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
        xpbdSolvedDistanceConstraints: xpbdSnapshot.lastStepStats.solvedDistanceConstraints,
        xpbdSolvedVolumeConstraints: xpbdSnapshot.lastStepStats.solvedVolumeConstraints,
        xpbdSolvedPinConstraints: xpbdSnapshot.lastStepStats.solvedPinConstraints,
        xpbdSolvedStaticCollisions: xpbdSnapshot.lastStepStats.solvedStaticCollisions,
        xpbdSolvedStaticFrictionCollisions: xpbdSnapshot.lastStepStats.solvedStaticFrictionCollisions,
        xpbdSolvedDynamicCollisions: xpbdSnapshot.lastStepStats.solvedDynamicCollisions,
        xpbdSolvedDynamicFrictionCollisions: xpbdSnapshot.lastStepStats.solvedDynamicFrictionCollisions,
        xpbdSolvedSelfCollisions: xpbdSnapshot.lastStepStats.solvedSelfCollisions,
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
      characterCount: this.characterRegistry.count(),
      clothCount: this.particleWorld.countCloths(),
      softBodyCount: this.particleWorld.countSoftBodies(),
      particleCount: this.particleWorld.countParticles(),
      shapeCount: this.shapeRegistry.count(),
      colliderCount: this.colliderRegistry.count(),
      jointCount: this.jointRegistry.count(),
      materialCount: this.materialRegistry.count(),
      bodies: this.bodyRegistry.list(),
      characters: this.characterRegistry.list(),
      shapes: this.shapeRegistry.list(),
      colliders: this.colliderRegistry.list(),
      joints: this.jointRegistry.list(),
      materials: this.materialRegistry.list(),
      cloths: this.particleWorld.listCloths(),
      softBodies: this.particleWorld.listSoftBodies(),
      xpbd: this.particleWorld.getSnapshot(),
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
      lastSolverStats: cloneSolverStats(this.lastSolverStats),
      lastXpbdStats: {
        ...this.lastXpbdStats
      }
    };
  }
}
