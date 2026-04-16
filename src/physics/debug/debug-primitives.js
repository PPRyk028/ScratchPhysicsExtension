import { cloneQuat, createIdentityQuat } from '../math/quat.js';
import { cloneVec3, createVec3 } from '../math/vec3.js';

function toFiniteNumber(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function createDebugColor(color, fallback) {
  const source = color ?? fallback;
  return {
    r: toFiniteNumber(source?.r, fallback.r),
    g: toFiniteNumber(source?.g, fallback.g),
    b: toFiniteNumber(source?.b, fallback.b),
    a: toFiniteNumber(source?.a, fallback.a)
  };
}

function cloneSource(source) {
  if (!source) {
    return null;
  }

  return { ...source };
}

function cloneDebugPrimitive(primitive) {
  if (primitive.type === DEBUG_PRIMITIVE_TYPES.POINT) {
    return createDebugPoint(primitive);
  }

  if (primitive.type === DEBUG_PRIMITIVE_TYPES.LINE) {
    return createDebugLine(primitive);
  }

  if (primitive.type === DEBUG_PRIMITIVE_TYPES.WIRE_BOX) {
    return createDebugWireBox(primitive);
  }

  return {
    ...primitive,
    source: cloneSource(primitive.source ?? null)
  };
}

export const DEBUG_FRAME_SCHEMA_VERSION = 'physics-debug-primitives@1';

export const DEBUG_PRIMITIVE_TYPES = Object.freeze({
  POINT: 'point',
  LINE: 'line',
  WIRE_BOX: 'wireBox'
});

export const DEFAULT_DEBUG_COLORS = Object.freeze({
  rigidBodyWireframe: Object.freeze({ r: 64, g: 180, b: 255, a: 1 }),
  rigidBodyCenter: Object.freeze({ r: 255, g: 196, b: 61, a: 1 }),
  sleepingRigidBodyWireframe: Object.freeze({ r: 145, g: 160, b: 174, a: 1 }),
  staticColliderWireframe: Object.freeze({ r: 107, g: 222, b: 157, a: 1 }),
  oneWayPlatformWireframe: Object.freeze({ r: 255, g: 204, b: 102, a: 1 }),
  sensorColliderWireframe: Object.freeze({ r: 241, g: 102, b: 255, a: 1 }),
  sensorOrigin: Object.freeze({ r: 255, g: 170, b: 255, a: 1 }),
  broadphaseAabb: Object.freeze({ r: 250, g: 128, b: 114, a: 0.9 }),
  contactPoint: Object.freeze({ r: 255, g: 72, b: 72, a: 1 }),
  contactNormal: Object.freeze({ r: 255, g: 150, b: 54, a: 1 }),
  triggerContactPoint: Object.freeze({ r: 255, g: 112, b: 214, a: 1 }),
  triggerContactNormal: Object.freeze({ r: 234, g: 166, b: 255, a: 1 }),
  characterController: Object.freeze({ r: 122, g: 255, b: 128, a: 1 }),
  characterGroundPoint: Object.freeze({ r: 255, g: 231, b: 115, a: 1 }),
  characterGroundNormal: Object.freeze({ r: 255, g: 179, b: 79, a: 1 }),
  characterMovePath: Object.freeze({ r: 117, g: 235, b: 255, a: 1 }),
  characterHitPoint: Object.freeze({ r: 255, g: 120, b: 120, a: 1 }),
  characterHitNormal: Object.freeze({ r: 255, g: 82, b: 82, a: 1 }),
  surfacePoint: Object.freeze({ r: 102, g: 255, b: 219, a: 1 }),
  surfaceNormal: Object.freeze({ r: 94, g: 226, b: 255, a: 1 }),
  surfaceSupportVelocity: Object.freeze({ r: 255, g: 217, b: 94, a: 1 }),
  surfaceConveyorVelocity: Object.freeze({ r: 255, g: 122, b: 204, a: 1 }),
  raycastHitLine: Object.freeze({ r: 76, g: 229, b: 255, a: 1 }),
  raycastMissLine: Object.freeze({ r: 113, g: 132, b: 164, a: 0.95 }),
  raycastHitPoint: Object.freeze({ r: 255, g: 64, b: 197, a: 1 }),
  raycastHitNormal: Object.freeze({ r: 192, g: 104, b: 255, a: 1 }),
  shapeCastHitLine: Object.freeze({ r: 78, g: 245, b: 194, a: 1 }),
  shapeCastMissLine: Object.freeze({ r: 102, g: 150, b: 130, a: 0.95 }),
  shapeCastHitPoint: Object.freeze({ r: 255, g: 214, b: 72, a: 1 }),
  shapeCastHitNormal: Object.freeze({ r: 255, g: 133, b: 64, a: 1 }),
  ccdPath: Object.freeze({ r: 255, g: 94, b: 94, a: 1 }),
  ccdHitPoint: Object.freeze({ r: 255, g: 255, b: 255, a: 1 }),
  ccdHitNormal: Object.freeze({ r: 255, g: 186, b: 59, a: 1 }),
  distanceJoint: Object.freeze({ r: 185, g: 136, b: 255, a: 1 }),
  sleepingDistanceJoint: Object.freeze({ r: 142, g: 128, b: 160, a: 0.95 }),
  pointToPointJoint: Object.freeze({ r: 255, g: 162, b: 73, a: 1 }),
  sleepingPointToPointJoint: Object.freeze({ r: 164, g: 137, b: 112, a: 0.95 }),
  hingeJoint: Object.freeze({ r: 255, g: 215, b: 94, a: 1 }),
  sleepingHingeJoint: Object.freeze({ r: 168, g: 154, b: 112, a: 0.95 }),
  fixedJoint: Object.freeze({ r: 145, g: 232, b: 129, a: 1 }),
  sleepingFixedJoint: Object.freeze({ r: 118, g: 158, b: 111, a: 0.95 }),
  brokenJoint: Object.freeze({ r: 255, g: 92, b: 92, a: 1 }),
  hingeAxisA: Object.freeze({ r: 102, g: 208, b: 255, a: 1 }),
  hingeAxisB: Object.freeze({ r: 255, g: 118, b: 118, a: 1 }),
  hingeMotor: Object.freeze({ r: 94, g: 255, b: 156, a: 1 }),
  clothStructuralEdge: Object.freeze({ r: 127, g: 214, b: 255, a: 1 }),
  clothShearEdge: Object.freeze({ r: 139, g: 167, b: 255, a: 0.95 }),
  clothBendEdge: Object.freeze({ r: 194, g: 154, b: 255, a: 0.9 }),
  clothParticle: Object.freeze({ r: 255, g: 221, b: 111, a: 1 }),
  clothPinnedParticle: Object.freeze({ r: 255, g: 117, b: 117, a: 1 }),
  clothContactPoint: Object.freeze({ r: 171, g: 255, b: 150, a: 1 }),
  clothContactNormal: Object.freeze({ r: 120, g: 255, b: 214, a: 1 }),
  clothSelfContactPoint: Object.freeze({ r: 255, g: 96, b: 214, a: 1 }),
  clothSelfContactNormal: Object.freeze({ r: 255, g: 164, b: 224, a: 1 }),
  softBodyStructuralEdge: Object.freeze({ r: 255, g: 196, b: 102, a: 1 }),
  softBodyShearEdge: Object.freeze({ r: 255, g: 161, b: 132, a: 0.95 }),
  softBodyBendEdge: Object.freeze({ r: 255, g: 130, b: 175, a: 0.9 }),
  softBodyParticle: Object.freeze({ r: 255, g: 232, b: 133, a: 1 }),
  softBodyPinnedParticle: Object.freeze({ r: 255, g: 118, b: 118, a: 1 }),
  softBodyContactPoint: Object.freeze({ r: 170, g: 255, b: 203, a: 1 }),
  softBodyContactNormal: Object.freeze({ r: 110, g: 255, b: 237, a: 1 })
});

export function countDebugPrimitivesByType(primitives) {
  const counts = {};

  for (const primitive of primitives) {
    counts[primitive.type] = (counts[primitive.type] ?? 0) + 1;
  }

  return counts;
}

export function createDebugPoint(options = {}) {
  return {
    type: DEBUG_PRIMITIVE_TYPES.POINT,
    id: String(options.id ?? '').trim() || 'debug-point',
    category: String(options.category ?? '').trim() || 'point',
    position: cloneVec3(options.position ?? createVec3()),
    color: createDebugColor(options.color, DEFAULT_DEBUG_COLORS.rigidBodyCenter),
    size: toFiniteNumber(options.size, 4),
    source: cloneSource(options.source ?? null)
  };
}

export function createDebugLine(options = {}) {
  return {
    type: DEBUG_PRIMITIVE_TYPES.LINE,
    id: String(options.id ?? '').trim() || 'debug-line',
    category: String(options.category ?? '').trim() || 'line',
    start: cloneVec3(options.start ?? createVec3()),
    end: cloneVec3(options.end ?? createVec3()),
    color: createDebugColor(options.color, DEFAULT_DEBUG_COLORS.rigidBodyWireframe),
    source: cloneSource(options.source ?? null)
  };
}

export function createDebugWireBox(options = {}) {
  return {
    type: DEBUG_PRIMITIVE_TYPES.WIRE_BOX,
    id: String(options.id ?? '').trim() || 'debug-wire-box',
    category: String(options.category ?? '').trim() || 'wire-box',
    center: cloneVec3(options.center ?? createVec3()),
    halfExtents: cloneVec3(options.halfExtents ?? createVec3(0.5, 0.5, 0.5)),
    rotation: cloneQuat(options.rotation ?? createIdentityQuat()),
    color: createDebugColor(options.color, DEFAULT_DEBUG_COLORS.rigidBodyWireframe),
    source: cloneSource(options.source ?? null)
  };
}

export function createDebugFrame(options = {}) {
  const primitives = Array.isArray(options.primitives)
    ? options.primitives.map((primitive) => cloneDebugPrimitive(primitive))
    : [];

  return {
    schemaVersion: DEBUG_FRAME_SCHEMA_VERSION,
    frameNumber: toFiniteNumber(options.frameNumber, 0),
    simulationTick: toFiniteNumber(options.simulationTick, 0),
    camera: {
      position: cloneVec3(options.camera?.position ?? createVec3()),
      target: cloneVec3(options.camera?.target ?? createVec3())
    },
    primitives,
    stats: {
      primitiveCount: primitives.length,
      byType: countDebugPrimitivesByType(primitives),
      ...options.stats
    }
  };
}
