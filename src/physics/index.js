import { createAabbFromCenterHalfExtents, createAabbFromMinMax, cloneAabb, computeLocalShapeAabb, computeShapeWorldAabb, getAabbOverlap, testAabbOverlap, testPointInAabb } from './collision/aabb.js';
import { buildBroadphasePairs, cloneBroadphasePair, cloneBroadphaseProxy, createBroadphasePair, createBroadphaseProxy } from './collision/broadphase.js';
import { collideBoxPair } from './collision/box-box.js';
import { buildConvexContactManifold, collideConvexPairWithGjkEpa, runEpa, runGjk } from './collision/gjk-epa.js';
import { cloneContactPair, createContactPair, runNarrowphase } from './collision/narrowphase.js';
import { clampPointToAabb, composePoses, createTangentBasis, getCapsuleSegmentEndpoints, getClosestPointOnSegment, getShapeSupportFeature, getShapeSupportPoint, getShapeWorldCenter, getShapeWorldPose, getSupportMappedPenetration, transformPointByPose } from './collision/support.js';
import { DEBUG_FRAME_SCHEMA_VERSION, DEBUG_PRIMITIVE_TYPES, DEFAULT_DEBUG_COLORS, createDebugFrame, createDebugLine, createDebugPoint, createDebugWireBox, countDebugPrimitivesByType } from './debug/debug-primitives.js';
import { cloneManifold, cloneManifoldContact, ManifoldCache } from './manifold/manifold-cache.js';
import { cloneQuat, conjugateQuat, createIdentityQuat, createQuat, integrateQuat, inverseRotateVec3ByQuat, lengthSquaredQuat, multiplyQuat, normalizeQuat, rotateVec3ByQuat } from './math/quat.js';
import { createVec3, cloneVec3, zeroVec3, scaleVec3, addVec3, addScaledVec3, subtractVec3, minVec3, maxVec3, dotVec3, lengthSquaredVec3, lengthVec3, negateVec3, normalizeVec3, crossVec3 } from './math/vec3.js';
import { solveNormalContactConstraints } from './rigid/normal-contact-solver.js';
import { PhysicsWorld } from './world/physics-world.js';
import { ShapeRegistry } from './world/shape-registry.js';
import { BodyRegistry } from './world/body-registry.js';
import { ColliderRegistry } from './world/collider-registry.js';
import { MaterialRegistry } from './world/material-registry.js';

export { createAabbFromCenterHalfExtents, createAabbFromMinMax, cloneAabb, computeLocalShapeAabb, computeShapeWorldAabb, getAabbOverlap, testAabbOverlap, testPointInAabb, buildBroadphasePairs, cloneBroadphasePair, cloneBroadphaseProxy, createBroadphasePair, createBroadphaseProxy, collideBoxPair, runGjk, runEpa, collideConvexPairWithGjkEpa, buildConvexContactManifold, cloneContactPair, createContactPair, runNarrowphase, clampPointToAabb, composePoses, createTangentBasis, getCapsuleSegmentEndpoints, getClosestPointOnSegment, getShapeSupportFeature, getShapeSupportPoint, getShapeWorldCenter, getShapeWorldPose, getSupportMappedPenetration, transformPointByPose, DEBUG_FRAME_SCHEMA_VERSION, DEBUG_PRIMITIVE_TYPES, DEFAULT_DEBUG_COLORS, createDebugFrame, createDebugLine, createDebugPoint, createDebugWireBox, countDebugPrimitivesByType, cloneManifold, cloneManifoldContact, ManifoldCache, createQuat, createIdentityQuat, cloneQuat, lengthSquaredQuat, normalizeQuat, conjugateQuat, multiplyQuat, rotateVec3ByQuat, inverseRotateVec3ByQuat, integrateQuat, createVec3, cloneVec3, zeroVec3, scaleVec3, addVec3, addScaledVec3, subtractVec3, minVec3, maxVec3, dotVec3, lengthSquaredVec3, lengthVec3, negateVec3, normalizeVec3, crossVec3, solveNormalContactConstraints, PhysicsWorld, ShapeRegistry, BodyRegistry, ColliderRegistry, MaterialRegistry };
