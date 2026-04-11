import { createIdentityQuat } from '../math/quat.js';
import { addVec3, cloneVec3, createVec3, maxVec3, minVec3 } from '../math/vec3.js';
import { getConvexHullBounds, getShapeSupportPoint, getShapeWorldPose } from './support.js';

function toNonNegativeNumber(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function absoluteHalfExtents(halfExtents) {
  return createVec3(
    Math.abs(toNonNegativeNumber(halfExtents?.x, 0)),
    Math.abs(toNonNegativeNumber(halfExtents?.y, 0)),
    Math.abs(toNonNegativeNumber(halfExtents?.z, 0))
  );
}

export function createAabbFromCenterHalfExtents(center, halfExtents) {
  const resolvedCenter = cloneVec3(center);
  const resolvedHalfExtents = absoluteHalfExtents(halfExtents);

  return {
    center: resolvedCenter,
    halfExtents: resolvedHalfExtents,
    min: createVec3(
      resolvedCenter.x - resolvedHalfExtents.x,
      resolvedCenter.y - resolvedHalfExtents.y,
      resolvedCenter.z - resolvedHalfExtents.z
    ),
    max: createVec3(
      resolvedCenter.x + resolvedHalfExtents.x,
      resolvedCenter.y + resolvedHalfExtents.y,
      resolvedCenter.z + resolvedHalfExtents.z
    )
  };
}

export function createAabbFromMinMax(min, max) {
  const resolvedMin = minVec3(min, max);
  const resolvedMax = maxVec3(min, max);

  return createAabbFromCenterHalfExtents(
    createVec3(
      (resolvedMin.x + resolvedMax.x) / 2,
      (resolvedMin.y + resolvedMax.y) / 2,
      (resolvedMin.z + resolvedMax.z) / 2
    ),
    createVec3(
      (resolvedMax.x - resolvedMin.x) / 2,
      (resolvedMax.y - resolvedMin.y) / 2,
      (resolvedMax.z - resolvedMin.z) / 2
    )
  );
}

export function cloneAabb(aabb) {
  return createAabbFromCenterHalfExtents(
    aabb?.center ?? createVec3(),
    aabb?.halfExtents ?? createVec3()
  );
}

export function testPointInAabb(point, aabb) {
  if (!aabb) {
    return false;
  }

  const resolvedPoint = cloneVec3(point);
  return (
    resolvedPoint.x >= aabb.min.x &&
    resolvedPoint.x <= aabb.max.x &&
    resolvedPoint.y >= aabb.min.y &&
    resolvedPoint.y <= aabb.max.y &&
    resolvedPoint.z >= aabb.min.z &&
    resolvedPoint.z <= aabb.max.z
  );
}

export function testAabbOverlap(left, right) {
  if (!left || !right) {
    return false;
  }

  return !(
    left.max.x < right.min.x ||
    left.min.x > right.max.x ||
    left.max.y < right.min.y ||
    left.min.y > right.max.y ||
    left.max.z < right.min.z ||
    left.min.z > right.max.z
  );
}

export function getAabbOverlap(left, right) {
  if (!testAabbOverlap(left, right)) {
    return null;
  }

  return createVec3(
    Math.min(left.max.x, right.max.x) - Math.max(left.min.x, right.min.x),
    Math.min(left.max.y, right.max.y) - Math.max(left.min.y, right.min.y),
    Math.min(left.max.z, right.max.z) - Math.max(left.min.z, right.min.z)
  );
}

export function computeLocalShapeAabb(shape) {
  return computeShapeWorldAabb(shape, {
    position: createVec3(),
    rotation: createIdentityQuat()
  });
}

export function computeShapeWorldAabb(shape, worldPose) {
  if (!shape) {
    return null;
  }

  if (shape.type === 'convex-hull') {
    const bounds = getConvexHullBounds(shape, worldPose);
    return createAabbFromMinMax(bounds.min, bounds.max);
  }

  const supportPose = getShapeWorldPose(shape, worldPose);
  const min = createVec3(
    getShapeSupportPoint(shape, worldPose, createVec3(-1, 0, 0)).x,
    getShapeSupportPoint(shape, worldPose, createVec3(0, -1, 0)).y,
    getShapeSupportPoint(shape, worldPose, createVec3(0, 0, -1)).z
  );
  const max = createVec3(
    getShapeSupportPoint(shape, worldPose, createVec3(1, 0, 0)).x,
    getShapeSupportPoint(shape, worldPose, createVec3(0, 1, 0)).y,
    getShapeSupportPoint(shape, worldPose, createVec3(0, 0, 1)).z
  );

  if (shape.type === 'sphere') {
    return createAabbFromCenterHalfExtents(
      supportPose.position,
      createVec3(shape.geometry.radius, shape.geometry.radius, shape.geometry.radius)
    );
  }

  return createAabbFromMinMax(min, max);
}
