import { addVec3, cloneVec3, createVec3, maxVec3, minVec3 } from '../math/vec3.js';

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
  const localCenter = cloneVec3(shape?.localPose?.position ?? createVec3());

  if (shape?.type === 'box') {
    return createAabbFromCenterHalfExtents(localCenter, shape.geometry.halfExtents);
  }

  if (shape?.type === 'sphere') {
    return createAabbFromCenterHalfExtents(localCenter, createVec3(shape.geometry.radius, shape.geometry.radius, shape.geometry.radius));
  }

  if (shape?.type === 'capsule') {
    return createAabbFromCenterHalfExtents(
      localCenter,
      createVec3(shape.geometry.radius, shape.geometry.halfHeight + shape.geometry.radius, shape.geometry.radius)
    );
  }

  if (shape?.type === 'convex-hull') {
    if (!Array.isArray(shape.geometry.vertices) || shape.geometry.vertices.length === 0) {
      return createAabbFromCenterHalfExtents(localCenter, createVec3());
    }

    let localMin = addVec3(localCenter, shape.geometry.vertices[0]);
    let localMax = addVec3(localCenter, shape.geometry.vertices[0]);

    for (let index = 1; index < shape.geometry.vertices.length; index += 1) {
      const vertex = addVec3(localCenter, shape.geometry.vertices[index]);
      localMin = minVec3(localMin, vertex);
      localMax = maxVec3(localMax, vertex);
    }

    return createAabbFromMinMax(localMin, localMax);
  }

  return null;
}

export function computeShapeWorldAabb(shape, worldPose) {
  const localAabb = computeLocalShapeAabb(shape);
  if (!localAabb) {
    return null;
  }

  return createAabbFromCenterHalfExtents(
    addVec3(worldPose?.position ?? createVec3(), localAabb.center),
    localAabb.halfExtents
  );
}
