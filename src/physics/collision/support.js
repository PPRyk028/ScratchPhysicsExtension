import { addScaledVec3, addVec3, cloneVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, maxVec3, minVec3, negateVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';

function clampNumber(value, minValue, maxValue) {
  return Math.max(minValue, Math.min(maxValue, value));
}

export function getShapeWorldCenter(shape, worldPose) {
  return addVec3(worldPose?.position ?? createVec3(), shape?.localPose?.position ?? createVec3());
}

export function getShapeSupportPoint(shape, worldPose, direction) {
  const center = getShapeWorldCenter(shape, worldPose);
  const dir = normalizeVec3(direction, createVec3(1, 0, 0));

  if (shape?.type === 'box') {
    return addVec3(center, createVec3(
      dir.x >= 0 ? shape.geometry.halfExtents.x : -shape.geometry.halfExtents.x,
      dir.y >= 0 ? shape.geometry.halfExtents.y : -shape.geometry.halfExtents.y,
      dir.z >= 0 ? shape.geometry.halfExtents.z : -shape.geometry.halfExtents.z
    ));
  }

  if (shape?.type === 'sphere') {
    return addScaledVec3(center, dir, shape.geometry.radius);
  }

  if (shape?.type === 'capsule') {
    const axisOffset = createVec3(0, dir.y >= 0 ? shape.geometry.halfHeight : -shape.geometry.halfHeight, 0);
    return addScaledVec3(addVec3(center, axisOffset), dir, shape.geometry.radius);
  }

  if (shape?.type === 'convex-hull') {
    if (!Array.isArray(shape.geometry.vertices) || shape.geometry.vertices.length === 0) {
      return center;
    }

    let bestVertex = addVec3(center, shape.geometry.vertices[0]);
    let bestDot = dotVec3(bestVertex, dir);

    for (let index = 1; index < shape.geometry.vertices.length; index += 1) {
      const candidate = addVec3(center, shape.geometry.vertices[index]);
      const candidateDot = dotVec3(candidate, dir);
      if (candidateDot > bestDot) {
        bestDot = candidateDot;
        bestVertex = candidate;
      }
    }

    return bestVertex;
  }

  return center;
}

export function clampPointToAabb(point, aabb) {
  return createVec3(
    clampNumber(point.x, aabb.min.x, aabb.max.x),
    clampNumber(point.y, aabb.min.y, aabb.max.y),
    clampNumber(point.z, aabb.min.z, aabb.max.z)
  );
}

export function getClosestPointOnSegment(point, segmentStart, segmentEnd) {
  const segment = subtractVec3(segmentEnd, segmentStart);
  const lengthSquared = lengthSquaredVec3(segment);
  if (lengthSquared <= 1e-12) {
    return cloneVec3(segmentStart);
  }

  const t = clampNumber(dotVec3(subtractVec3(point, segmentStart), segment) / lengthSquared, 0, 1);
  return addScaledVec3(segmentStart, segment, t);
}

export function getCapsuleSegmentEndpoints(shape, worldPose) {
  const center = getShapeWorldCenter(shape, worldPose);
  return {
    start: addVec3(center, createVec3(0, -shape.geometry.halfHeight, 0)),
    end: addVec3(center, createVec3(0, shape.geometry.halfHeight, 0))
  };
}

export function createTangentBasis(normal) {
  const unitNormal = normalizeVec3(normal, createVec3(0, 1, 0));
  const helperAxis = Math.abs(unitNormal.y) < 0.9 ? createVec3(0, 1, 0) : createVec3(1, 0, 0);
  const tangentA = normalizeVec3(crossVec3(helperAxis, unitNormal), createVec3(1, 0, 0));
  const tangentB = normalizeVec3(crossVec3(unitNormal, tangentA), createVec3(0, 0, 1));

  return {
    normal: unitNormal,
    tangentA,
    tangentB
  };
}

export function getSupportMappedPenetration(shapeA, poseA, shapeB, poseB, normal) {
  const unitNormal = normalizeVec3(normal, createVec3(0, 1, 0));
  const supportA = getShapeSupportPoint(shapeA, poseA, unitNormal);
  const supportB = getShapeSupportPoint(shapeB, poseB, negateVec3(unitNormal));
  const signedSeparation = dotVec3(subtractVec3(supportB, supportA), unitNormal);

  return {
    supportA,
    supportB,
    signedSeparation,
    penetration: Math.max(0, -signedSeparation)
  };
}

export function getConvexHullBounds(shape, worldPose) {
  const center = getShapeWorldCenter(shape, worldPose);
  if (!Array.isArray(shape?.geometry?.vertices) || shape.geometry.vertices.length === 0) {
    return {
      min: center,
      max: center
    };
  }

  let minPoint = addVec3(center, shape.geometry.vertices[0]);
  let maxPoint = cloneVec3(minPoint);

  for (let index = 1; index < shape.geometry.vertices.length; index += 1) {
    const worldVertex = addVec3(center, shape.geometry.vertices[index]);
    minPoint = minVec3(minPoint, worldVertex);
    maxPoint = maxVec3(maxPoint, worldVertex);
  }

  return {
    min: minPoint,
    max: maxPoint
  };
}
