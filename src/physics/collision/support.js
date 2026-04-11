import { cloneQuat, createIdentityQuat, inverseRotateVec3ByQuat, multiplyQuat, rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, addVec3, cloneVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, maxVec3, minVec3, negateVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';

const SUPPORT_POLYGON_EPSILON = 1e-5;
const SUPPORT_FACE_ALIGNMENT_THRESHOLD = 0.965;
const SUPPORT_FACE_PROJECTION_EPSILON = 1e-3;

function clampNumber(value, minValue, maxValue) {
  return Math.max(minValue, Math.min(maxValue, value));
}

export function getShapeWorldCenter(shape, worldPose) {
  return getShapeWorldPose(shape, worldPose).position;
}

export function composePoses(parentPose, localPose) {
  const parentRotation = cloneQuat(parentPose?.rotation ?? createIdentityQuat());
  const localRotation = cloneQuat(localPose?.rotation ?? createIdentityQuat());
  const localPosition = cloneVec3(localPose?.position ?? createVec3());

  return {
    position: addVec3(parentPose?.position ?? createVec3(), rotateVec3ByQuat(parentRotation, localPosition)),
    rotation: multiplyQuat(parentRotation, localRotation)
  };
}

export function transformPointByPose(pose, point) {
  return addVec3(pose?.position ?? createVec3(), rotateVec3ByQuat(pose?.rotation ?? createIdentityQuat(), point));
}

export function getShapeWorldPose(shape, worldPose) {
  return composePoses(worldPose, shape?.localPose ?? {
    position: createVec3(),
    rotation: createIdentityQuat()
  });
}

export function getShapeSupportFeature(shape, worldPose, direction) {
  const shapeWorldPose = getShapeWorldPose(shape, worldPose);
  const dir = normalizeVec3(direction, createVec3(1, 0, 0));
  const localDirection = inverseRotateVec3ByQuat(shapeWorldPose.rotation, dir);

  if (shape?.type === 'box') {
    const localPoint = createVec3(
      localDirection.x >= 0 ? shape.geometry.halfExtents.x : -shape.geometry.halfExtents.x,
      localDirection.y >= 0 ? shape.geometry.halfExtents.y : -shape.geometry.halfExtents.y,
      localDirection.z >= 0 ? shape.geometry.halfExtents.z : -shape.geometry.halfExtents.z
    );
    return {
      worldPoint: transformPointByPose(shapeWorldPose, localPoint),
      localPoint,
      featureId: `corner:${localPoint.x >= 0 ? '+' : '-'}${localPoint.y >= 0 ? '+' : '-'}${localPoint.z >= 0 ? '+' : '-'}`,
      featureType: 'vertex'
    };
  }

  if (shape?.type === 'sphere') {
    const localPoint = scaleVec3(localDirection, shape.geometry.radius);
    return {
      worldPoint: addScaledVec3(shapeWorldPose.position, dir, shape.geometry.radius),
      localPoint,
      featureId: 'sphere:surface',
      featureType: 'surface'
    };
  }

  if (shape?.type === 'capsule') {
    const axisOffset = createVec3(0, localDirection.y >= 0 ? shape.geometry.halfHeight : -shape.geometry.halfHeight, 0);
    const localPoint = addScaledVec3(axisOffset, localDirection, shape.geometry.radius);
    return {
      worldPoint: addScaledVec3(transformPointByPose(shapeWorldPose, axisOffset), dir, shape.geometry.radius),
      localPoint,
      featureId: `capsule:${axisOffset.y >= 0 ? 'top' : 'bottom'}`,
      featureType: 'capsule-cap'
    };
  }

  if (shape?.type === 'convex-hull') {
    if (!Array.isArray(shape.geometry.vertices) || shape.geometry.vertices.length === 0) {
      return {
        worldPoint: shapeWorldPose.position,
        localPoint: createVec3(),
        featureId: 'vertex:empty',
        featureType: 'vertex'
      };
    }

    let bestIndex = 0;
    let bestLocalPoint = cloneVec3(shape.geometry.vertices[0]);
    let bestWorldPoint = transformPointByPose(shapeWorldPose, bestLocalPoint);
    let bestDot = dotVec3(bestWorldPoint, dir);

    for (let index = 1; index < shape.geometry.vertices.length; index += 1) {
      const candidateLocalPoint = shape.geometry.vertices[index];
      const candidateWorldPoint = transformPointByPose(shapeWorldPose, candidateLocalPoint);
      const candidateDot = dotVec3(candidateWorldPoint, dir);
      if (candidateDot > bestDot + 1e-8) {
        bestDot = candidateDot;
        bestIndex = index;
        bestLocalPoint = cloneVec3(candidateLocalPoint);
        bestWorldPoint = candidateWorldPoint;
      }
    }

    return {
      worldPoint: bestWorldPoint,
      localPoint: bestLocalPoint,
      featureId: `vertex:${bestIndex}`,
      featureType: 'vertex'
    };
  }

  return {
    worldPoint: shapeWorldPose.position,
    localPoint: createVec3(),
    featureId: 'shape:center',
    featureType: 'point'
  };
}

export function getShapeSupportPoint(shape, worldPose, direction) {
  return getShapeSupportFeature(shape, worldPose, direction).worldPoint;
}

function dedupeSupportPolygonPoints(points) {
  const uniquePoints = [];
  const seen = new Set();

  for (const point of points) {
    const key = `${point.worldPoint.x.toFixed(6)}|${point.worldPoint.y.toFixed(6)}|${point.worldPoint.z.toFixed(6)}`;
    if (seen.has(key)) {
      continue;
    }

    seen.add(key);
    uniquePoints.push(point);
  }

  return uniquePoints;
}

function buildBoxSupportPolygon(shapeWorldPose, halfExtents, localDirection, direction) {
  const axisWeights = [
    Math.abs(localDirection.x),
    Math.abs(localDirection.y),
    Math.abs(localDirection.z)
  ];
  let dominantAxisIndex = 0;
  if (axisWeights[1] > axisWeights[dominantAxisIndex]) {
    dominantAxisIndex = 1;
  }
  if (axisWeights[2] > axisWeights[dominantAxisIndex]) {
    dominantAxisIndex = 2;
  }

  const axisSigns = [
    localDirection.x >= 0 ? 1 : -1,
    localDirection.y >= 0 ? 1 : -1,
    localDirection.z >= 0 ? 1 : -1
  ];
  const signs = [-1, 1];
  const points = [];

  for (const signA of signs) {
    for (const signB of signs) {
      const localPoint = createVec3(
        dominantAxisIndex === 0 ? halfExtents.x * axisSigns[0] : halfExtents.x * signA,
        dominantAxisIndex === 1 ? halfExtents.y * axisSigns[1] : halfExtents.y * (dominantAxisIndex === 0 ? signA : signB),
        dominantAxisIndex === 2 ? halfExtents.z * axisSigns[2] : halfExtents.z * signB
      );
      const worldPoint = transformPointByPose(shapeWorldPose, localPoint);
      points.push({
        worldPoint,
        localPoint,
        featureId: `corner:${localPoint.x >= 0 ? '+' : '-'}${localPoint.y >= 0 ? '+' : '-'}${localPoint.z >= 0 ? '+' : '-'}`,
        featureType: 'vertex'
      });
    }
  }

  const planeOffset = points.reduce((maxProjection, point) => Math.max(maxProjection, dotVec3(point.worldPoint, direction)), -Infinity);

  return {
    points: dedupeSupportPolygonPoints(points),
    planeOffset
  };
}

function buildConvexHullSupportVertexPolygon(shapeWorldPose, vertices, direction) {
  const points = [];
  let maxDot = -Infinity;

  for (let index = 0; index < vertices.length; index += 1) {
    const localPoint = vertices[index];
    const worldPoint = transformPointByPose(shapeWorldPose, localPoint);
    const projection = dotVec3(worldPoint, direction);

    if (projection > maxDot + SUPPORT_POLYGON_EPSILON) {
      maxDot = projection;
      points.length = 0;
    }

    if (projection >= maxDot - SUPPORT_POLYGON_EPSILON) {
      points.push({
        worldPoint,
        localPoint: cloneVec3(localPoint),
        featureId: `vertex:${index}`,
        featureType: 'vertex'
      });
    }
  }

  return {
    points: dedupeSupportPolygonPoints(points),
    planeOffset: maxDot
  };
}

function buildConvexHullSupportPolygon(shapeWorldPose, geometry, direction) {
  const vertices = Array.isArray(geometry?.vertices) ? geometry.vertices : [];
  const fallback = buildConvexHullSupportVertexPolygon(shapeWorldPose, vertices, direction);
  const faces = Array.isArray(geometry?.faces) ? geometry.faces : [];
  if (faces.length === 0) {
    return fallback;
  }

  const faceCandidates = [];
  for (const face of faces) {
    const boundaryIndices = Array.isArray(face.boundaryIndices) ? face.boundaryIndices : [];
    if (boundaryIndices.length < 2) {
      continue;
    }

    const points = boundaryIndices.map((index) => {
      const localPoint = vertices[index];
      return {
        worldPoint: transformPointByPose(shapeWorldPose, localPoint),
        localPoint: cloneVec3(localPoint),
        featureId: `vertex:${index}`,
        featureType: 'vertex'
      };
    });
    const planeOffset = points.reduce((maxProjection, point) => Math.max(maxProjection, dotVec3(point.worldPoint, direction)), -Infinity);
    const worldNormal = normalizeVec3(rotateVec3ByQuat(shapeWorldPose.rotation, face.normal ?? createVec3(0, 1, 0)), direction);
    const alignment = dotVec3(worldNormal, direction);

    faceCandidates.push({
      points,
      planeOffset,
      alignment
    });
  }

  const selectedFace = faceCandidates
    .filter((candidate) =>
      candidate.alignment >= SUPPORT_FACE_ALIGNMENT_THRESHOLD &&
      candidate.planeOffset >= fallback.planeOffset - SUPPORT_FACE_PROJECTION_EPSILON
    )
    .sort((left, right) => {
      if (right.alignment !== left.alignment) {
        return right.alignment - left.alignment;
      }

      if (right.points.length !== left.points.length) {
        return right.points.length - left.points.length;
      }

      return right.planeOffset - left.planeOffset;
    })[0];

  if (selectedFace) {
    return {
      points: dedupeSupportPolygonPoints(selectedFace.points),
      planeOffset: selectedFace.planeOffset
    };
  }

  return fallback;
}

export function getShapeSupportPolygon(shape, worldPose, direction) {
  const shapeWorldPose = getShapeWorldPose(shape, worldPose);
  const resolvedDirection = normalizeVec3(direction, createVec3(1, 0, 0));
  const localDirection = inverseRotateVec3ByQuat(shapeWorldPose.rotation, resolvedDirection);

  if (shape?.type === 'box') {
    const result = buildBoxSupportPolygon(shapeWorldPose, shape.geometry.halfExtents, localDirection, resolvedDirection);
    return {
      normal: resolvedDirection,
      planeOffset: result.planeOffset,
      points: result.points
    };
  }

  if (shape?.type === 'convex-hull') {
    const result = buildConvexHullSupportPolygon(shapeWorldPose, shape.geometry ?? {}, resolvedDirection);
    return {
      normal: resolvedDirection,
      planeOffset: result.planeOffset,
      points: result.points
    };
  }

  const feature = getShapeSupportFeature(shape, worldPose, resolvedDirection);
  return {
    normal: resolvedDirection,
    planeOffset: dotVec3(feature.worldPoint, resolvedDirection),
    points: [feature]
  };
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
  const shapeWorldPose = getShapeWorldPose(shape, worldPose);
  const axis = rotateVec3ByQuat(shapeWorldPose.rotation, createVec3(0, 1, 0));
  return {
    start: addScaledVec3(shapeWorldPose.position, axis, -shape.geometry.halfHeight),
    end: addScaledVec3(shapeWorldPose.position, axis, shape.geometry.halfHeight)
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
  const shapeWorldPose = getShapeWorldPose(shape, worldPose);
  if (!Array.isArray(shape?.geometry?.vertices) || shape.geometry.vertices.length === 0) {
    return {
      min: shapeWorldPose.position,
      max: shapeWorldPose.position
    };
  }

  let minPoint = transformPointByPose(shapeWorldPose, shape.geometry.vertices[0]);
  let maxPoint = cloneVec3(minPoint);

  for (let index = 1; index < shape.geometry.vertices.length; index += 1) {
    const worldVertex = transformPointByPose(shapeWorldPose, shape.geometry.vertices[index]);
    minPoint = minVec3(minPoint, worldVertex);
    maxPoint = maxVec3(maxPoint, worldVertex);
  }

  return {
    min: minPoint,
    max: maxPoint
  };
}
