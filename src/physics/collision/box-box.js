import { createVec3 } from '../math/vec3.js';
import { getAabbOverlap } from './aabb.js';

function midpoint(minValue, maxValue) {
  return (minValue + maxValue) / 2;
}

function chooseContactAxis(pair, overlap) {
  const axes = ['x', 'y', 'z'];
  let bestAxis = axes[0];
  let bestOverlap = overlap[bestAxis];
  let bestDelta = Math.abs(pair.aabbB.center[bestAxis] - pair.aabbA.center[bestAxis]);

  for (let index = 1; index < axes.length; index += 1) {
    const axis = axes[index];
    const candidateOverlap = overlap[axis];
    const candidateDelta = Math.abs(pair.aabbB.center[axis] - pair.aabbA.center[axis]);

    if (candidateOverlap < bestOverlap - 1e-8) {
      bestAxis = axis;
      bestOverlap = candidateOverlap;
      bestDelta = candidateDelta;
      continue;
    }

    if (Math.abs(candidateOverlap - bestOverlap) <= 1e-8 && candidateDelta > bestDelta + 1e-8) {
      bestAxis = axis;
      bestOverlap = candidateOverlap;
      bestDelta = candidateDelta;
    }
  }

  return bestAxis;
}

function createAxisNormal(axis, sign) {
  if (axis === 'x') {
    return createVec3(sign, 0, 0);
  }

  if (axis === 'y') {
    return createVec3(0, sign, 0);
  }

  return createVec3(0, 0, sign);
}

function getFaceCoordinate(pair, axis, normalSign) {
  if (normalSign >= 0) {
    if (axis === 'x') {
      return midpoint(pair.aabbA.max.x, pair.aabbB.min.x);
    }

    if (axis === 'y') {
      return midpoint(pair.aabbA.max.y, pair.aabbB.min.y);
    }

    return midpoint(pair.aabbA.max.z, pair.aabbB.min.z);
  }

  if (axis === 'x') {
    return midpoint(pair.aabbA.min.x, pair.aabbB.max.x);
  }

  if (axis === 'y') {
    return midpoint(pair.aabbA.min.y, pair.aabbB.max.y);
  }

  return midpoint(pair.aabbA.min.z, pair.aabbB.max.z);
}

function getAxisRange(pair, axis) {
  return {
    min: Math.max(pair.aabbA.min[axis], pair.aabbB.min[axis]),
    max: Math.min(pair.aabbA.max[axis], pair.aabbB.max[axis])
  };
}

function dedupeContactPoints(points) {
  const uniquePoints = [];
  const seenKeys = new Set();

  for (const point of points) {
    const key = `${point.x.toFixed(6)}|${point.y.toFixed(6)}|${point.z.toFixed(6)}`;
    if (seenKeys.has(key)) {
      continue;
    }

    seenKeys.add(key);
    uniquePoints.push(point);
  }

  return uniquePoints;
}

function createFaceContactPoints(pair, axis, normalSign) {
  const contactPlane = getFaceCoordinate(pair, axis, normalSign);
  const tangentAxes = ['x', 'y', 'z'].filter((candidateAxis) => candidateAxis !== axis);
  const firstRange = getAxisRange(pair, tangentAxes[0]);
  const secondRange = getAxisRange(pair, tangentAxes[1]);
  const firstValues = [firstRange.min, firstRange.max];
  const secondValues = [secondRange.min, secondRange.max];
  const points = [];

  for (const firstValue of firstValues) {
    for (const secondValue of secondValues) {
      const point = {
        x: midpoint(pair.aabbA.center.x, pair.aabbB.center.x),
        y: midpoint(pair.aabbA.center.y, pair.aabbB.center.y),
        z: midpoint(pair.aabbA.center.z, pair.aabbB.center.z)
      };
      point[axis] = contactPlane;
      point[tangentAxes[0]] = firstValue;
      point[tangentAxes[1]] = secondValue;
      points.push(createVec3(point.x, point.y, point.z));
    }
  }

  return dedupeContactPoints(points);
}

export function collideBoxPair(pair) {
  const overlap = getAabbOverlap(pair?.aabbA, pair?.aabbB);
  if (!overlap) {
    return null;
  }

  const axis = chooseContactAxis(pair, overlap);
  const centerDelta = pair.aabbB.center[axis] - pair.aabbA.center[axis];
  const normalSign = centerDelta >= 0 ? 1 : -1;
  const normal = createAxisNormal(axis, normalSign);
  const penetration = overlap[axis];
  const contactPoints = createFaceContactPoints(pair, axis, normalSign);

  return {
    id: `${pair.pairKey}:contact-pair`,
    pairKey: pair.pairKey,
    algorithm: 'box-box-aabb-v2',
    status: 'touching',
    pairKind: pair.pairKind,
    colliderAId: pair.colliderAId,
    colliderBId: pair.colliderBId,
    bodyAId: pair.bodyAId,
    bodyBId: pair.bodyBId,
    shapeAType: pair.shapeAType,
    shapeBType: pair.shapeBType,
    normal,
    penetration,
    contactCount: contactPoints.length,
    contacts: contactPoints.map((contactPoint, index) => ({
      id: `${pair.pairKey}:contact-${index}`,
      position: contactPoint,
      normal,
      penetration,
      separation: -penetration,
      featureId: `axis:${axis}:corner:${index}`
    }))
  };
}
