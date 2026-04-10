import { createVec3 } from '../math/vec3.js';
import { getAabbOverlap } from './aabb.js';

function midpoint(minValue, maxValue) {
  return (minValue + maxValue) / 2;
}

function chooseContactAxis(overlap) {
  if (overlap.x <= overlap.y && overlap.x <= overlap.z) {
    return 'x';
  }

  if (overlap.y <= overlap.z) {
    return 'y';
  }

  return 'z';
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

export function collideBoxPair(pair) {
  const overlap = getAabbOverlap(pair?.aabbA, pair?.aabbB);
  if (!overlap) {
    return null;
  }

  const axis = chooseContactAxis(overlap);
  const centerDelta = pair.aabbB.center[axis] - pair.aabbA.center[axis];
  const normalSign = centerDelta >= 0 ? 1 : -1;
  const normal = createAxisNormal(axis, normalSign);
  const penetration = overlap[axis];
  const contactPoint = createVec3(
    midpoint(Math.max(pair.aabbA.min.x, pair.aabbB.min.x), Math.min(pair.aabbA.max.x, pair.aabbB.max.x)),
    midpoint(Math.max(pair.aabbA.min.y, pair.aabbB.min.y), Math.min(pair.aabbA.max.y, pair.aabbB.max.y)),
    midpoint(Math.max(pair.aabbA.min.z, pair.aabbB.min.z), Math.min(pair.aabbA.max.z, pair.aabbB.max.z))
  );

  return {
    id: `${pair.pairKey}:contact-pair`,
    pairKey: pair.pairKey,
    algorithm: 'box-box-aabb-v1',
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
    contactCount: 1,
    contacts: [
      {
        id: `${pair.pairKey}:contact-0`,
        position: contactPoint,
        normal,
        penetration,
        separation: -penetration,
        featureId: `axis:${axis}`
      }
    ]
  };
}
