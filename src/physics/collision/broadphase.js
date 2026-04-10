import { cloneAabb, testAabbOverlap } from './aabb.js';

function canonicalMotionType(motionType, hasBody) {
  if (!hasBody) {
    return 'static';
  }

  const resolved = String(motionType ?? '').trim().toLowerCase();
  if (resolved === 'static' || resolved === 'kinematic') {
    return resolved;
  }

  return 'dynamic';
}

function motionTypeRank(motionType) {
  if (motionType === 'dynamic') {
    return 0;
  }

  if (motionType === 'kinematic') {
    return 1;
  }

  return 2;
}

function sortMotionTypes(left, right) {
  if (motionTypeRank(left) <= motionTypeRank(right)) {
    return [left, right];
  }

  return [right, left];
}

function createPairKey(leftColliderId, rightColliderId) {
  return [leftColliderId, rightColliderId].sort().join('|');
}

function resolvePairKind(left, right) {
  const [leftType, rightType] = sortMotionTypes(left.motionType, right.motionType);
  return `${leftType}-${rightType}`;
}

function cloneNullableId(value) {
  return value ? String(value) : null;
}

function compareProxiesBySweepAxis(left, right) {
  if (left.aabb.min.x !== right.aabb.min.x) {
    return left.aabb.min.x - right.aabb.min.x;
  }

  if (left.aabb.min.y !== right.aabb.min.y) {
    return left.aabb.min.y - right.aabb.min.y;
  }

  if (left.aabb.min.z !== right.aabb.min.z) {
    return left.aabb.min.z - right.aabb.min.z;
  }

  return left.colliderId.localeCompare(right.colliderId);
}

export function createBroadphaseProxy(options = {}) {
  const motionType = canonicalMotionType(options.motionType, Boolean(options.bodyId));

  return {
    id: String(options.id ?? options.colliderId ?? '').trim() || String(options.colliderId ?? '').trim(),
    colliderId: String(options.colliderId ?? '').trim() || null,
    bodyId: cloneNullableId(options.bodyId),
    shapeId: cloneNullableId(options.shapeId),
    materialId: cloneNullableId(options.materialId),
    shapeType: String(options.shapeType ?? '').trim() || 'unknown',
    motionType,
    isStatic: motionType === 'static',
    isDynamic: motionType === 'dynamic',
    isKinematic: motionType === 'kinematic',
    isSensor: Boolean(options.isSensor),
    aabb: cloneAabb(options.aabb)
  };
}

export function cloneBroadphaseProxy(proxy) {
  return createBroadphaseProxy(proxy);
}

export function createBroadphasePair(options = {}) {
  return {
    id: String(options.id ?? options.pairKey ?? '').trim() || createPairKey(options.colliderAId, options.colliderBId),
    pairKey: String(options.pairKey ?? '').trim() || createPairKey(options.colliderAId, options.colliderBId),
    type: 'potential-collision-pair',
    pairKind: String(options.pairKind ?? '').trim() || 'dynamic-dynamic',
    colliderAId: cloneNullableId(options.colliderAId),
    colliderBId: cloneNullableId(options.colliderBId),
    bodyAId: cloneNullableId(options.bodyAId),
    bodyBId: cloneNullableId(options.bodyBId),
    shapeAId: cloneNullableId(options.shapeAId),
    shapeBId: cloneNullableId(options.shapeBId),
    materialAId: cloneNullableId(options.materialAId),
    materialBId: cloneNullableId(options.materialBId),
    shapeAType: String(options.shapeAType ?? '').trim() || 'unknown',
    shapeBType: String(options.shapeBType ?? '').trim() || 'unknown',
    motionTypeA: canonicalMotionType(options.motionTypeA, Boolean(options.bodyAId)),
    motionTypeB: canonicalMotionType(options.motionTypeB, Boolean(options.bodyBId)),
    aabbA: cloneAabb(options.aabbA),
    aabbB: cloneAabb(options.aabbB)
  };
}

export function cloneBroadphasePair(pair) {
  return createBroadphasePair(pair);
}

export function shouldGenerateBroadphasePair(left, right) {
  if (!left || !right) {
    return false;
  }

  if (!left.colliderId || !right.colliderId || left.colliderId === right.colliderId) {
    return false;
  }

  if (left.isSensor || right.isSensor) {
    return false;
  }

  if (left.bodyId && right.bodyId && left.bodyId === right.bodyId) {
    return false;
  }

  if (left.isStatic && right.isStatic) {
    return false;
  }

  return testAabbOverlap(left.aabb, right.aabb);
}

export function buildBroadphasePairs(proxies) {
  const sortedProxies = Array.isArray(proxies)
    ? proxies.map((proxy) => cloneBroadphaseProxy(proxy)).sort(compareProxiesBySweepAxis)
    : [];
  const pairs = [];

  for (let leftIndex = 0; leftIndex < sortedProxies.length; leftIndex += 1) {
    const left = sortedProxies[leftIndex];

    for (let rightIndex = leftIndex + 1; rightIndex < sortedProxies.length; rightIndex += 1) {
      const right = sortedProxies[rightIndex];

      if (right.aabb.min.x > left.aabb.max.x) {
        break;
      }

      if (!shouldGenerateBroadphasePair(left, right)) {
        continue;
      }

      pairs.push(createBroadphasePair({
        pairKey: createPairKey(left.colliderId, right.colliderId),
        pairKind: resolvePairKind(left, right),
        colliderAId: left.colliderId,
        colliderBId: right.colliderId,
        bodyAId: left.bodyId,
        bodyBId: right.bodyId,
        shapeAId: left.shapeId,
        shapeBId: right.shapeId,
        materialAId: left.materialId,
        materialBId: right.materialId,
        shapeAType: left.shapeType,
        shapeBType: right.shapeType,
        motionTypeA: left.motionType,
        motionTypeB: right.motionType,
        aabbA: left.aabb,
        aabbB: right.aabb
      }));
    }
  }

  return pairs;
}
