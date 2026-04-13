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

function shouldSwapPairOrder(options) {
  const leftRank = motionTypeRank(canonicalMotionType(options.motionTypeA, Boolean(options.bodyAId)));
  const rightRank = motionTypeRank(canonicalMotionType(options.motionTypeB, Boolean(options.bodyBId)));
  if (leftRank !== rightRank) {
    return leftRank < rightRank;
  }

  const leftColliderId = String(options.colliderAId ?? '').trim();
  const rightColliderId = String(options.colliderBId ?? '').trim();
  if (!leftColliderId || !rightColliderId) {
    return false;
  }

  return leftColliderId.localeCompare(rightColliderId) > 0;
}

function swapPairSides(options) {
  return {
    ...options,
    colliderAId: options.colliderBId,
    colliderBId: options.colliderAId,
    bodyAId: options.bodyBId,
    bodyBId: options.bodyAId,
    shapeAId: options.shapeBId,
    shapeBId: options.shapeAId,
    materialAId: options.materialBId,
    materialBId: options.materialAId,
    shapeAType: options.shapeBType,
    shapeBType: options.shapeAType,
    motionTypeA: options.motionTypeB,
    motionTypeB: options.motionTypeA,
    aabbA: options.aabbB,
    aabbB: options.aabbA
  };
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
    collisionLayer: Number(options.collisionLayer ?? 1),
    collisionMask: Number(options.collisionMask ?? 0x7fffffff),
    aabb: cloneAabb(options.aabb)
  };
}

export function cloneBroadphaseProxy(proxy) {
  return createBroadphaseProxy(proxy);
}

export function createBroadphasePair(options = {}) {
  const canonicalOptions = shouldSwapPairOrder(options)
    ? swapPairSides(options)
    : options;

  return {
    id: String(canonicalOptions.id ?? canonicalOptions.pairKey ?? '').trim() || createPairKey(canonicalOptions.colliderAId, canonicalOptions.colliderBId),
    pairKey: String(canonicalOptions.pairKey ?? '').trim() || createPairKey(canonicalOptions.colliderAId, canonicalOptions.colliderBId),
    type: 'potential-collision-pair',
    pairKind: String(canonicalOptions.pairKind ?? '').trim() || 'dynamic-dynamic',
    colliderAId: cloneNullableId(canonicalOptions.colliderAId),
    colliderBId: cloneNullableId(canonicalOptions.colliderBId),
    bodyAId: cloneNullableId(canonicalOptions.bodyAId),
    bodyBId: cloneNullableId(canonicalOptions.bodyBId),
    shapeAId: cloneNullableId(canonicalOptions.shapeAId),
    shapeBId: cloneNullableId(canonicalOptions.shapeBId),
    materialAId: cloneNullableId(canonicalOptions.materialAId),
    materialBId: cloneNullableId(canonicalOptions.materialBId),
    shapeAType: String(canonicalOptions.shapeAType ?? '').trim() || 'unknown',
    shapeBType: String(canonicalOptions.shapeBType ?? '').trim() || 'unknown',
    motionTypeA: canonicalMotionType(canonicalOptions.motionTypeA, Boolean(canonicalOptions.bodyAId)),
    motionTypeB: canonicalMotionType(canonicalOptions.motionTypeB, Boolean(canonicalOptions.bodyBId)),
    aabbA: cloneAabb(canonicalOptions.aabbA),
    aabbB: cloneAabb(canonicalOptions.aabbB)
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
