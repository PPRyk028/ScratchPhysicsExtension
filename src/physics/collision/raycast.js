import { combineAabbs, computeShapeWorldAabb, createAabbFromCenterHalfExtents, expandAabb, intersectRayAabb, testAabbOverlap } from './aabb.js';
import { runEpa, runGjk } from './gjk-epa.js';
import { getClosestPointOnSegment, getShapeWorldPose } from './support.js';
import { cloneQuat, createIdentityQuat, inverseRotateVec3ByQuat, rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, addVec3, cloneVec3, createVec3, dotVec3, lengthSquaredVec3, negateVec3, normalizeVec3, subtractVec3 } from '../math/vec3.js';

function createQueryEndPoint(origin, direction, maxDistance) {
  return addScaledVec3(origin, direction, maxDistance);
}

function cloneOptionalVec3(vector) {
  return vector ? cloneVec3(vector) : null;
}

function cloneVec3List(items) {
  return Array.isArray(items) ? items.map((item) => cloneVec3(item)) : [];
}

function createShapeHit(options = {}) {
  return {
    distance: Number(options.distance ?? 0),
    point: cloneVec3(options.point ?? createVec3()),
    normal: normalizeVec3(options.normal ?? createVec3(0, 1, 0), createVec3(0, 1, 0)),
    sweepPosition: cloneOptionalVec3(options.sweepPosition ?? null),
    algorithm: String(options.algorithm ?? 'query').trim() || 'query',
    featureId: String(options.featureId ?? '').trim() || 'feature:unknown',
    sampleId: String(options.sampleId ?? '').trim() || null
  };
}

function createBaseQueryResult(options = {}) {
  const origin = cloneVec3(options.origin ?? createVec3());
  const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
  const maxDistance = Number(options.maxDistance ?? 0);

  return {
    hit: Boolean(options.hit),
    origin,
    direction,
    maxDistance,
    endPoint: cloneVec3(options.endPoint ?? createQueryEndPoint(origin, direction, maxDistance)),
    distance: options.hit ? Number(options.distance ?? 0) : null,
    point: options.hit ? cloneOptionalVec3(options.point ?? createVec3()) : null,
    normal: options.hit ? cloneOptionalVec3(options.normal ?? createVec3(0, 1, 0)) : null,
    sweepPosition: options.hit ? cloneOptionalVec3(options.sweepPosition ?? createVec3()) : null,
    colliderId: options.hit ? String(options.colliderId ?? '').trim() || null : null,
    bodyId: options.hit ? String(options.bodyId ?? '').trim() || null : null,
    shapeId: options.hit ? String(options.shapeId ?? '').trim() || null : null,
    materialId: options.hit ? String(options.materialId ?? '').trim() || null : null,
    shapeType: options.hit ? String(options.shapeType ?? '').trim() || null : null,
    algorithm: options.hit ? String(options.algorithm ?? '').trim() || null : null,
    featureId: options.hit ? String(options.featureId ?? '').trim() || null : null,
    proxyCount: Math.max(0, Math.floor(Number(options.proxyCount ?? 0))),
    candidateCount: Math.max(0, Math.floor(Number(options.candidateCount ?? 0))),
    testedShapeCount: Math.max(0, Math.floor(Number(options.testedShapeCount ?? 0)))
  };
}

export function createRaycastResult(options = {}) {
  return createBaseQueryResult(options);
}

export function cloneRaycastResult(result) {
  if (!result) {
    return null;
  }

  return createRaycastResult(result);
}

export function createShapeCastResult(options = {}) {
  return {
    ...createBaseQueryResult(options),
    castType: String(options.castType ?? 'shape').trim() || 'shape',
    radius: Number(options.radius ?? 0),
    halfHeight: Number(options.halfHeight ?? 0),
    rotation: cloneQuat(options.rotation ?? createIdentityQuat()),
    sampleId: options.hit ? String(options.sampleId ?? '').trim() || null : null,
    sampleOrigins: cloneVec3List(options.sampleOrigins),
    sampleEndPoints: cloneVec3List(options.sampleEndPoints)
  };
}

export function cloneShapeCastResult(result) {
  if (!result) {
    return null;
  }

  return createShapeCastResult(result);
}

function chooseCloserHit(currentBest, candidate) {
  if (!candidate) {
    return currentBest;
  }

  if (!currentBest) {
    return candidate;
  }

  if (candidate.distance < currentBest.distance - 1e-8) {
    return candidate;
  }

  return currentBest;
}

function isSupportMappedShape(shape) {
  return shape?.type === 'box' ||
    shape?.type === 'sphere' ||
    shape?.type === 'capsule' ||
    shape?.type === 'convex-hull';
}

function createPoseAtDistance(origin, rotation, direction, distance) {
  return {
    position: addScaledVec3(origin, direction, distance),
    rotation: cloneQuat(rotation ?? createIdentityQuat())
  };
}

function createCastShape(castType, radius, halfHeight = 0) {
  if (castType === 'capsule') {
    return {
      type: 'capsule',
      geometry: {
        radius,
        halfHeight
      },
      localPose: {
        position: createVec3(),
        rotation: createIdentityQuat()
      }
    };
  }

  return {
    type: 'sphere',
    geometry: {
      radius
    },
    localPose: {
      position: createVec3(),
      rotation: createIdentityQuat()
    }
  };
}

function createSupportMappedHit(queryShape, queryPose, targetShape, targetPose, distance, algorithm) {
  const gjk = runGjk(queryShape, queryPose, targetShape, targetPose);
  if (!gjk.hit) {
    return null;
  }

  const epa = runEpa(queryShape, queryPose, targetShape, targetPose, gjk.simplex);
  if (!epa) {
    return null;
  }

  return createShapeHit({
    distance,
    point: epa.contactPosition,
    normal: negateVec3(epa.normal),
    sweepPosition: queryPose.position,
    algorithm,
    featureId: 'gjk-epa:toi'
  });
}

export function castConvexShapesWithToi(options = {}) {
  const queryShape = options.queryShape ?? null;
  const targetShape = options.targetShape ?? null;
  if (!isSupportMappedShape(queryShape) || !isSupportMappedShape(targetShape)) {
    return null;
  }

  const origin = cloneVec3(options.origin ?? createVec3());
  const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
  const maxDistance = Number.isFinite(Number(options.maxDistance)) ? Math.max(0, Number(options.maxDistance)) : 0;
  const rotation = cloneQuat(options.rotation ?? createIdentityQuat());
  const targetPose = options.targetPose ?? {
    position: createVec3(),
    rotation: createIdentityQuat()
  };
  const targetAabb = computeShapeWorldAabb(targetShape, targetPose);
  const distanceTolerance = Math.max(1e-5, Number(options.distanceTolerance ?? Math.max(maxDistance / 16384, 5e-4)));
  const maxDepth = Math.max(8, Math.floor(Number(options.maxDepth ?? 22)));
  const overlapCache = new Map();

  function getQueryPose(distance) {
    return createPoseAtDistance(origin, rotation, direction, distance);
  }

  function getOverlapSample(distance) {
    const key = distance.toFixed(8);
    if (overlapCache.has(key)) {
      return overlapCache.get(key);
    }

    const queryPose = getQueryPose(distance);
    const gjk = runGjk(queryShape, queryPose, targetShape, targetPose);
    const value = {
      hit: gjk.hit,
      queryPose,
      gjk
    };
    overlapCache.set(key, value);
    return value;
  }

  function resolveHit(distance) {
    const queryPose = getQueryPose(distance);
    return createSupportMappedHit(queryShape, queryPose, targetShape, targetPose, distance, 'convex-toi-v1');
  }

  function recurse(distanceStart, distanceEnd, depth) {
    const sweepAabb = computeSweptShapeAabb({
      shape: queryShape,
      origin: addScaledVec3(origin, direction, distanceStart),
      direction,
      maxDistance: Math.max(0, distanceEnd - distanceStart),
      rotation
    });

    if (!sweepAabb || !targetAabb || !testAabbOverlap(sweepAabb, targetAabb)) {
      return null;
    }

    const startSample = getOverlapSample(distanceStart);
    if (startSample.hit) {
      return resolveHit(distanceStart);
    }

    const endSample = getOverlapSample(distanceEnd);
    const segmentLength = distanceEnd - distanceStart;
    if (depth >= maxDepth || segmentLength <= distanceTolerance) {
      const sampleDistances = [distanceStart, (distanceStart + distanceEnd) / 2, distanceEnd];
      for (const distance of sampleDistances) {
        const sample = getOverlapSample(distance);
        if (sample.hit) {
          return resolveHit(distance);
        }
      }

      return null;
    }

    const midDistance = (distanceStart + distanceEnd) / 2;
    const leftHit = recurse(distanceStart, midDistance, depth + 1);
    if (leftHit) {
      return leftHit;
    }

    if (endSample.hit) {
      const rightHit = recurse(midDistance, distanceEnd, depth + 1);
      if (rightHit) {
        return rightHit;
      }
      return resolveHit(distanceEnd);
    }

    return recurse(midDistance, distanceEnd, depth + 1);
  }

  return recurse(0, maxDistance, 0);
}

function raycastSphere(origin, direction, maxDistance, center, radius) {
  const offset = subtractVec3(origin, center);
  const b = dotVec3(offset, direction);
  const c = dotVec3(offset, offset) - radius * radius;

  if (c > 0 && b > 0) {
    return null;
  }

  const discriminant = b * b - c;
  if (discriminant < 0) {
    return null;
  }

  let distance = -b - Math.sqrt(discriminant);
  if (distance < 0) {
    distance = 0;
  }

  if (distance > maxDistance) {
    return null;
  }

  const point = addScaledVec3(origin, direction, distance);
  const normal = normalizeVec3(subtractVec3(point, center), negateVec3(direction));

  return createShapeHit({
    distance,
    point,
    normal,
    sweepPosition: point,
    algorithm: 'sphere-raycast-v1',
    featureId: 'sphere:surface'
  });
}

function raycastBox(shape, worldPose, origin, direction, maxDistance) {
  const shapePose = getShapeWorldPose(shape, worldPose);
  const localOrigin = inverseRotateVec3ByQuat(shapePose.rotation, subtractVec3(origin, shapePose.position));
  const localDirection = inverseRotateVec3ByQuat(shapePose.rotation, direction);
  const localAabb = createAabbFromCenterHalfExtents(createVec3(), shape.geometry.halfExtents);
  const localHit = intersectRayAabb(localOrigin, localDirection, maxDistance, localAabb);

  if (!localHit) {
    return null;
  }

  return createShapeHit({
    distance: localHit.distance,
    point: addVec3(shapePose.position, rotateVec3ByQuat(shapePose.rotation, localHit.point)),
    normal: rotateVec3ByQuat(shapePose.rotation, localHit.normal),
    sweepPosition: addScaledVec3(origin, direction, localHit.distance),
    algorithm: 'box-raycast-v1',
    featureId: `face:${localHit.axis}:${localHit.normal[localHit.axis] >= 0 ? '+' : '-'}`
  });
}

function raycastCapsule(shape, worldPose, origin, direction, maxDistance) {
  const shapePose = getShapeWorldPose(shape, worldPose);
  const localOrigin = inverseRotateVec3ByQuat(shapePose.rotation, subtractVec3(origin, shapePose.position));
  const localDirection = inverseRotateVec3ByQuat(shapePose.rotation, direction);
  const radius = Number(shape.geometry.radius ?? 0);
  const halfHeight = Number(shape.geometry.halfHeight ?? 0);
  const top = createVec3(0, halfHeight, 0);
  const bottom = createVec3(0, -halfHeight, 0);
  const closestPoint = getClosestPointOnSegment(localOrigin, bottom, top);
  const insideOffset = subtractVec3(localOrigin, closestPoint);

  if (lengthSquaredVec3(insideOffset) <= radius * radius) {
    return createShapeHit({
      distance: 0,
      point: cloneVec3(origin),
      normal: rotateVec3ByQuat(shapePose.rotation, normalizeVec3(insideOffset, negateVec3(localDirection))),
      sweepPosition: cloneVec3(origin),
      algorithm: 'capsule-raycast-v1',
      featureId: 'capsule:inside'
    });
  }

  let bestHit = null;
  const radialA = localDirection.x * localDirection.x + localDirection.z * localDirection.z;
  if (radialA > 1e-12) {
    const radialB = localOrigin.x * localDirection.x + localOrigin.z * localDirection.z;
    const radialC = localOrigin.x * localOrigin.x + localOrigin.z * localOrigin.z - radius * radius;
    const discriminant = radialB * radialB - radialA * radialC;

    if (discriminant >= 0) {
      const sqrtDiscriminant = Math.sqrt(discriminant);
      for (const candidateDistance of [
        (-radialB - sqrtDiscriminant) / radialA,
        (-radialB + sqrtDiscriminant) / radialA
      ]) {
        if (candidateDistance < 0 || candidateDistance > maxDistance) {
          continue;
        }

        const candidatePoint = addScaledVec3(localOrigin, localDirection, candidateDistance);
        if (candidatePoint.y < -halfHeight - 1e-8 || candidatePoint.y > halfHeight + 1e-8) {
          continue;
        }

        bestHit = chooseCloserHit(bestHit, createShapeHit({
          distance: candidateDistance,
          point: addVec3(shapePose.position, rotateVec3ByQuat(shapePose.rotation, candidatePoint)),
          normal: rotateVec3ByQuat(shapePose.rotation, normalizeVec3(createVec3(candidatePoint.x, 0, candidatePoint.z), negateVec3(localDirection))),
          sweepPosition: addScaledVec3(origin, direction, candidateDistance),
          algorithm: 'capsule-raycast-v1',
          featureId: 'capsule:side'
        }));
      }
    }
  }

  for (const [capName, capCenter] of [['top', top], ['bottom', bottom]]) {
    const capHit = raycastSphere(localOrigin, localDirection, maxDistance, capCenter, radius);
    if (!capHit) {
      continue;
    }

    bestHit = chooseCloserHit(bestHit, createShapeHit({
      distance: capHit.distance,
      point: addVec3(shapePose.position, rotateVec3ByQuat(shapePose.rotation, capHit.point)),
      normal: rotateVec3ByQuat(shapePose.rotation, capHit.normal),
      sweepPosition: addScaledVec3(origin, direction, capHit.distance),
      algorithm: 'capsule-raycast-v1',
      featureId: `capsule:${capName}`
    }));
  }

  return bestHit;
}

function raycastConvexHullFallback(shape, worldPose, origin, direction, maxDistance) {
  const aabb = computeShapeWorldAabb(shape, worldPose);
  const hit = intersectRayAabb(origin, direction, maxDistance, aabb);
  if (!hit) {
    return null;
  }

  return createShapeHit({
    distance: hit.distance,
    point: hit.point,
    normal: hit.normal,
    sweepPosition: addScaledVec3(origin, direction, hit.distance),
    algorithm: 'convex-aabb-raycast-v1',
    featureId: `aabb:${hit.axis}`
  });
}

export function raycastShape(options = {}) {
  const shape = options.shape ?? null;
  if (!shape) {
    return null;
  }

  const origin = cloneVec3(options.origin ?? createVec3());
  const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
  const maxDistance = Number.isFinite(Number(options.maxDistance)) ? Math.max(0, Number(options.maxDistance)) : Number.POSITIVE_INFINITY;
  const worldPose = options.worldPose ?? {
    position: createVec3(),
    rotation: undefined
  };

  if (shape.type === 'box') {
    return raycastBox(shape, worldPose, origin, direction, maxDistance);
  }

  if (shape.type === 'sphere') {
    return raycastSphere(origin, direction, maxDistance, getShapeWorldPose(shape, worldPose).position, Number(shape.geometry.radius ?? 0));
  }

  if (shape.type === 'capsule') {
    return raycastCapsule(shape, worldPose, origin, direction, maxDistance);
  }

  if (shape.type === 'convex-hull') {
    return raycastConvexHullFallback(shape, worldPose, origin, direction, maxDistance);
  }

  return null;
}

function createInflatedBoxShape(shape, inflationRadius) {
  return {
    ...shape,
    geometry: {
      ...shape.geometry,
      halfExtents: createVec3(
        Number(shape.geometry.halfExtents.x ?? 0) + inflationRadius,
        Number(shape.geometry.halfExtents.y ?? 0) + inflationRadius,
        Number(shape.geometry.halfExtents.z ?? 0) + inflationRadius
      )
    }
  };
}

function createInflatedSphereShape(shape, inflationRadius) {
  return {
    ...shape,
    geometry: {
      ...shape.geometry,
      radius: Number(shape.geometry.radius ?? 0) + inflationRadius
    }
  };
}

function createInflatedCapsuleShape(shape, inflationRadius) {
  return {
    ...shape,
    geometry: {
      ...shape.geometry,
      radius: Number(shape.geometry.radius ?? 0) + inflationRadius
    }
  };
}

function sphereCastShapeHit(options = {}) {
  const origin = cloneVec3(options.origin ?? createVec3());
  const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
  const radius = Math.max(0, Number(options.radius ?? 0));
  const maxDistance = Number.isFinite(Number(options.maxDistance)) ? Math.max(0, Number(options.maxDistance)) : Number.POSITIVE_INFINITY;
  const shape = options.shape ?? null;
  const worldPose = options.worldPose ?? {
    position: createVec3(),
    rotation: createIdentityQuat()
  };

  if (!shape) {
    return null;
  }

  if (isSupportMappedShape(shape)) {
    const toiHit = castConvexShapesWithToi({
      queryShape: createCastShape('sphere', radius),
      targetShape: shape,
      targetPose: worldPose,
      origin,
      rotation: createIdentityQuat(),
      direction,
      maxDistance,
      distanceTolerance: options.distanceTolerance,
      maxDepth: options.maxDepth
    });

    if (toiHit) {
      return createShapeHit({
        ...toiHit,
        sampleId: options.sampleId ?? 'center'
      });
    }
  }

  let targetHit = null;
  if (shape.type === 'box') {
    targetHit = raycastBox(createInflatedBoxShape(shape, radius), worldPose, origin, direction, maxDistance);
  } else if (shape.type === 'sphere') {
    const sphereCenter = getShapeWorldPose(shape, worldPose).position;
    targetHit = raycastSphere(origin, direction, maxDistance, sphereCenter, Number(shape.geometry.radius ?? 0) + radius);
  } else if (shape.type === 'capsule') {
    targetHit = raycastCapsule(createInflatedCapsuleShape(shape, radius), worldPose, origin, direction, maxDistance);
  } else if (shape.type === 'convex-hull') {
    const expandedAabb = expandAabb(computeShapeWorldAabb(shape, worldPose), createVec3(radius, radius, radius));
    const aabbHit = intersectRayAabb(origin, direction, maxDistance, expandedAabb);
    if (aabbHit) {
      targetHit = createShapeHit({
        distance: aabbHit.distance,
        point: aabbHit.point,
        normal: aabbHit.normal,
        sweepPosition: addScaledVec3(origin, direction, aabbHit.distance),
        algorithm: 'convex-aabb-sphere-cast-v1',
        featureId: `aabb:${aabbHit.axis}`
      });
    }
  }

  if (!targetHit) {
    return null;
  }

  const sweepPosition = cloneVec3(targetHit.sweepPosition ?? addScaledVec3(origin, direction, targetHit.distance));
  return createShapeHit({
    distance: targetHit.distance,
    point: addScaledVec3(sweepPosition, targetHit.normal, -radius),
    normal: targetHit.normal,
    sweepPosition,
    algorithm: targetHit.algorithm,
    featureId: targetHit.featureId,
    sampleId: options.sampleId ?? 'center'
  });
}

function getCapsuleCastSamples(origin, halfHeight, rotation) {
  if (halfHeight <= 1e-8) {
    return [
      {
        id: 'center',
        position: cloneVec3(origin)
      }
    ];
  }

  const axisOffset = rotateVec3ByQuat(rotation ?? createIdentityQuat(), createVec3(0, halfHeight, 0));
  return [
    {
      id: 'center',
      position: cloneVec3(origin)
    },
    {
      id: 'top',
      position: addVec3(origin, axisOffset)
    },
    {
      id: 'bottom',
      position: addScaledVec3(origin, axisOffset, -1)
    }
  ];
}

export function sphereCastShape(options = {}) {
  return sphereCastShapeHit(options);
}

export function capsuleCastShape(options = {}) {
  const origin = cloneVec3(options.origin ?? createVec3());
  const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
  const maxDistance = Number.isFinite(Number(options.maxDistance)) ? Math.max(0, Number(options.maxDistance)) : Number.POSITIVE_INFINITY;
  const radius = Math.max(0, Number(options.radius ?? 0));
  const halfHeight = Math.max(0, Number(options.halfHeight ?? 0));
  const rotation = cloneQuat(options.rotation ?? createIdentityQuat());
  const shape = options.shape ?? null;
  if (!shape) {
    return null;
  }

  if (isSupportMappedShape(shape)) {
    const toiHit = castConvexShapesWithToi({
      queryShape: createCastShape('capsule', radius, halfHeight),
      targetShape: shape,
      targetPose: options.worldPose ?? {
        position: createVec3(),
        rotation: createIdentityQuat()
      },
      origin,
      rotation,
      direction,
      maxDistance,
      distanceTolerance: options.distanceTolerance,
      maxDepth: options.maxDepth
    });

    if (toiHit) {
      return createShapeHit({
        ...toiHit,
        sampleId: 'capsule'
      });
    }
  }

  const sampleOrigins = getCapsuleCastSamples(origin, halfHeight, rotation);
  let bestHit = null;

  for (const sample of sampleOrigins) {
    const sampleHit = sphereCastShapeHit({
      ...options,
      origin: sample.position,
      direction,
      maxDistance,
      radius,
      sampleId: sample.id
    });

    if (!sampleHit) {
      continue;
    }

    if (!bestHit || sampleHit.distance < bestHit.distance - 1e-8) {
      bestHit = {
        ...sampleHit,
        sweepPosition: addScaledVec3(origin, direction, sampleHit.distance),
        sampleId: sample.id
      };
    }
  }

  if (!bestHit) {
    return null;
  }

  return createShapeHit({
    distance: bestHit.distance,
    point: bestHit.point,
    normal: bestHit.normal,
    sweepPosition: bestHit.sweepPosition,
    algorithm: `capsule-cast-fallback-v1:${bestHit.algorithm}`,
    featureId: bestHit.featureId,
    sampleId: bestHit.sampleId
  });
}

export function computeSweptShapeAabb(options = {}) {
  const shape = options.shape ?? null;
  if (!shape) {
    return null;
  }

  const origin = cloneVec3(options.origin ?? createVec3());
  const direction = normalizeVec3(options.direction ?? createVec3(0, -1, 0), createVec3(0, -1, 0));
  const maxDistance = Number.isFinite(Number(options.maxDistance)) ? Math.max(0, Number(options.maxDistance)) : 0;
  const rotation = cloneQuat(options.rotation ?? createIdentityQuat());
  const worldPose = {
    position: origin,
    rotation
  };
  const endPose = {
    position: addScaledVec3(origin, direction, maxDistance),
    rotation
  };

  return combineAabbs(
    computeShapeWorldAabb(shape, worldPose),
    computeShapeWorldAabb(shape, endPose)
  );
}
