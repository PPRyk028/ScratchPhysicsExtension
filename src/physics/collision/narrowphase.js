import { cloneVec3, createVec3, lengthVec3, negateVec3, normalizeVec3, subtractVec3 } from '../math/vec3.js';
import { getAabbOverlap } from './aabb.js';
import { buildConvexContactManifold, collideConvexPairWithGjkEpa } from './gjk-epa.js';
import { clampPointToAabb, getCapsuleSegmentEndpoints, getClosestPointOnSegment, getShapeSupportPoint, getShapeWorldCenter } from './support.js';

function cloneContact(contact) {
  return {
    id: contact.id,
    position: cloneVec3(contact.position),
    normal: cloneVec3(contact.normal),
    penetration: contact.penetration,
    separation: contact.separation,
    featureId: contact.featureId
  };
}

function chooseDefaultNormal(pair) {
  const overlap = getAabbOverlap(pair?.aabbA, pair?.aabbB);
  if (!overlap) {
    return {
      normal: createVec3(0, 1, 0),
      penetration: 0,
      axis: 'y'
    };
  }

  const axes = ['x', 'y', 'z'];
  let axis = axes[0];
  let bestOverlap = overlap[axis];
  let bestDelta = Math.abs(pair.aabbB.center[axis] - pair.aabbA.center[axis]);

  for (let index = 1; index < axes.length; index += 1) {
    const candidateAxis = axes[index];
    const candidateOverlap = overlap[candidateAxis];
    const candidateDelta = Math.abs(pair.aabbB.center[candidateAxis] - pair.aabbA.center[candidateAxis]);

    if (candidateOverlap < bestOverlap - 1e-8) {
      axis = candidateAxis;
      bestOverlap = candidateOverlap;
      bestDelta = candidateDelta;
      continue;
    }

    if (Math.abs(candidateOverlap - bestOverlap) <= 1e-8 && candidateDelta > bestDelta + 1e-8) {
      axis = candidateAxis;
      bestOverlap = candidateOverlap;
      bestDelta = candidateDelta;
    }
  }

  const delta = pair.aabbB.center[axis] - pair.aabbA.center[axis];
  const sign = delta >= 0 ? 1 : -1;
  return {
    normal: axis === 'x'
      ? createVec3(sign, 0, 0)
      : axis === 'y'
        ? createVec3(0, sign, 0)
        : createVec3(0, 0, sign),
    penetration: overlap[axis],
    axis
  };
}

function createContactPairCore(pair, options = {}) {
  const contacts = Array.isArray(options.contacts) ? options.contacts : [];
  return createContactPair({
    id: `${pair.pairKey}:contact-pair`,
    pairKey: pair.pairKey,
    algorithm: options.algorithm,
    status: options.status ?? 'touching',
    pairKind: pair.pairKind,
    colliderAId: pair.colliderAId,
    colliderBId: pair.colliderBId,
    bodyAId: pair.bodyAId,
    bodyBId: pair.bodyBId,
    shapeAType: pair.shapeAType,
    shapeBType: pair.shapeBType,
    normal: options.normal,
    penetration: options.penetration,
    contactCount: contacts.length,
    contacts
  });
}

function createSingleContactPair(pair, options = {}) {
  const normal = cloneVec3(options.normal ?? createVec3(0, 1, 0));
  const penetration = Number(options.penetration ?? 0);
  const contactPosition = cloneVec3(options.position ?? createVec3());

  return createContactPairCore(pair, {
    algorithm: options.algorithm,
    normal,
    penetration,
    contacts: [
      {
        id: `${pair.pairKey}:contact-0`,
        position: contactPosition,
        normal,
        penetration,
        separation: -penetration,
        featureId: options.featureId ?? 'contact:0'
      }
    ]
  });
}

function createManifoldContactPair(pair, options = {}) {
  const normal = cloneVec3(options.normal ?? createVec3(0, 1, 0));
  const penetration = Number(options.penetration ?? 0);
  const contacts = Array.isArray(options.contacts) ? options.contacts : [];

  return createContactPairCore(pair, {
    algorithm: options.algorithm,
    normal,
    penetration,
    contacts: contacts.map((contact, index) => ({
      id: `${pair.pairKey}:contact-${index}`,
      position: cloneVec3(contact.position),
      normal,
      penetration: Number(contact.penetration ?? penetration),
      separation: Number(contact.separation ?? -Number(contact.penetration ?? penetration)),
      featureId: contact.featureId ?? `contact:${index}`
    }))
  });
}

function isSupportMappedShape(shapeType) {
  return shapeType === 'box' || shapeType === 'sphere' || shapeType === 'capsule' || shapeType === 'convex-hull';
}

function collideSphereSpherePair(pair, shapeA, poseA, shapeB, poseB) {
  const centerA = getShapeWorldCenter(shapeA, poseA);
  const centerB = getShapeWorldCenter(shapeB, poseB);
  const delta = subtractVec3(centerB, centerA);
  const distance = lengthVec3(delta);
  const radiusSum = shapeA.geometry.radius + shapeB.geometry.radius;
  if (distance > radiusSum + 1e-8) {
    return null;
  }

  const fallback = chooseDefaultNormal(pair);
  const normal = normalizeVec3(delta, fallback.normal);
  const penetration = Math.max(radiusSum - distance, fallback.penetration);
  const supportA = getShapeSupportPoint(shapeA, poseA, normal);
  const supportB = getShapeSupportPoint(shapeB, poseB, negateVec3(normal));
  const contactPosition = createVec3(
    (supportA.x + supportB.x) / 2,
    (supportA.y + supportB.y) / 2,
    (supportA.z + supportB.z) / 2
  );

  return createSingleContactPair(pair, {
    algorithm: 'sphere-sphere-v1',
    normal,
    penetration,
    position: contactPosition,
    featureId: 'sphere-sphere'
  });
}

function collideSphereBoxPair(pair, sphereShape, spherePose, boxShape, boxPose, sphereIsA) {
  const sphereCenter = getShapeWorldCenter(sphereShape, spherePose);
  const boxAabb = sphereIsA ? pair.aabbB : pair.aabbA;
  const closestPoint = clampPointToAabb(sphereCenter, boxAabb);
  const outward = subtractVec3(sphereCenter, closestPoint);
  const distance = lengthVec3(outward);
  const fallback = chooseDefaultNormal(pair);
  const normalFromBoxToSphere = distance > 1e-8 ? normalizeVec3(outward, fallback.normal) : fallback.normal;
  const normal = sphereIsA ? negateVec3(normalFromBoxToSphere) : normalFromBoxToSphere;
  const penetration = Math.max(sphereShape.geometry.radius - distance, fallback.penetration);
  const sphereSurface = getShapeSupportPoint(sphereShape, spherePose, sphereIsA ? normal : negateVec3(normal));
  const contactPosition = createVec3(
    (closestPoint.x + sphereSurface.x) / 2,
    (closestPoint.y + sphereSurface.y) / 2,
    (closestPoint.z + sphereSurface.z) / 2
  );

  if (distance > sphereShape.geometry.radius + 1e-8 && fallback.penetration <= 0) {
    return null;
  }

  return createSingleContactPair(pair, {
    algorithm: 'sphere-box-v1',
    normal,
    penetration,
    position: contactPosition,
    featureId: sphereIsA ? 'sphere-box:a' : 'sphere-box:b'
  });
}

function collideCapsuleSpherePair(pair, capsuleShape, capsulePose, sphereShape, spherePose, capsuleIsA) {
  const sphereCenter = getShapeWorldCenter(sphereShape, spherePose);
  const capsuleSegment = getCapsuleSegmentEndpoints(capsuleShape, capsulePose);
  const closestPoint = getClosestPointOnSegment(sphereCenter, capsuleSegment.start, capsuleSegment.end);
  const delta = subtractVec3(sphereCenter, closestPoint);
  const distance = lengthVec3(delta);
  const radiusSum = capsuleShape.geometry.radius + sphereShape.geometry.radius;
  if (distance > radiusSum + 1e-8) {
    return null;
  }

  const fallback = chooseDefaultNormal(pair);
  const normalFromCapsuleToSphere = normalizeVec3(delta, fallback.normal);
  const normal = capsuleIsA ? normalFromCapsuleToSphere : negateVec3(normalFromCapsuleToSphere);
  const penetration = Math.max(radiusSum - distance, fallback.penetration);
  const capsuleSurface = getShapeSupportPoint(capsuleShape, capsulePose, capsuleIsA ? normal : negateVec3(normal));
  const sphereSurface = getShapeSupportPoint(sphereShape, spherePose, capsuleIsA ? negateVec3(normal) : normal);
  const contactPosition = createVec3(
    (capsuleSurface.x + sphereSurface.x) / 2,
    (capsuleSurface.y + sphereSurface.y) / 2,
    (capsuleSurface.z + sphereSurface.z) / 2
  );

  return createSingleContactPair(pair, {
    algorithm: 'capsule-sphere-v1',
    normal,
    penetration,
    position: contactPosition,
    featureId: capsuleIsA ? 'capsule-sphere:a' : 'capsule-sphere:b'
  });
}

function collideGjkEpaPair(pair, shapeA, poseA, shapeB, poseB) {
  if (!isSupportMappedShape(shapeA.type) || !isSupportMappedShape(shapeB.type)) {
    return null;
  }

  const result = collideConvexPairWithGjkEpa(shapeA, poseA, shapeB, poseB);
  if (!result || result.penetration <= 0) {
    return null;
  }

  const contacts = buildConvexContactManifold(shapeA, poseA, shapeB, poseB, result);
  if (contacts.length === 0) {
    return createSingleContactPair(pair, {
      algorithm: 'gjk-epa-manifold-v1',
      normal: result.normal,
      penetration: result.penetration,
      position: result.contactPosition,
      featureId: 'gjk:contact'
    });
  }

  return createManifoldContactPair(pair, {
    algorithm: 'gjk-epa-manifold-v1',
    normal: result.normal,
    penetration: Math.max(...contacts.map((contact) => Number(contact.penetration ?? 0)), result.penetration),
    contacts
  });
}

function collidePair(pair, shapeA, poseA, shapeB, poseB) {
  if (!shapeA || !shapeB || !poseA || !poseB) {
    return null;
  }

  return collideGjkEpaPair(pair, shapeA, poseA, shapeB, poseB);
}

export function createContactPair(options = {}) {
  return {
    id: String(options.id ?? options.pairKey ?? '').trim() || String(options.pairKey ?? '').trim(),
    pairKey: String(options.pairKey ?? '').trim() || null,
    algorithm: String(options.algorithm ?? '').trim() || 'unknown',
    status: String(options.status ?? '').trim() || 'unknown',
    pairKind: String(options.pairKind ?? '').trim() || 'dynamic-dynamic',
    colliderAId: String(options.colliderAId ?? '').trim() || null,
    colliderBId: String(options.colliderBId ?? '').trim() || null,
    bodyAId: String(options.bodyAId ?? '').trim() || null,
    bodyBId: String(options.bodyBId ?? '').trim() || null,
    shapeAType: String(options.shapeAType ?? '').trim() || 'unknown',
    shapeBType: String(options.shapeBType ?? '').trim() || 'unknown',
    friction: Number(options.friction ?? 0.5),
    restitution: Number(options.restitution ?? 0),
    restitutionThreshold: Number(options.restitutionThreshold ?? 1),
    normal: cloneVec3(options.normal ?? createVec3()),
    penetration: Number(options.penetration ?? 0),
    contactCount: Number(options.contactCount ?? (Array.isArray(options.contacts) ? options.contacts.length : 0)),
    contacts: Array.isArray(options.contacts) ? options.contacts.map((contact) => cloneContact(contact)) : []
  };
}

export function cloneContactPair(contactPair) {
  return createContactPair(contactPair);
}

export function runNarrowphase(pairs, options = {}) {
  const contactPairs = [];
  const unsupportedPairs = [];
  const getShape = typeof options.getShape === 'function' ? options.getShape : () => null;
  const getPose = typeof options.getPose === 'function' ? options.getPose : () => null;

  for (const pair of Array.isArray(pairs) ? pairs : []) {
    const shapeA = getShape(pair.shapeAId);
    const shapeB = getShape(pair.shapeBId);
    const poseA = getPose(pair.colliderAId);
    const poseB = getPose(pair.colliderBId);
    const contactPair = collidePair(pair, shapeA, poseA, shapeB, poseB);

    if (contactPair) {
      contactPairs.push(contactPair);
      continue;
    }

    unsupportedPairs.push(pair.pairKey);
  }

  return {
    contactPairs,
    summary: {
      pairCount: Array.isArray(pairs) ? pairs.length : 0,
      contactCount: contactPairs.length,
      unsupportedPairCount: unsupportedPairs.length,
      unsupportedPairs,
      algorithms: contactPairs.reduce((counts, contactPair) => {
        counts[contactPair.algorithm] = (counts[contactPair.algorithm] ?? 0) + 1;
        return counts;
      }, {})
    }
  };
}
