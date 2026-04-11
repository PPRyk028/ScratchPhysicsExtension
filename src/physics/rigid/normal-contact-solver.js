import { createIdentityQuat, inverseRotateVec3ByQuat, rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, addVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';
import { createTangentBasis } from '../collision/support.js';

function createEmptySolverStats(iterations) {
  return {
    iterations,
    manifoldCount: 0,
    warmStartedContactCount: 0,
    solvedContactCount: 0,
    solvedTangentContactCount: 0,
    restitutionContactCount: 0,
    skippedContactCount: 0,
    impulsesApplied: 0,
    frictionImpulsesApplied: 0,
    positionCorrections: 0,
    maxPenetration: 0
  };
}

function getDynamicBody(bodyRegistry, bodyId) {
  if (!bodyId) {
    return null;
  }

  const body = bodyRegistry.getMutable(bodyId);
  if (!body || !body.enabled || body.motionType !== 'dynamic' || body.sleeping) {
    return null;
  }

  return body;
}

function applyInverseInertiaWorld(body, worldVector) {
  if (!body || body.motionType !== 'dynamic') {
    return createVec3();
  }

  const rotation = body.rotation ?? createIdentityQuat();
  const localVector = inverseRotateVec3ByQuat(rotation, worldVector ?? createVec3());
  const localResult = createVec3(
    localVector.x * Number(body.inverseInertia?.x ?? 0),
    localVector.y * Number(body.inverseInertia?.y ?? 0),
    localVector.z * Number(body.inverseInertia?.z ?? 0)
  );
  return rotateVec3ByQuat(rotation, localResult);
}

function getContactOffset(body, contactPosition) {
  if (!body) {
    return createVec3();
  }

  return subtractVec3(contactPosition, body.position);
}

function getVelocityAtPoint(body, contactPosition) {
  if (!body) {
    return createVec3();
  }

  const offset = getContactOffset(body, contactPosition);
  return addVec3(body.linearVelocity, crossVec3(body.angularVelocity, offset));
}

function getRelativeVelocity(bodyA, bodyB, contactPosition) {
  const velocityA = getVelocityAtPoint(bodyA, contactPosition);
  const velocityB = getVelocityAtPoint(bodyB, contactPosition);
  return subtractVec3(velocityB, velocityA);
}

function applyImpulse(bodyA, bodyB, contactPosition, impulse) {
  if (bodyA) {
    const offsetA = getContactOffset(bodyA, contactPosition);
    const impulseA = scaleVec3(impulse, -1);
    bodyA.linearVelocity = addScaledVec3(bodyA.linearVelocity, impulseA, bodyA.inverseMass);
    bodyA.angularVelocity = addVec3(bodyA.angularVelocity, applyInverseInertiaWorld(bodyA, crossVec3(offsetA, impulseA)));
  }

  if (bodyB) {
    const offsetB = getContactOffset(bodyB, contactPosition);
    bodyB.linearVelocity = addScaledVec3(bodyB.linearVelocity, impulse, bodyB.inverseMass);
    bodyB.angularVelocity = addVec3(bodyB.angularVelocity, applyInverseInertiaWorld(bodyB, crossVec3(offsetB, impulse)));
  }
}

function applyPositionCorrection(bodyA, bodyB, direction, correctionMagnitude) {
  if (bodyA) {
    bodyA.position = addScaledVec3(bodyA.position, direction, -correctionMagnitude * bodyA.inverseMass);
  }

  if (bodyB) {
    bodyB.position = addScaledVec3(bodyB.position, direction, correctionMagnitude * bodyB.inverseMass);
  }
}

function computeEffectiveMass(bodyA, bodyB, contactPosition, direction) {
  let inverseMassSum = (bodyA?.inverseMass ?? 0) + (bodyB?.inverseMass ?? 0);

  if (bodyA) {
    const offsetA = getContactOffset(bodyA, contactPosition);
    const angularMassA = crossVec3(applyInverseInertiaWorld(bodyA, crossVec3(offsetA, direction)), offsetA);
    inverseMassSum += dotVec3(direction, angularMassA);
  }

  if (bodyB) {
    const offsetB = getContactOffset(bodyB, contactPosition);
    const angularMassB = crossVec3(applyInverseInertiaWorld(bodyB, crossVec3(offsetB, direction)), offsetB);
    inverseMassSum += dotVec3(direction, angularMassB);
  }

  return inverseMassSum;
}

function applyWarmStart(bodyA, bodyB, manifold, contact, tangentBasis, stats) {
  if (contact.accumulatedNormalImpulse > 0) {
    applyImpulse(bodyA, bodyB, contact.position, scaleVec3(manifold.normal, contact.accumulatedNormalImpulse));
    stats.warmStartedContactCount += 1;
  }

  if (Math.abs(contact.accumulatedTangentImpulseA) > 1e-8) {
    applyImpulse(bodyA, bodyB, contact.position, scaleVec3(tangentBasis.tangentA, contact.accumulatedTangentImpulseA));
    stats.frictionImpulsesApplied += Math.abs(contact.accumulatedTangentImpulseA);
  }

  if (Math.abs(contact.accumulatedTangentImpulseB) > 1e-8) {
    applyImpulse(bodyA, bodyB, contact.position, scaleVec3(tangentBasis.tangentB, contact.accumulatedTangentImpulseB));
    stats.frictionImpulsesApplied += Math.abs(contact.accumulatedTangentImpulseB);
  }
}

function solveNormalImpulse(bodyA, bodyB, manifold, contact, deltaTime, baumgarte, allowedPenetration, stats) {
  const inverseMassSum = computeEffectiveMass(bodyA, bodyB, contact.position, manifold.normal);
  if (inverseMassSum <= 1e-8) {
    stats.skippedContactCount += 1;
    return;
  }

  const relativeVelocity = getRelativeVelocity(bodyA, bodyB, contact.position);
  const relativeNormalVelocity = dotVec3(relativeVelocity, manifold.normal);
  const separationWithSlop = Math.min(0, contact.separation + allowedPenetration);
  const positionBias = (baumgarte * separationWithSlop) / deltaTime;
  const restitutionThreshold = Number(manifold.restitutionThreshold ?? 1);
  const bounceVelocity = manifold.restitution > 0 && relativeNormalVelocity < -restitutionThreshold
    ? -manifold.restitution * relativeNormalVelocity
    : 0;

  if (bounceVelocity > 0) {
    stats.restitutionContactCount += 1;
  }

  const impulseDelta = -(relativeNormalVelocity - bounceVelocity + positionBias) / inverseMassSum;
  const previousImpulse = contact.accumulatedNormalImpulse;
  const nextImpulse = Math.max(previousImpulse + impulseDelta, 0);
  const appliedImpulse = nextImpulse - previousImpulse;

  if (Math.abs(appliedImpulse) <= 1e-8) {
    return;
  }

  contact.accumulatedNormalImpulse = nextImpulse;
  applyImpulse(bodyA, bodyB, contact.position, scaleVec3(manifold.normal, appliedImpulse));
  stats.solvedContactCount += 1;
  stats.impulsesApplied += Math.abs(appliedImpulse);
}

function solveTangentImpulse(bodyA, bodyB, contact, tangentDirection, inverseMassSum, maxFrictionImpulse, tangentImpulseKey, stats) {
  if (inverseMassSum <= 1e-8) {
    return;
  }

  const relativeVelocity = getRelativeVelocity(bodyA, bodyB, contact.position);
  const relativeTangentVelocity = dotVec3(relativeVelocity, tangentDirection);
  const impulseDelta = -relativeTangentVelocity / inverseMassSum;
  const previousImpulse = contact[tangentImpulseKey];
  const nextImpulse = Math.max(-maxFrictionImpulse, Math.min(maxFrictionImpulse, previousImpulse + impulseDelta));
  const appliedImpulse = nextImpulse - previousImpulse;

  if (Math.abs(appliedImpulse) <= 1e-8) {
    return;
  }

  contact[tangentImpulseKey] = nextImpulse;
  applyImpulse(bodyA, bodyB, contact.position, scaleVec3(tangentDirection, appliedImpulse));
  stats.solvedTangentContactCount += 1;
  stats.frictionImpulsesApplied += Math.abs(appliedImpulse);
}

export function solveNormalContactConstraints(options = {}) {
  const manifolds = Array.isArray(options.manifolds) ? options.manifolds : [];
  const bodyRegistry = options.bodyRegistry;
  const deltaTime = Math.max(Number(options.deltaTime ?? 0), 1e-8);
  const iterations = Math.max(1, Math.floor(Number(options.iterations ?? 6)));
  const baumgarte = Number(options.baumgarte ?? 0.2);
  const allowedPenetration = Number(options.allowedPenetration ?? 0.01);
  const positionCorrectionPercent = Number(options.positionCorrectionPercent ?? 0.8);
  const stats = createEmptySolverStats(iterations);
  stats.manifoldCount = manifolds.length;

  for (const manifold of manifolds) {
    const bodyA = getDynamicBody(bodyRegistry, manifold.bodyAId);
    const bodyB = getDynamicBody(bodyRegistry, manifold.bodyBId);

    for (const contact of manifold.contacts) {
      const tangentBasis = createTangentBasis(manifold.normal);
      stats.maxPenetration = Math.max(stats.maxPenetration, contact.penetration);

      if (computeEffectiveMass(bodyA, bodyB, contact.position, manifold.normal) <= 1e-8) {
        stats.skippedContactCount += 1;
        continue;
      }

      applyWarmStart(bodyA, bodyB, manifold, contact, tangentBasis, stats);
    }
  }

  for (let iteration = 0; iteration < iterations; iteration += 1) {
    for (const manifold of manifolds) {
      const bodyA = getDynamicBody(bodyRegistry, manifold.bodyAId);
      const bodyB = getDynamicBody(bodyRegistry, manifold.bodyBId);

      for (const contact of manifold.contacts) {
        solveNormalImpulse(
          bodyA,
          bodyB,
          manifold,
          contact,
          deltaTime,
          baumgarte,
          allowedPenetration,
          stats
        );

        const tangentBasis = createTangentBasis(manifold.normal);
        const maxFrictionImpulse = manifold.friction * contact.accumulatedNormalImpulse;
        const inverseMassA = computeEffectiveMass(bodyA, bodyB, contact.position, tangentBasis.tangentA);
        const inverseMassB = computeEffectiveMass(bodyA, bodyB, contact.position, tangentBasis.tangentB);
        solveTangentImpulse(bodyA, bodyB, contact, tangentBasis.tangentA, inverseMassA, maxFrictionImpulse, 'accumulatedTangentImpulseA', stats);
        solveTangentImpulse(bodyA, bodyB, contact, tangentBasis.tangentB, inverseMassB, maxFrictionImpulse, 'accumulatedTangentImpulseB', stats);
      }
    }
  }

  for (const manifold of manifolds) {
    const bodyA = getDynamicBody(bodyRegistry, manifold.bodyAId);
    const bodyB = getDynamicBody(bodyRegistry, manifold.bodyBId);

    for (const contact of manifold.contacts) {
      const linearInverseMassSum = (bodyA?.inverseMass ?? 0) + (bodyB?.inverseMass ?? 0);
      if (linearInverseMassSum <= 1e-8) {
        continue;
      }

      const correctionMagnitude = Math.max(contact.penetration - allowedPenetration, 0) * positionCorrectionPercent / linearInverseMassSum;
      if (correctionMagnitude <= 1e-8) {
        continue;
      }

      const correctionDirection = lengthSquaredVec3(manifold.normal) > 0 ? normalizeVec3(manifold.normal, createVec3(0, 1, 0)) : createVec3(0, 1, 0);
      applyPositionCorrection(bodyA, bodyB, correctionDirection, correctionMagnitude);
      stats.positionCorrections += 1;
    }
  }

  return stats;
}
