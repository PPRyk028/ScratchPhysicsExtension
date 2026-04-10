import { addScaledVec3, createVec3, dotVec3, lengthSquaredVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';
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
  if (!body || !body.enabled || body.motionType !== 'dynamic') {
    return null;
  }

  return body;
}

function applyImpulse(bodyA, bodyB, direction, impulseMagnitude) {
  if (bodyA) {
    bodyA.linearVelocity = addScaledVec3(bodyA.linearVelocity, direction, -impulseMagnitude * bodyA.inverseMass);
  }

  if (bodyB) {
    bodyB.linearVelocity = addScaledVec3(bodyB.linearVelocity, direction, impulseMagnitude * bodyB.inverseMass);
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

function getRelativeVelocity(bodyA, bodyB) {
  const velocityA = bodyA?.linearVelocity ?? createVec3();
  const velocityB = bodyB?.linearVelocity ?? createVec3();
  return subtractVec3(velocityB, velocityA);
}

function applyWarmStart(bodyA, bodyB, manifold, contact, tangentBasis, stats) {
  if (contact.accumulatedNormalImpulse > 0) {
    applyImpulse(bodyA, bodyB, manifold.normal, contact.accumulatedNormalImpulse);
    stats.warmStartedContactCount += 1;
  }

  if (Math.abs(contact.accumulatedTangentImpulseA) > 1e-8) {
    applyImpulse(bodyA, bodyB, tangentBasis.tangentA, contact.accumulatedTangentImpulseA);
    stats.frictionImpulsesApplied += Math.abs(contact.accumulatedTangentImpulseA);
  }

  if (Math.abs(contact.accumulatedTangentImpulseB) > 1e-8) {
    applyImpulse(bodyA, bodyB, tangentBasis.tangentB, contact.accumulatedTangentImpulseB);
    stats.frictionImpulsesApplied += Math.abs(contact.accumulatedTangentImpulseB);
  }
}

function solveNormalImpulse(bodyA, bodyB, manifold, contact, inverseMassSum, deltaTime, baumgarte, allowedPenetration, restitutionThreshold, stats) {
  const relativeVelocity = getRelativeVelocity(bodyA, bodyB);
  const relativeNormalVelocity = dotVec3(relativeVelocity, manifold.normal);
  const separationWithSlop = Math.min(0, contact.separation + allowedPenetration);
  const positionBias = (baumgarte * separationWithSlop) / deltaTime;
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
  applyImpulse(bodyA, bodyB, manifold.normal, appliedImpulse);
  stats.solvedContactCount += 1;
  stats.impulsesApplied += Math.abs(appliedImpulse);
}

function solveTangentImpulse(bodyA, bodyB, contact, tangentDirection, inverseMassSum, maxFrictionImpulse, tangentImpulseKey, stats) {
  const relativeVelocity = getRelativeVelocity(bodyA, bodyB);
  const relativeTangentVelocity = dotVec3(relativeVelocity, tangentDirection);
  const impulseDelta = -relativeTangentVelocity / inverseMassSum;
  const previousImpulse = contact[tangentImpulseKey];
  const nextImpulse = Math.max(-maxFrictionImpulse, Math.min(maxFrictionImpulse, previousImpulse + impulseDelta));
  const appliedImpulse = nextImpulse - previousImpulse;

  if (Math.abs(appliedImpulse) <= 1e-8) {
    return;
  }

  contact[tangentImpulseKey] = nextImpulse;
  applyImpulse(bodyA, bodyB, tangentDirection, appliedImpulse);
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
  const restitutionThreshold = Number(options.restitutionThreshold ?? 1);
  const stats = createEmptySolverStats(iterations);
  stats.manifoldCount = manifolds.length;

  for (const manifold of manifolds) {
    const bodyA = getDynamicBody(bodyRegistry, manifold.bodyAId);
    const bodyB = getDynamicBody(bodyRegistry, manifold.bodyBId);
    const manifoldContactCount = Math.max(1, manifold.contacts.length);
    const inverseMassSum = ((bodyA?.inverseMass ?? 0) + (bodyB?.inverseMass ?? 0)) * manifoldContactCount;

    for (const contact of manifold.contacts) {
      if (inverseMassSum <= 0) {
        stats.skippedContactCount += 1;
        continue;
      }

      stats.maxPenetration = Math.max(stats.maxPenetration, contact.penetration);
      const tangentBasis = createTangentBasis(manifold.normal);
      applyWarmStart(bodyA, bodyB, manifold, contact, tangentBasis, stats);
    }
  }

  for (let iteration = 0; iteration < iterations; iteration += 1) {
    for (const manifold of manifolds) {
      const bodyA = getDynamicBody(bodyRegistry, manifold.bodyAId);
      const bodyB = getDynamicBody(bodyRegistry, manifold.bodyBId);
      const manifoldContactCount = Math.max(1, manifold.contacts.length);
      const inverseMassSum = ((bodyA?.inverseMass ?? 0) + (bodyB?.inverseMass ?? 0)) * manifoldContactCount;

      for (const contact of manifold.contacts) {
        if (inverseMassSum <= 0) {
          continue;
        }

        solveNormalImpulse(
          bodyA,
          bodyB,
          manifold,
          contact,
          inverseMassSum,
          deltaTime,
          baumgarte,
          allowedPenetration,
          restitutionThreshold,
          stats
        );

        const tangentBasis = createTangentBasis(manifold.normal);
        const maxFrictionImpulse = manifold.friction * contact.accumulatedNormalImpulse;
        solveTangentImpulse(bodyA, bodyB, contact, tangentBasis.tangentA, inverseMassSum, maxFrictionImpulse, 'accumulatedTangentImpulseA', stats);
        solveTangentImpulse(bodyA, bodyB, contact, tangentBasis.tangentB, inverseMassSum, maxFrictionImpulse, 'accumulatedTangentImpulseB', stats);
      }
    }
  }

  for (const manifold of manifolds) {
    const bodyA = getDynamicBody(bodyRegistry, manifold.bodyAId);
    const bodyB = getDynamicBody(bodyRegistry, manifold.bodyBId);
    const manifoldContactCount = Math.max(1, manifold.contacts.length);
    const inverseMassSum = ((bodyA?.inverseMass ?? 0) + (bodyB?.inverseMass ?? 0)) * manifoldContactCount;

    for (const contact of manifold.contacts) {
      if (inverseMassSum <= 0) {
        continue;
      }

      const correctionMagnitude = Math.max(contact.penetration - allowedPenetration, 0) * positionCorrectionPercent / inverseMassSum;
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
