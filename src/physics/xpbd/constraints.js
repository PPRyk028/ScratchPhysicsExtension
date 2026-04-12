import { addScaledVec3, cloneVec3, createVec3, lengthVec3, normalizeVec3, subtractVec3 } from '../math/vec3.js';

function toNonNegativeNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function getConstraintAlpha(constraint, deltaTime) {
  const compliance = toNonNegativeNumber(constraint.compliance, 0);
  if (compliance === 0 || deltaTime <= 0) {
    return 0;
  }

  return compliance / (deltaTime * deltaTime);
}

export function cloneXpbdConstraint(constraint) {
  if (!constraint) {
    return null;
  }

  return {
    ...constraint,
    targetPosition: constraint.targetPosition ? cloneVec3(constraint.targetPosition) : null
  };
}

export function createDistanceConstraint(options = {}) {
  return {
    id: String(options.id ?? '').trim() || 'distance-constraint',
    type: 'distance',
    kind: String(options.kind ?? 'stretch').trim() || 'stretch',
    particleAId: String(options.particleAId ?? '').trim(),
    particleBId: String(options.particleBId ?? '').trim(),
    restLength: Math.max(0, Number(options.restLength ?? 0)),
    compliance: toNonNegativeNumber(options.compliance, 0),
    lambda: 0
  };
}

export function createPinConstraint(options = {}) {
  return {
    id: String(options.id ?? '').trim() || 'pin-constraint',
    type: 'pin',
    kind: String(options.kind ?? 'pin').trim() || 'pin',
    particleId: String(options.particleId ?? '').trim(),
    targetPosition: cloneVec3(options.targetPosition ?? createVec3()),
    compliance: toNonNegativeNumber(options.compliance, 0),
    lambda: 0
  };
}

export function resetConstraintLambdas(constraints) {
  for (const constraint of constraints) {
    constraint.lambda = 0;
  }
}

export function solveDistanceConstraint(constraint, particleA, particleB, deltaTime) {
  if (!constraint || !particleA || !particleB) {
    return false;
  }

  const difference = subtractVec3(particleB.predictedPosition, particleA.predictedPosition);
  const distance = lengthVec3(difference);
  if (distance <= 1e-8) {
    return false;
  }

  const gradient = normalizeVec3(difference, createVec3(1, 0, 0));
  const constraintError = distance - Math.max(0, Number(constraint.restLength ?? 0));
  const inverseMassSum = Number(particleA.inverseMass ?? 0) + Number(particleB.inverseMass ?? 0);
  const alpha = getConstraintAlpha(constraint, deltaTime);
  const denominator = inverseMassSum + alpha;
  if (denominator <= 1e-8) {
    return false;
  }

  const deltaLambda = (-constraintError - alpha * Number(constraint.lambda ?? 0)) / denominator;
  constraint.lambda = Number(constraint.lambda ?? 0) + deltaLambda;

  if (particleA.inverseMass > 0) {
    particleA.predictedPosition = addScaledVec3(
      particleA.predictedPosition,
      gradient,
      -deltaLambda * particleA.inverseMass
    );
  }

  if (particleB.inverseMass > 0) {
    particleB.predictedPosition = addScaledVec3(
      particleB.predictedPosition,
      gradient,
      deltaLambda * particleB.inverseMass
    );
  }

  return true;
}

export function solvePinConstraint(constraint, particle, deltaTime) {
  if (!constraint || !particle) {
    return false;
  }

  const difference = subtractVec3(particle.predictedPosition, constraint.targetPosition);
  const distance = lengthVec3(difference);
  if (distance <= 1e-8) {
    particle.predictedPosition = cloneVec3(constraint.targetPosition);
    return false;
  }

  const gradient = normalizeVec3(difference, createVec3(0, 1, 0));
  const inverseMass = Number(particle.inverseMass ?? 0);
  const alpha = getConstraintAlpha(constraint, deltaTime);
  const denominator = inverseMass + alpha;
  if (denominator <= 1e-8) {
    particle.predictedPosition = cloneVec3(constraint.targetPosition);
    return false;
  }

  const deltaLambda = (-distance - alpha * Number(constraint.lambda ?? 0)) / denominator;
  constraint.lambda = Number(constraint.lambda ?? 0) + deltaLambda;
  particle.predictedPosition = addScaledVec3(
    particle.predictedPosition,
    gradient,
    inverseMass * deltaLambda
  );

  return true;
}
