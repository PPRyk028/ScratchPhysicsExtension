import { addScaledVec3, addVec3, cloneVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, lengthVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';

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

export function createVolumeConstraint(options = {}) {
  return {
    id: String(options.id ?? '').trim() || 'volume-constraint',
    type: 'volume',
    kind: String(options.kind ?? 'volume').trim() || 'volume',
    particleAId: String(options.particleAId ?? '').trim(),
    particleBId: String(options.particleBId ?? '').trim(),
    particleCId: String(options.particleCId ?? '').trim(),
    particleDId: String(options.particleDId ?? '').trim(),
    restVolume: Number(options.restVolume ?? 0),
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

function computeSignedTetraVolume(pointA, pointB, pointC, pointD) {
  const edgeAB = subtractVec3(pointB, pointA);
  const edgeAC = subtractVec3(pointC, pointA);
  const edgeAD = subtractVec3(pointD, pointA);
  return dotVec3(crossVec3(edgeAB, edgeAC), edgeAD) / 6;
}

export function solveVolumeConstraint(constraint, particleA, particleB, particleC, particleD, deltaTime) {
  if (!constraint || !particleA || !particleB || !particleC || !particleD) {
    return false;
  }

  const predictedA = particleA.predictedPosition;
  const predictedB = particleB.predictedPosition;
  const predictedC = particleC.predictedPosition;
  const predictedD = particleD.predictedPosition;

  const gradientB = scaleVec3(crossVec3(subtractVec3(predictedC, predictedA), subtractVec3(predictedD, predictedA)), 1 / 6);
  const gradientC = scaleVec3(crossVec3(subtractVec3(predictedD, predictedA), subtractVec3(predictedB, predictedA)), 1 / 6);
  const gradientD = scaleVec3(crossVec3(subtractVec3(predictedB, predictedA), subtractVec3(predictedC, predictedA)), 1 / 6);
  const gradientA = scaleVec3(addVec3(addVec3(gradientB, gradientC), gradientD), -1);

  const inverseMassA = Number(particleA.inverseMass ?? 0);
  const inverseMassB = Number(particleB.inverseMass ?? 0);
  const inverseMassC = Number(particleC.inverseMass ?? 0);
  const inverseMassD = Number(particleD.inverseMass ?? 0);
  const denominator =
    inverseMassA * lengthSquaredVec3(gradientA) +
    inverseMassB * lengthSquaredVec3(gradientB) +
    inverseMassC * lengthSquaredVec3(gradientC) +
    inverseMassD * lengthSquaredVec3(gradientD) +
    getConstraintAlpha(constraint, deltaTime);
  if (denominator <= 1e-8) {
    return false;
  }

  const volume = computeSignedTetraVolume(predictedA, predictedB, predictedC, predictedD);
  const constraintError = volume - Number(constraint.restVolume ?? 0);
  const alpha = getConstraintAlpha(constraint, deltaTime);
  const deltaLambda = (-constraintError - alpha * Number(constraint.lambda ?? 0)) / denominator;
  constraint.lambda = Number(constraint.lambda ?? 0) + deltaLambda;

  if (inverseMassA > 0) {
    particleA.predictedPosition = addScaledVec3(particleA.predictedPosition, gradientA, inverseMassA * deltaLambda);
  }
  if (inverseMassB > 0) {
    particleB.predictedPosition = addScaledVec3(particleB.predictedPosition, gradientB, inverseMassB * deltaLambda);
  }
  if (inverseMassC > 0) {
    particleC.predictedPosition = addScaledVec3(particleC.predictedPosition, gradientC, inverseMassC * deltaLambda);
  }
  if (inverseMassD > 0) {
    particleD.predictedPosition = addScaledVec3(particleD.predictedPosition, gradientD, inverseMassD * deltaLambda);
  }

  return true;
}
