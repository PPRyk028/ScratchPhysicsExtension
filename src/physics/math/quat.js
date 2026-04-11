import { addScaledVec3, createVec3, crossVec3, normalizeVec3 } from './vec3.js';

function toFiniteNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

export function createQuat(x = 0, y = 0, z = 0, w = 1) {
  return {
    x: toFiniteNumber(x, 0),
    y: toFiniteNumber(y, 0),
    z: toFiniteNumber(z, 0),
    w: toFiniteNumber(w, 1)
  };
}

export function createIdentityQuat() {
  return createQuat(0, 0, 0, 1);
}

export function createQuatFromAxisAngle(axis, angleRadians = 0) {
  const unitAxis = normalizeVec3(axis, createVec3(0, 1, 0));
  const halfAngle = toFiniteNumber(angleRadians, 0) * 0.5;
  const sine = Math.sin(halfAngle);
  return normalizeQuat(createQuat(
    unitAxis.x * sine,
    unitAxis.y * sine,
    unitAxis.z * sine,
    Math.cos(halfAngle)
  ));
}

export function cloneQuat(quaternion) {
  return createQuat(quaternion?.x, quaternion?.y, quaternion?.z, quaternion?.w);
}

export function lengthSquaredQuat(quaternion) {
  return (
    toFiniteNumber(quaternion?.x, 0) * toFiniteNumber(quaternion?.x, 0) +
    toFiniteNumber(quaternion?.y, 0) * toFiniteNumber(quaternion?.y, 0) +
    toFiniteNumber(quaternion?.z, 0) * toFiniteNumber(quaternion?.z, 0) +
    toFiniteNumber(quaternion?.w, 1) * toFiniteNumber(quaternion?.w, 1)
  );
}

export function normalizeQuat(quaternion, fallback = createIdentityQuat()) {
  const lengthSquared = lengthSquaredQuat(quaternion);
  if (lengthSquared <= 1e-12) {
    return cloneQuat(fallback);
  }

  const inverseLength = 1 / Math.sqrt(lengthSquared);
  return createQuat(
    quaternion.x * inverseLength,
    quaternion.y * inverseLength,
    quaternion.z * inverseLength,
    quaternion.w * inverseLength
  );
}

export function conjugateQuat(quaternion) {
  return createQuat(
    -toFiniteNumber(quaternion?.x, 0),
    -toFiniteNumber(quaternion?.y, 0),
    -toFiniteNumber(quaternion?.z, 0),
    toFiniteNumber(quaternion?.w, 1)
  );
}

export function multiplyQuat(left, right) {
  const a = cloneQuat(left);
  const b = cloneQuat(right);

  return createQuat(
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
  );
}

export function rotateVec3ByQuat(quaternion, vector) {
  const unitQuat = normalizeQuat(quaternion);
  const qVector = createVec3(unitQuat.x, unitQuat.y, unitQuat.z);
  const doubledCross = addScaledVec3(createVec3(), crossVec3(qVector, vector), 2);
  return addScaledVec3(
    addScaledVec3(vector, doubledCross, unitQuat.w),
    crossVec3(qVector, doubledCross),
    1
  );
}

export function inverseRotateVec3ByQuat(quaternion, vector) {
  return rotateVec3ByQuat(conjugateQuat(quaternion), vector);
}

export function integrateQuat(quaternion, angularVelocity, deltaTime) {
  const unitQuat = normalizeQuat(quaternion);
  const omegaQuat = createQuat(angularVelocity?.x, angularVelocity?.y, angularVelocity?.z, 0);
  const derivative = multiplyQuat(omegaQuat, unitQuat);
  return normalizeQuat(createQuat(
    unitQuat.x + derivative.x * 0.5 * deltaTime,
    unitQuat.y + derivative.y * 0.5 * deltaTime,
    unitQuat.z + derivative.z * 0.5 * deltaTime,
    unitQuat.w + derivative.w * 0.5 * deltaTime
  ));
}
