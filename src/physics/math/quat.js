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

export function cloneQuat(quaternion) {
  return createQuat(quaternion?.x, quaternion?.y, quaternion?.z, quaternion?.w);
}

