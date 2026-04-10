function toFiniteNumber(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

export function createVec3(x = 0, y = 0, z = 0) {
  return {
    x: toFiniteNumber(x),
    y: toFiniteNumber(y),
    z: toFiniteNumber(z)
  };
}

export function cloneVec3(vector) {
  return createVec3(vector?.x, vector?.y, vector?.z);
}

export function zeroVec3() {
  return createVec3(0, 0, 0);
}

export function scaleVec3(vector, scalar) {
  return createVec3(
    toFiniteNumber(vector?.x) * toFiniteNumber(scalar, 1),
    toFiniteNumber(vector?.y) * toFiniteNumber(scalar, 1),
    toFiniteNumber(vector?.z) * toFiniteNumber(scalar, 1)
  );
}

export function addVec3(left, right) {
  return createVec3(
    toFiniteNumber(left?.x) + toFiniteNumber(right?.x),
    toFiniteNumber(left?.y) + toFiniteNumber(right?.y),
    toFiniteNumber(left?.z) + toFiniteNumber(right?.z)
  );
}

export function addScaledVec3(base, delta, scale) {
  return addVec3(base, scaleVec3(delta, scale));
}

export function subtractVec3(left, right) {
  return createVec3(
    toFiniteNumber(left?.x) - toFiniteNumber(right?.x),
    toFiniteNumber(left?.y) - toFiniteNumber(right?.y),
    toFiniteNumber(left?.z) - toFiniteNumber(right?.z)
  );
}

export function minVec3(left, right) {
  return createVec3(
    Math.min(toFiniteNumber(left?.x), toFiniteNumber(right?.x)),
    Math.min(toFiniteNumber(left?.y), toFiniteNumber(right?.y)),
    Math.min(toFiniteNumber(left?.z), toFiniteNumber(right?.z))
  );
}

export function maxVec3(left, right) {
  return createVec3(
    Math.max(toFiniteNumber(left?.x), toFiniteNumber(right?.x)),
    Math.max(toFiniteNumber(left?.y), toFiniteNumber(right?.y)),
    Math.max(toFiniteNumber(left?.z), toFiniteNumber(right?.z))
  );
}
