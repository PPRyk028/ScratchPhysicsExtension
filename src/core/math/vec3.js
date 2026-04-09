export function createVec3(x = 0, y = 0, z = 0) {
  return {
    x: Number.isFinite(Number(x)) ? Number(x) : 0,
    y: Number.isFinite(Number(y)) ? Number(y) : 0,
    z: Number.isFinite(Number(z)) ? Number(z) : 0
  };
}

export function cloneVec3(vector) {
  return createVec3(vector.x, vector.y, vector.z);
}

export function scaleVec3(vector, scalar) {
  return createVec3(vector.x * scalar, vector.y * scalar, vector.z * scalar);
}

export function vec3ToObject(vector) {
  return {
    x: vector.x,
    y: vector.y,
    z: vector.z
  };
}

