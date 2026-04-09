import { createVec3 } from './vec3.js';

export function createIdentityMat4() {
  return [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  ];
}

export function createTranslationMat4(position) {
  const value = createIdentityMat4();
  value[12] = position.x;
  value[13] = position.y;
  value[14] = position.z;
  return value;
}

export function createUniformScaleMat4(scale) {
  const size = Number.isFinite(Number(scale)) ? Number(scale) : 1;
  return [
    size, 0, 0, 0,
    0, size, 0, 0,
    0, 0, size, 0,
    0, 0, 0, 1
  ];
}

export function multiplyMat4(left, right) {
  const result = new Array(16).fill(0);
  for (let row = 0; row < 4; row += 1) {
    for (let column = 0; column < 4; column += 1) {
      for (let index = 0; index < 4; index += 1) {
        result[row * 4 + column] += left[row * 4 + index] * right[index * 4 + column];
      }
    }
  }
  return result;
}

export function buildCubeModelMatrix(position, size) {
  const resolvedPosition = createVec3(position.x, position.y, position.z);
  return multiplyMat4(
    createTranslationMat4(resolvedPosition),
    createUniformScaleMat4(size)
  );
}

