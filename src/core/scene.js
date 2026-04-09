import { cloneVec3, createVec3, vec3ToObject } from './math/vec3.js';

function createDefaultCamera() {
  return {
    position: createVec3(0, 0, 400),
    rotation: createVec3(0, 0, 0),
    fov: 60
  };
}

function normalizeObjectId(rawId, objectCount) {
  const stringId = String(rawId || '').trim();
  if (stringId) {
    return stringId;
  }

  return `cube-${objectCount + 1}`;
}

export class Scene3D {
  constructor() {
    this.reset();
  }

  reset() {
    this.camera = createDefaultCamera();
    this.objects = [];
    this.frameCount = 0;
  }

  setCameraPosition(position) {
    this.camera.position = cloneVec3(position);
  }

  addCube({ id, position, size }) {
    const cube = {
      id: normalizeObjectId(id, this.objects.length),
      kind: 'cube',
      position: cloneVec3(position),
      size: Number.isFinite(Number(size)) ? Number(size) : 100
    };

    this.objects = this.objects.filter((object) => object.id !== cube.id);
    this.objects.push(cube);
    return cube;
  }

  nextFrame() {
    this.frameCount += 1;
    return this.frameCount;
  }

  snapshot() {
    return {
      camera: {
        position: vec3ToObject(this.camera.position),
        rotation: vec3ToObject(this.camera.rotation),
        fov: this.camera.fov
      },
      objects: this.objects.map((object) => ({
        id: object.id,
        kind: object.kind,
        position: vec3ToObject(object.position),
        size: object.size
      })),
      objectCount: this.objects.length,
      frameCount: this.frameCount
    };
  }
}

