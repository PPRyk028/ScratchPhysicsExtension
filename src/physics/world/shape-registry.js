import { cloneQuat, createIdentityQuat } from '../math/quat.js';
import { cloneVec3, createVec3 } from '../math/vec3.js';
import { BaseRegistry } from './base-registry.js';

function toPositiveNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
}

function cloneVertices(vertices) {
  return Array.isArray(vertices) ? vertices.map((vertex) => cloneVec3(vertex)) : [];
}

function createLocalPose(localPose) {
  return {
    position: cloneVec3(localPose?.position ?? createVec3()),
    rotation: cloneQuat(localPose?.rotation ?? createIdentityQuat())
  };
}

function cloneGeometry(shape) {
  if (shape.type === 'box') {
    return {
      halfExtents: cloneVec3(shape.geometry.halfExtents)
    };
  }

  if (shape.type === 'sphere') {
    return {
      radius: shape.geometry.radius
    };
  }

  if (shape.type === 'capsule') {
    return {
      radius: shape.geometry.radius,
      halfHeight: shape.geometry.halfHeight
    };
  }

  if (shape.type === 'convex-hull') {
    return {
      vertices: cloneVertices(shape.geometry.vertices)
    };
  }

  return { ...shape.geometry };
}

export class ShapeRegistry extends BaseRegistry {
  constructor() {
    super('shape');
  }

  cloneRecord(shape) {
    return {
      id: shape.id,
      type: shape.type,
      geometry: cloneGeometry(shape),
      localPose: createLocalPose(shape.localPose),
      userData: shape.userData ?? null
    };
  }

  createBoxShape(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      type: 'box',
      geometry: {
        halfExtents: cloneVec3(options.halfExtents ?? createVec3(0.5, 0.5, 0.5))
      },
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }

  createSphereShape(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      type: 'sphere',
      geometry: {
        radius: toPositiveNumber(options.radius, 0.5)
      },
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }

  createCapsuleShape(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      type: 'capsule',
      geometry: {
        radius: toPositiveNumber(options.radius, 0.5),
        halfHeight: toPositiveNumber(options.halfHeight, 0.5)
      },
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }

  createConvexHullShape(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      type: 'convex-hull',
      geometry: {
        vertices: cloneVertices(options.vertices)
      },
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }
}

