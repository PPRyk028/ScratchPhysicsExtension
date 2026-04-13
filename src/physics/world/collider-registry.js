import { cloneQuat, createIdentityQuat } from '../math/quat.js';
import { cloneVec3, createVec3 } from '../math/vec3.js';
import { BaseRegistry } from './base-registry.js';

function createLocalPose(localPose) {
  return {
    position: cloneVec3(localPose?.position ?? createVec3()),
    rotation: cloneQuat(localPose?.rotation ?? createIdentityQuat())
  };
}

function normalizeCollisionBits(value, fallback) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) {
    return fallback;
  }

  return Math.max(0, Math.min(0x7fffffff, Math.trunc(parsed)));
}

export class ColliderRegistry extends BaseRegistry {
  constructor() {
    super('collider');
  }

  cloneRecord(collider) {
    return {
      id: collider.id,
      type: collider.type,
      shapeId: collider.shapeId,
      bodyId: collider.bodyId,
      materialId: collider.materialId,
      enabled: collider.enabled,
      isSensor: collider.isSensor,
      isOneWay: collider.isOneWay,
      collisionLayer: collider.collisionLayer,
      collisionMask: collider.collisionMask,
      localPose: createLocalPose(collider.localPose),
      userData: collider.userData ?? null
    };
  }

  createCollider(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      type: 'collider',
      shapeId: String(options.shapeId ?? '').trim() || null,
      bodyId: String(options.bodyId ?? '').trim() || null,
      materialId: String(options.materialId ?? '').trim() || null,
      enabled: options.enabled !== false,
      isSensor: Boolean(options.isSensor),
      isOneWay: Boolean(options.isOneWay) && !Boolean(options.isSensor),
      collisionLayer: normalizeCollisionBits(options.collisionLayer, 1),
      collisionMask: normalizeCollisionBits(options.collisionMask, 0x7fffffff),
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }
}
