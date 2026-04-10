import { cloneQuat, createIdentityQuat } from '../math/quat.js';
import { cloneVec3, createVec3 } from '../math/vec3.js';
import { BaseRegistry } from './base-registry.js';

function createLocalPose(localPose) {
  return {
    position: cloneVec3(localPose?.position ?? createVec3()),
    rotation: cloneQuat(localPose?.rotation ?? createIdentityQuat())
  };
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
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }
}
