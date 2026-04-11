import { cloneQuat, createIdentityQuat } from '../math/quat.js';
import { cloneVec3, createVec3 } from '../math/vec3.js';
import { BaseRegistry } from './base-registry.js';

function toFiniteNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function normalizeMotionType(value) {
  const motionType = String(value ?? '').trim().toLowerCase();
  if (motionType === 'static' || motionType === 'kinematic') {
    return motionType;
  }

  return 'dynamic';
}

function resolveMass(motionType, mass) {
  if (motionType !== 'dynamic') {
    return 0;
  }

  const parsedMass = toFiniteNumber(mass, 1);
  return parsedMass > 0 ? parsedMass : 1;
}

function cloneNullableValue(value) {
  if (value === undefined) {
    return null;
  }

  return value;
}

export class BodyRegistry extends BaseRegistry {
  constructor() {
    super('body');
  }

  cloneRecord(body) {
    return {
      id: body.id,
      type: body.type,
      motionType: body.motionType,
      shapeId: body.shapeId,
      primaryColliderId: body.primaryColliderId,
      colliderIds: [...body.colliderIds],
      position: cloneVec3(body.position),
      rotation: cloneQuat(body.rotation),
      linearVelocity: cloneVec3(body.linearVelocity),
      angularVelocity: cloneVec3(body.angularVelocity),
      forceAccumulator: cloneVec3(body.forceAccumulator),
      torqueAccumulator: cloneVec3(body.torqueAccumulator),
      mass: body.mass,
      inverseMass: body.inverseMass,
      inertia: cloneVec3(body.inertia),
      inverseInertia: cloneVec3(body.inverseInertia),
      canSleep: body.canSleep,
      sleeping: body.sleeping,
      sleepTimer: body.sleepTimer,
      enabled: body.enabled,
      userData: cloneNullableValue(body.userData)
    };
  }

  createRigidBody(options = {}) {
    const motionType = normalizeMotionType(options.motionType);
    const mass = resolveMass(motionType, options.mass);

    return this.store({
      id: this.allocateId(options.id),
      type: 'rigid-body',
      motionType,
      shapeId: String(options.shapeId ?? '').trim() || null,
      primaryColliderId: String(options.primaryColliderId ?? '').trim() || null,
      colliderIds: Array.isArray(options.colliderIds) ? [...options.colliderIds] : [],
      position: cloneVec3(options.position ?? createVec3()),
      rotation: cloneQuat(options.rotation ?? createIdentityQuat()),
      linearVelocity: cloneVec3(options.linearVelocity ?? createVec3()),
      angularVelocity: cloneVec3(options.angularVelocity ?? createVec3()),
      forceAccumulator: cloneVec3(options.forceAccumulator ?? createVec3()),
      torqueAccumulator: cloneVec3(options.torqueAccumulator ?? createVec3()),
      mass,
      inverseMass: mass > 0 ? 1 / mass : 0,
      inertia: cloneVec3(options.inertia ?? createVec3()),
      inverseInertia: cloneVec3(options.inverseInertia ?? createVec3()),
      canSleep: options.canSleep !== false,
      sleeping: Boolean(options.sleeping),
      sleepTimer: Math.max(0, toFiniteNumber(options.sleepTimer, 0)),
      enabled: options.enabled !== false,
      userData: cloneNullableValue(options.userData)
    });
  }

  attachCollider(bodyId, colliderId, shapeId = null) {
    const body = this.getMutable(bodyId);
    if (!body) {
      return null;
    }

    if (!body.colliderIds.includes(colliderId)) {
      body.colliderIds.push(colliderId);
    }

    if (!body.primaryColliderId) {
      body.primaryColliderId = colliderId;
    }

    if (shapeId && !body.shapeId) {
      body.shapeId = shapeId;
    }

    return this.cloneRecord(body);
  }
}
