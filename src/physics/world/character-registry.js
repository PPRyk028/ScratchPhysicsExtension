import { cloneVec3, createVec3 } from '../math/vec3.js';
import { BaseRegistry } from './base-registry.js';

function toFiniteNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function cloneNullableValue(value) {
  if (value === undefined) {
    return null;
  }

  return value;
}

function createDefaultGroundState() {
  return {
    grounded: false,
    walkable: false,
    distance: null,
    angleDegrees: null,
    normal: createVec3(0, 1, 0),
    point: createVec3(),
    colliderId: null,
    bodyId: null
  };
}

export class CharacterRegistry extends BaseRegistry {
  constructor() {
    super('character');
  }

  cloneRecord(character) {
    return {
      id: character.id,
      type: character.type,
      bodyId: character.bodyId,
      colliderId: character.colliderId,
      shapeId: character.shapeId,
      radius: character.radius,
      halfHeight: character.halfHeight,
      skinWidth: character.skinWidth,
      groundProbeDistance: character.groundProbeDistance,
      maxGroundAngleDegrees: character.maxGroundAngleDegrees,
      gravityScale: character.gravityScale,
      jumpSpeed: character.jumpSpeed,
      stepOffset: character.stepOffset,
      groundSnapDistance: character.groundSnapDistance,
      enabled: character.enabled !== false,
      jumpRequested: character.jumpRequested === true,
      verticalVelocity: character.verticalVelocity ?? 0,
      moveIntent: cloneVec3(character.moveIntent ?? createVec3()),
      grounded: character.grounded === true,
      walkable: character.walkable === true,
      groundDistance: character.groundDistance,
      groundAngleDegrees: character.groundAngleDegrees,
      groundNormal: cloneVec3(character.groundNormal ?? createVec3(0, 1, 0)),
      groundPoint: cloneVec3(character.groundPoint ?? createVec3()),
      groundColliderId: character.groundColliderId ?? null,
      groundBodyId: character.groundBodyId ?? null,
      lastRequestedMove: cloneVec3(character.lastRequestedMove ?? createVec3()),
      lastActualMove: cloneVec3(character.lastActualMove ?? createVec3()),
      lastHitNormal: cloneVec3(character.lastHitNormal ?? createVec3()),
      lastHitColliderId: character.lastHitColliderId ?? null,
      lastHitBodyId: character.lastHitBodyId ?? null,
      lastHitDistance: character.lastHitDistance ?? null,
      userData: cloneNullableValue(character.userData)
    };
  }

  createKinematicCapsule(options = {}) {
    const groundState = options.groundState ?? createDefaultGroundState();
    return this.store({
      id: this.allocateId(options.id),
      type: 'kinematic-capsule',
      bodyId: String(options.bodyId ?? '').trim() || null,
      colliderId: String(options.colliderId ?? '').trim() || null,
      shapeId: String(options.shapeId ?? '').trim() || null,
      radius: Math.max(0, toFiniteNumber(options.radius, 0.5)),
      halfHeight: Math.max(0, toFiniteNumber(options.halfHeight, 1)),
      skinWidth: Math.max(0, toFiniteNumber(options.skinWidth, 0.5)),
      groundProbeDistance: Math.max(0, toFiniteNumber(options.groundProbeDistance, 4)),
      maxGroundAngleDegrees: Math.max(0, toFiniteNumber(options.maxGroundAngleDegrees, 55)),
      gravityScale: Math.max(0, toFiniteNumber(options.gravityScale, 1)),
      jumpSpeed: toFiniteNumber(options.jumpSpeed, 8),
      stepOffset: Math.max(0, toFiniteNumber(options.stepOffset, 6)),
      groundSnapDistance: Math.max(0, toFiniteNumber(options.groundSnapDistance, 2)),
      enabled: options.enabled !== false,
      jumpRequested: options.jumpRequested === true,
      verticalVelocity: toFiniteNumber(options.verticalVelocity, 0),
      moveIntent: cloneVec3(options.moveIntent ?? createVec3()),
      grounded: groundState.grounded === true,
      walkable: groundState.walkable === true,
      groundDistance: groundState.distance ?? null,
      groundAngleDegrees: groundState.angleDegrees ?? null,
      groundNormal: cloneVec3(groundState.normal ?? createVec3(0, 1, 0)),
      groundPoint: cloneVec3(groundState.point ?? createVec3()),
      groundColliderId: groundState.colliderId ?? null,
      groundBodyId: groundState.bodyId ?? null,
      lastRequestedMove: cloneVec3(options.lastRequestedMove ?? createVec3()),
      lastActualMove: cloneVec3(options.lastActualMove ?? createVec3()),
      lastHitNormal: cloneVec3(options.lastHitNormal ?? createVec3()),
      lastHitColliderId: String(options.lastHitColliderId ?? '').trim() || null,
      lastHitBodyId: String(options.lastHitBodyId ?? '').trim() || null,
      lastHitDistance: options.lastHitDistance ?? null,
      userData: cloneNullableValue(options.userData)
    });
  }
}
