import { cloneVec3, createVec3, lengthVec3, normalizeVec3, subtractVec3 } from '../math/vec3.js';
import { BaseRegistry } from './base-registry.js';

function toNonNegativeNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function toOptionalNumber(value, fallback = null) {
  if (value === undefined || value === null || value === '') {
    return fallback;
  }

  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function toOptionalNonNegativeNumber(value, fallback = null) {
  const parsed = toOptionalNumber(value, fallback);
  return parsed !== null && parsed >= 0 ? parsed : fallback;
}

function clampDistanceToRange(distance, minDistance, maxDistance) {
  let resolvedDistance = toNonNegativeNumber(distance, 0);

  if (minDistance !== null) {
    resolvedDistance = Math.max(resolvedDistance, minDistance);
  }

  if (maxDistance !== null) {
    resolvedDistance = Math.min(resolvedDistance, maxDistance);
  }

  return resolvedDistance;
}

function normalizeDistanceLimits(minDistance, maxDistance) {
  let resolvedMin = toOptionalNumber(minDistance, null);
  let resolvedMax = toOptionalNumber(maxDistance, null);

  resolvedMin = resolvedMin !== null && resolvedMin >= 0 ? resolvedMin : null;
  resolvedMax = resolvedMax !== null && resolvedMax >= 0 ? resolvedMax : null;

  if (resolvedMin !== null && resolvedMax !== null && resolvedMin > resolvedMax) {
    const temp = resolvedMin;
    resolvedMin = resolvedMax;
    resolvedMax = temp;
  }

  return {
    minDistance: resolvedMin,
    maxDistance: resolvedMax
  };
}

function normalizeAngleLimits(lowerAngle, upperAngle) {
  let resolvedLower = toOptionalNumber(lowerAngle, null);
  let resolvedUpper = toOptionalNumber(upperAngle, null);

  if (resolvedLower !== null && resolvedUpper !== null && resolvedLower > resolvedUpper) {
    const temp = resolvedLower;
    resolvedLower = resolvedUpper;
    resolvedUpper = temp;
  }

  return {
    lowerAngle: resolvedLower,
    upperAngle: resolvedUpper
  };
}

function cloneNullableValue(value) {
  if (value === undefined) {
    return null;
  }

  return value;
}

function computeDistance(localAnchorA, localAnchorB) {
  return lengthVec3(subtractVec3(localAnchorB ?? createVec3(), localAnchorA ?? createVec3()));
}

export class JointRegistry extends BaseRegistry {
  constructor() {
    super('joint');
  }

  cloneRecord(joint) {
    return {
      id: joint.id,
      type: joint.type,
      bodyAId: joint.bodyAId,
      bodyBId: joint.bodyBId,
      collideConnected: joint.collideConnected === true,
      localAnchorA: cloneVec3(joint.localAnchorA),
      localAnchorB: cloneVec3(joint.localAnchorB),
      localAxisA: cloneVec3(joint.localAxisA),
      localAxisB: cloneVec3(joint.localAxisB),
      localReferenceA: cloneVec3(joint.localReferenceA),
      localReferenceB: cloneVec3(joint.localReferenceB),
      distance: joint.distance,
      minDistance: joint.minDistance,
      maxDistance: joint.maxDistance,
      springFrequency: joint.springFrequency,
      dampingRatio: joint.dampingRatio,
      lowerAngle: joint.lowerAngle,
      upperAngle: joint.upperAngle,
      angularDamping: joint.angularDamping,
      breakForce: joint.breakForce,
      breakTorque: joint.breakTorque,
      broken: joint.broken,
      lastAppliedForce: joint.lastAppliedForce,
      lastAppliedTorque: joint.lastAppliedTorque,
      motorEnabled: joint.motorEnabled,
      motorMode: joint.motorMode,
      motorSpeed: joint.motorSpeed,
      motorTargetAngle: joint.motorTargetAngle,
      motorServoGain: joint.motorServoGain,
      maxMotorTorque: joint.maxMotorTorque,
      accumulatedImpulse: joint.accumulatedImpulse,
      accumulatedLinearImpulse: cloneVec3(joint.accumulatedLinearImpulse),
      accumulatedAngularImpulse: cloneVec3(joint.accumulatedAngularImpulse),
      accumulatedMotorImpulse: joint.accumulatedMotorImpulse,
      enabled: joint.enabled,
      userData: cloneNullableValue(joint.userData)
    };
  }

  createDistanceJoint(options = {}) {
    const localAnchorA = cloneVec3(options.localAnchorA ?? createVec3());
    const localAnchorB = cloneVec3(options.localAnchorB ?? createVec3());
    const fallbackDistance = computeDistance(localAnchorA, localAnchorB);
    const { minDistance, maxDistance } = normalizeDistanceLimits(options.minDistance, options.maxDistance);
    const distance = clampDistanceToRange(
      toNonNegativeNumber(options.distance, fallbackDistance),
      minDistance,
      maxDistance
    );

    return this.store({
      id: this.allocateId(options.id),
      type: 'distance-joint',
      bodyAId: String(options.bodyAId ?? '').trim() || null,
      bodyBId: String(options.bodyBId ?? '').trim() || null,
      collideConnected: options.collideConnected === true,
      localAnchorA,
      localAnchorB,
      localAxisA: createVec3(0, 1, 0),
      localAxisB: createVec3(0, 1, 0),
      localReferenceA: createVec3(1, 0, 0),
      localReferenceB: createVec3(1, 0, 0),
      distance,
      minDistance,
      maxDistance,
      springFrequency: toNonNegativeNumber(options.springFrequency, 0),
      dampingRatio: toNonNegativeNumber(options.dampingRatio, 0),
      lowerAngle: null,
      upperAngle: null,
      angularDamping: 0,
      breakForce: null,
      breakTorque: null,
      broken: false,
      lastAppliedForce: 0,
      lastAppliedTorque: 0,
      motorEnabled: false,
      motorMode: 'speed',
      motorSpeed: 0,
      motorTargetAngle: 0,
      motorServoGain: 8,
      maxMotorTorque: 0,
      accumulatedImpulse: Number.isFinite(Number(options.accumulatedImpulse))
        ? Number(options.accumulatedImpulse)
        : 0,
      accumulatedLinearImpulse: cloneVec3(options.accumulatedLinearImpulse ?? createVec3()),
      accumulatedAngularImpulse: cloneVec3(options.accumulatedAngularImpulse ?? createVec3()),
      accumulatedMotorImpulse: 0,
      enabled: options.enabled !== false,
      userData: cloneNullableValue(options.userData)
    });
  }

  createPointToPointJoint(options = {}) {
    const localAnchorA = cloneVec3(options.localAnchorA ?? createVec3());
    const localAnchorB = cloneVec3(options.localAnchorB ?? createVec3());

    return this.store({
      id: this.allocateId(options.id),
      type: 'point-to-point-joint',
      bodyAId: String(options.bodyAId ?? '').trim() || null,
      bodyBId: String(options.bodyBId ?? '').trim() || null,
      collideConnected: options.collideConnected === true,
      localAnchorA,
      localAnchorB,
      localAxisA: createVec3(0, 1, 0),
      localAxisB: createVec3(0, 1, 0),
      localReferenceA: createVec3(1, 0, 0),
      localReferenceB: createVec3(1, 0, 0),
      distance: 0,
      minDistance: null,
      maxDistance: null,
      springFrequency: 0,
      dampingRatio: 0,
      lowerAngle: null,
      upperAngle: null,
      angularDamping: 0,
      breakForce: null,
      breakTorque: null,
      broken: false,
      lastAppliedForce: 0,
      lastAppliedTorque: 0,
      motorEnabled: false,
      motorMode: 'speed',
      motorSpeed: 0,
      motorTargetAngle: 0,
      motorServoGain: 8,
      maxMotorTorque: 0,
      accumulatedImpulse: 0,
      accumulatedLinearImpulse: cloneVec3(options.accumulatedLinearImpulse ?? createVec3()),
      accumulatedAngularImpulse: cloneVec3(options.accumulatedAngularImpulse ?? createVec3()),
      accumulatedMotorImpulse: 0,
      enabled: options.enabled !== false,
      userData: cloneNullableValue(options.userData)
    });
  }

  createHingeJoint(options = {}) {
    const localAnchorA = cloneVec3(options.localAnchorA ?? createVec3());
    const localAnchorB = cloneVec3(options.localAnchorB ?? createVec3());
    const localAxisA = normalizeVec3(options.localAxisA ?? createVec3(0, 1, 0), createVec3(0, 1, 0));
    const localAxisB = normalizeVec3(options.localAxisB ?? createVec3(0, 1, 0), createVec3(0, 1, 0));
    const { lowerAngle, upperAngle } = normalizeAngleLimits(options.lowerAngle, options.upperAngle);

    return this.store({
      id: this.allocateId(options.id),
      type: 'hinge-joint',
      bodyAId: String(options.bodyAId ?? '').trim() || null,
      bodyBId: String(options.bodyBId ?? '').trim() || null,
      collideConnected: options.collideConnected === true,
      localAnchorA,
      localAnchorB,
      localAxisA,
      localAxisB,
      localReferenceA: normalizeVec3(options.localReferenceA ?? createVec3(1, 0, 0), createVec3(1, 0, 0)),
      localReferenceB: normalizeVec3(options.localReferenceB ?? createVec3(1, 0, 0), createVec3(1, 0, 0)),
      distance: 0,
      minDistance: null,
      maxDistance: null,
      springFrequency: 0,
      dampingRatio: 0,
      lowerAngle,
      upperAngle,
      angularDamping: toNonNegativeNumber(options.angularDamping, 0),
      breakForce: null,
      breakTorque: null,
      broken: false,
      lastAppliedForce: 0,
      lastAppliedTorque: 0,
      motorEnabled: options.motorEnabled === true && toNonNegativeNumber(options.maxMotorTorque, 0) > 0,
      motorMode: options.motorMode === 'servo' ? 'servo' : 'speed',
      motorSpeed: Number.isFinite(Number(options.motorSpeed)) ? Number(options.motorSpeed) : 0,
      motorTargetAngle: Number.isFinite(Number(options.motorTargetAngle)) ? Number(options.motorTargetAngle) : 0,
      motorServoGain: toNonNegativeNumber(options.motorServoGain, 8),
      maxMotorTorque: toNonNegativeNumber(options.maxMotorTorque, 0),
      accumulatedImpulse: 0,
      accumulatedLinearImpulse: cloneVec3(options.accumulatedLinearImpulse ?? createVec3()),
      accumulatedAngularImpulse: cloneVec3(options.accumulatedAngularImpulse ?? createVec3()),
      accumulatedMotorImpulse: Number.isFinite(Number(options.accumulatedMotorImpulse))
        ? Number(options.accumulatedMotorImpulse)
        : 0,
      enabled: options.enabled !== false,
      userData: cloneNullableValue(options.userData)
    });
  }

  createFixedJoint(options = {}) {
    const localAnchorA = cloneVec3(options.localAnchorA ?? createVec3());
    const localAnchorB = cloneVec3(options.localAnchorB ?? createVec3());
    const localAxisA = normalizeVec3(options.localAxisA ?? createVec3(0, 1, 0), createVec3(0, 1, 0));
    const localAxisB = normalizeVec3(options.localAxisB ?? createVec3(0, 1, 0), createVec3(0, 1, 0));

    return this.store({
      id: this.allocateId(options.id),
      type: 'fixed-joint',
      bodyAId: String(options.bodyAId ?? '').trim() || null,
      bodyBId: String(options.bodyBId ?? '').trim() || null,
      collideConnected: options.collideConnected === true,
      localAnchorA,
      localAnchorB,
      localAxisA,
      localAxisB,
      localReferenceA: normalizeVec3(options.localReferenceA ?? createVec3(1, 0, 0), createVec3(1, 0, 0)),
      localReferenceB: normalizeVec3(options.localReferenceB ?? createVec3(1, 0, 0), createVec3(1, 0, 0)),
      distance: 0,
      minDistance: null,
      maxDistance: null,
      springFrequency: 0,
      dampingRatio: 0,
      lowerAngle: null,
      upperAngle: null,
      angularDamping: 0,
      breakForce: toOptionalNonNegativeNumber(options.breakForce, null),
      breakTorque: toOptionalNonNegativeNumber(options.breakTorque, null),
      broken: false,
      lastAppliedForce: 0,
      lastAppliedTorque: 0,
      motorEnabled: false,
      motorMode: 'speed',
      motorSpeed: 0,
      motorTargetAngle: 0,
      motorServoGain: 8,
      maxMotorTorque: 0,
      accumulatedImpulse: 0,
      accumulatedLinearImpulse: cloneVec3(options.accumulatedLinearImpulse ?? createVec3()),
      accumulatedAngularImpulse: cloneVec3(options.accumulatedAngularImpulse ?? createVec3()),
      accumulatedMotorImpulse: 0,
      enabled: options.enabled !== false,
      userData: cloneNullableValue(options.userData)
    });
  }
}
