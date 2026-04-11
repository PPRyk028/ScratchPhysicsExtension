import { createTangentBasis } from '../collision/support.js';
import { createIdentityQuat, inverseRotateVec3ByQuat, rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, addVec3, cloneVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';

const WORLD_AXES = Object.freeze([
  Object.freeze({ x: 1, y: 0, z: 0 }),
  Object.freeze({ x: 0, y: 1, z: 0 }),
  Object.freeze({ x: 0, y: 0, z: 1 })
]);

function clampNumber(value, minValue, maxValue) {
  return Math.max(minValue, Math.min(maxValue, value));
}

function toOptionalFiniteNumber(value) {
  if (value === undefined || value === null || value === '') {
    return null;
  }

  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
}

function createEmptyJointSolverStats(iterations) {
  return {
    iterations,
    jointCount: 0,
    warmStartedJointCount: 0,
    solvedJointCount: 0,
    skippedJointCount: 0,
    jointImpulsesApplied: 0,
    maxJointError: 0,
    jointResults: []
  };
}

function createJointSolveResult(joint) {
  return {
    jointId: joint.id,
    jointType: joint.type,
    linearImpulse: 0,
    angularImpulse: 0,
    motorImpulse: 0,
    maxLinearError: 0,
    maxAngularError: 0
  };
}

function getDynamicBody(bodyRegistry, bodyId) {
  if (!bodyId) {
    return null;
  }

  const body = bodyRegistry.getMutable(bodyId);
  if (!body || !body.enabled || body.motionType !== 'dynamic' || body.sleeping) {
    return null;
  }

  return body;
}

function getAnyBody(bodyRegistry, bodyId) {
  if (!bodyId) {
    return null;
  }

  return bodyRegistry.get(bodyId);
}

function applyInverseInertiaWorld(body, worldVector) {
  if (!body || body.motionType !== 'dynamic') {
    return createVec3();
  }

  const rotation = body.rotation ?? createIdentityQuat();
  const localVector = inverseRotateVec3ByQuat(rotation, worldVector ?? createVec3());
  const localResult = createVec3(
    localVector.x * Number(body.inverseInertia?.x ?? 0),
    localVector.y * Number(body.inverseInertia?.y ?? 0),
    localVector.z * Number(body.inverseInertia?.z ?? 0)
  );
  return rotateVec3ByQuat(rotation, localResult);
}

function getAnchorOffset(body, localAnchor) {
  if (!body) {
    return createVec3();
  }

  return rotateVec3ByQuat(body.rotation ?? createIdentityQuat(), localAnchor ?? createVec3());
}

function getWorldAnchor(body, localAnchor) {
  if (!body) {
    return cloneVec3(localAnchor ?? createVec3());
  }

  return addVec3(body.position, getAnchorOffset(body, localAnchor));
}

function getVelocityAtOffset(body, offset) {
  if (!body) {
    return createVec3();
  }

  return addVec3(body.linearVelocity, crossVec3(body.angularVelocity, offset));
}

function getRelativeVelocity(bodyA, bodyB, offsetA, offsetB) {
  return subtractVec3(
    getVelocityAtOffset(bodyB, offsetB),
    getVelocityAtOffset(bodyA, offsetA)
  );
}

function applyLinearImpulse(bodyA, bodyB, offsetA, offsetB, impulse) {
  if (bodyA) {
    const impulseA = scaleVec3(impulse, -1);
    bodyA.linearVelocity = addScaledVec3(bodyA.linearVelocity, impulseA, bodyA.inverseMass);
    bodyA.angularVelocity = addVec3(bodyA.angularVelocity, applyInverseInertiaWorld(bodyA, crossVec3(offsetA, impulseA)));
  }

  if (bodyB) {
    bodyB.linearVelocity = addScaledVec3(bodyB.linearVelocity, impulse, bodyB.inverseMass);
    bodyB.angularVelocity = addVec3(bodyB.angularVelocity, applyInverseInertiaWorld(bodyB, crossVec3(offsetB, impulse)));
  }
}

function applyAngularImpulse(bodyA, bodyB, impulse) {
  if (bodyA) {
    bodyA.angularVelocity = addVec3(bodyA.angularVelocity, applyInverseInertiaWorld(bodyA, scaleVec3(impulse, -1)));
  }

  if (bodyB) {
    bodyB.angularVelocity = addVec3(bodyB.angularVelocity, applyInverseInertiaWorld(bodyB, impulse));
  }
}

function computeLinearEffectiveMass(bodyA, bodyB, offsetA, offsetB, direction) {
  let inverseMassSum = (bodyA?.inverseMass ?? 0) + (bodyB?.inverseMass ?? 0);

  if (bodyA) {
    const angularMassA = crossVec3(applyInverseInertiaWorld(bodyA, crossVec3(offsetA, direction)), offsetA);
    inverseMassSum += dotVec3(direction, angularMassA);
  }

  if (bodyB) {
    const angularMassB = crossVec3(applyInverseInertiaWorld(bodyB, crossVec3(offsetB, direction)), offsetB);
    inverseMassSum += dotVec3(direction, angularMassB);
  }

  return inverseMassSum;
}

function computeAngularEffectiveMass(bodyA, bodyB, direction) {
  const angularMassA = bodyA ? applyInverseInertiaWorld(bodyA, direction) : createVec3();
  const angularMassB = bodyB ? applyInverseInertiaWorld(bodyB, direction) : createVec3();
  return dotVec3(direction, addVec3(angularMassA, angularMassB));
}

function getRelativeAngularVelocity(bodyA, bodyB) {
  return subtractVec3(bodyB?.angularVelocity ?? createVec3(), bodyA?.angularVelocity ?? createVec3());
}

function prepareJointContext(bodyRegistry, joint) {
  const bodyA = getDynamicBody(bodyRegistry, joint.bodyAId);
  const bodyB = getDynamicBody(bodyRegistry, joint.bodyBId);
  const anchorBodyA = bodyA ?? getAnyBody(bodyRegistry, joint.bodyAId);
  const anchorBodyB = bodyB ?? getAnyBody(bodyRegistry, joint.bodyBId);

  return {
    joint,
    bodyA,
    bodyB,
    anchorBodyA,
    anchorBodyB,
    result: createJointSolveResult(joint)
  };
}

function refreshJointFrame(context) {
  const offsetA = getAnchorOffset(context.anchorBodyA, context.joint.localAnchorA);
  const offsetB = getAnchorOffset(context.anchorBodyB, context.joint.localAnchorB);
  const worldAnchorA = getWorldAnchor(context.anchorBodyA, context.joint.localAnchorA);
  const worldAnchorB = getWorldAnchor(context.anchorBodyB, context.joint.localAnchorB);
  const delta = subtractVec3(worldAnchorB, worldAnchorA);

  return {
    offsetA,
    offsetB,
    worldAnchorA,
    worldAnchorB,
    delta
  };
}

function getDistanceConstraintState(joint, currentDistance) {
  const minDistance = toOptionalFiniteNumber(joint.minDistance);
  const maxDistance = toOptionalFiniteNumber(joint.maxDistance);
  const restDistance = Number.isFinite(Number(joint.distance)) ? Math.max(0, Number(joint.distance)) : currentDistance;
  const springFrequency = Math.max(0, Number(joint.springFrequency ?? 0));

  if (minDistance !== null && currentDistance < minDistance) {
    return {
      active: true,
      mode: 'min-limit',
      jointError: currentDistance - minDistance,
      springFrequency
    };
  }

  if (maxDistance !== null && currentDistance > maxDistance) {
    return {
      active: true,
      mode: 'max-limit',
      jointError: currentDistance - maxDistance,
      springFrequency
    };
  }

  if ((minDistance !== null || maxDistance !== null) && springFrequency <= 0) {
    return {
      active: false,
      mode: 'range',
      jointError: 0,
      springFrequency
    };
  }

  return {
    active: true,
    mode: springFrequency > 0 ? 'spring' : 'distance',
    jointError: currentDistance - restDistance,
    springFrequency
  };
}

function buildAngularFrame(context) {
  const joint = context.joint;
  const axisA = normalizeVec3(
    rotateVec3ByQuat(context.anchorBodyA?.rotation ?? createIdentityQuat(), joint.localAxisA ?? createVec3(0, 1, 0)),
    createVec3(0, 1, 0)
  );
  const axisB = normalizeVec3(
    rotateVec3ByQuat(context.anchorBodyB?.rotation ?? createIdentityQuat(), joint.localAxisB ?? createVec3(0, 1, 0)),
    createVec3(0, 1, 0)
  );
  const alignmentAxis = normalizeVec3(addVec3(axisA, axisB), axisA);
  const tangentBasis = createTangentBasis(alignmentAxis);
  const referenceA = normalizeVec3(
    rotateVec3ByQuat(context.anchorBodyA?.rotation ?? createIdentityQuat(), joint.localReferenceA ?? tangentBasis.tangentA),
    tangentBasis.tangentA
  );
  const referenceB = normalizeVec3(
    rotateVec3ByQuat(context.anchorBodyB?.rotation ?? createIdentityQuat(), joint.localReferenceB ?? tangentBasis.tangentA),
    tangentBasis.tangentA
  );
  const angularError = Math.atan2(
    dotVec3(crossVec3(referenceA, referenceB), alignmentAxis),
    dotVec3(referenceA, referenceB)
  );

  return {
    axisA,
    axisB,
    alignmentAxis,
    tangentBasis,
    referenceA,
    referenceB,
    angularError
  };
}

function applyWarmStart(context, frame, stats) {
  const joint = context.joint;
  let warmed = false;

  if (joint.type === 'distance-joint' && Math.abs(Number(joint.accumulatedImpulse ?? 0)) > 1e-8) {
    const distanceNormal = normalizeVec3(frame.delta, createVec3(1, 0, 0));
    applyLinearImpulse(context.bodyA, context.bodyB, frame.offsetA, frame.offsetB, scaleVec3(distanceNormal, joint.accumulatedImpulse));
    stats.jointImpulsesApplied += Math.abs(joint.accumulatedImpulse);
    warmed = true;
  }

  if (lengthSquaredVec3(joint.accumulatedLinearImpulse ?? createVec3()) > 1e-8) {
    applyLinearImpulse(context.bodyA, context.bodyB, frame.offsetA, frame.offsetB, joint.accumulatedLinearImpulse);
    stats.jointImpulsesApplied += Math.sqrt(lengthSquaredVec3(joint.accumulatedLinearImpulse));
    warmed = true;
  }

  if (lengthSquaredVec3(joint.accumulatedAngularImpulse ?? createVec3()) > 1e-8) {
    applyAngularImpulse(context.bodyA, context.bodyB, joint.accumulatedAngularImpulse);
    stats.jointImpulsesApplied += Math.sqrt(lengthSquaredVec3(joint.accumulatedAngularImpulse));
    warmed = true;
  }

  if (joint.type === 'hinge-joint' && joint.motorEnabled && Math.abs(Number(joint.accumulatedMotorImpulse ?? 0)) > 1e-8) {
    const angularFrame = buildAngularFrame(context);
    applyAngularImpulse(context.bodyA, context.bodyB, scaleVec3(angularFrame.alignmentAxis, Number(joint.accumulatedMotorImpulse)));
    stats.jointImpulsesApplied += Math.abs(Number(joint.accumulatedMotorImpulse));
    warmed = true;
  }

  if (warmed) {
    stats.warmStartedJointCount += 1;
  }
}

function solveDistanceJoint(context, frame, deltaTime, baumgarte, allowedStretch, stats) {
  const joint = context.joint;
  const currentDistance = Math.sqrt(lengthSquaredVec3(frame.delta));
  const normal = normalizeVec3(frame.delta, createVec3(1, 0, 0));
  const constraintState = getDistanceConstraintState(joint, currentDistance);
  const jointError = constraintState.jointError;
  stats.maxJointError = Math.max(stats.maxJointError, Math.abs(jointError));
  context.result.maxLinearError = Math.max(context.result.maxLinearError, Math.abs(jointError));

  if (!constraintState.active) {
    return;
  }

  const effectiveMass = computeLinearEffectiveMass(context.bodyA, context.bodyB, frame.offsetA, frame.offsetB, normal);
  if (effectiveMass <= 1e-8) {
    stats.skippedJointCount += 1;
    return;
  }

  const relativeVelocity = getRelativeVelocity(context.bodyA, context.bodyB, frame.offsetA, frame.offsetB);
  const relativeNormalVelocity = dotVec3(relativeVelocity, normal);
  const hardConstraint = constraintState.mode !== 'spring';
  const positionBias = hardConstraint && Math.abs(jointError) > allowedStretch
    ? (baumgarte * jointError) / deltaTime
    : 0;
  const springFrequency = constraintState.mode === 'spring' ? constraintState.springFrequency : 0;
  const dampingRatio = springFrequency > 0 ? Math.max(0, Number(joint.dampingRatio ?? 0)) : 0;
  const omega = springFrequency > 0 ? (Math.PI * 2 * springFrequency) : 0;
  const springVelocityBias = springFrequency > 0 ? (omega * omega * jointError * deltaTime) : 0;
  const dampingScale = springFrequency > 0 ? (1 + (2 * dampingRatio * omega * deltaTime)) : 1;
  const impulseDelta = -((relativeNormalVelocity * dampingScale) + positionBias + springVelocityBias) / effectiveMass;

  if (Math.abs(impulseDelta) <= 1e-8) {
    return;
  }

  joint.accumulatedImpulse = Number(joint.accumulatedImpulse ?? 0) + impulseDelta;
  applyLinearImpulse(context.bodyA, context.bodyB, frame.offsetA, frame.offsetB, scaleVec3(normal, impulseDelta));
  context.result.linearImpulse += Math.abs(impulseDelta);
  stats.solvedJointCount += 1;
  stats.jointImpulsesApplied += Math.abs(impulseDelta);
}

function solvePointToPointLinear(context, frame, deltaTime, baumgarte, allowedStretch, stats) {
  const joint = context.joint;

  for (const axis of WORLD_AXES) {
    const axisDirection = createVec3(axis.x, axis.y, axis.z);
    const effectiveMass = computeLinearEffectiveMass(context.bodyA, context.bodyB, frame.offsetA, frame.offsetB, axisDirection);
    if (effectiveMass <= 1e-8) {
      stats.skippedJointCount += 1;
      continue;
    }

    const relativeVelocity = getRelativeVelocity(context.bodyA, context.bodyB, frame.offsetA, frame.offsetB);
    const errorAlongAxis = dotVec3(frame.delta, axisDirection);
    stats.maxJointError = Math.max(stats.maxJointError, Math.abs(errorAlongAxis));
    context.result.maxLinearError = Math.max(context.result.maxLinearError, Math.abs(errorAlongAxis));
    const relativeAxisVelocity = dotVec3(relativeVelocity, axisDirection);
    const positionBias = Math.abs(errorAlongAxis) > allowedStretch
      ? (baumgarte * errorAlongAxis) / deltaTime
      : 0;
    const impulseDelta = -(relativeAxisVelocity + positionBias) / effectiveMass;

    if (Math.abs(impulseDelta) <= 1e-8) {
      continue;
    }

    const impulse = scaleVec3(axisDirection, impulseDelta);
    joint.accumulatedLinearImpulse = addVec3(joint.accumulatedLinearImpulse ?? createVec3(), impulse);
    applyLinearImpulse(context.bodyA, context.bodyB, frame.offsetA, frame.offsetB, impulse);
    context.result.linearImpulse += Math.abs(impulseDelta);
    stats.solvedJointCount += 1;
    stats.jointImpulsesApplied += Math.abs(impulseDelta);
  }
}

function solveAngularAxisAlignment(context, angularFrame, deltaTime, baumgarte, allowedStretch, stats) {
  const joint = context.joint;
  const alignmentError = crossVec3(angularFrame.axisA, angularFrame.axisB);
  const relativeAngularVelocity = getRelativeAngularVelocity(context.bodyA, context.bodyB);

  for (const tangent of [angularFrame.tangentBasis.tangentA, angularFrame.tangentBasis.tangentB]) {
    const effectiveMass = computeAngularEffectiveMass(context.bodyA, context.bodyB, tangent);
    if (effectiveMass <= 1e-8) {
      stats.skippedJointCount += 1;
      continue;
    }

    const errorAlongTangent = dotVec3(alignmentError, tangent);
    stats.maxJointError = Math.max(stats.maxJointError, Math.abs(errorAlongTangent));
    context.result.maxAngularError = Math.max(context.result.maxAngularError, Math.abs(errorAlongTangent));
    const relativeAngularVelocityAlongTangent = dotVec3(relativeAngularVelocity, tangent);
    const positionBias = Math.abs(errorAlongTangent) > allowedStretch
      ? (baumgarte * errorAlongTangent) / deltaTime
      : 0;
    const impulseDelta = -(relativeAngularVelocityAlongTangent + positionBias) / effectiveMass;

    if (Math.abs(impulseDelta) <= 1e-8) {
      continue;
    }

    const impulse = scaleVec3(tangent, impulseDelta);
    joint.accumulatedAngularImpulse = addVec3(joint.accumulatedAngularImpulse ?? createVec3(), impulse);
    applyAngularImpulse(context.bodyA, context.bodyB, impulse);
    context.result.angularImpulse += Math.abs(impulseDelta);
    stats.solvedJointCount += 1;
    stats.jointImpulsesApplied += Math.abs(impulseDelta);
  }
}

function solveAngularAngleTarget(context, angularFrame, targetAngle, deltaTime, baumgarte, allowedStretch, stats) {
  const angleError = angularFrame.angularError - targetAngle;
  stats.maxJointError = Math.max(stats.maxJointError, Math.abs(angleError));
  context.result.maxAngularError = Math.max(context.result.maxAngularError, Math.abs(angleError));

  if (Math.abs(angleError) <= 1e-8) {
    return;
  }

  const effectiveMass = computeAngularEffectiveMass(context.bodyA, context.bodyB, angularFrame.alignmentAxis);
  if (effectiveMass <= 1e-8) {
    stats.skippedJointCount += 1;
    return;
  }

  const relativeAngularVelocity = getRelativeAngularVelocity(context.bodyA, context.bodyB);
  const relativeAxisVelocity = dotVec3(relativeAngularVelocity, angularFrame.alignmentAxis);
  const positionBias = Math.abs(angleError) > allowedStretch
    ? (baumgarte * angleError) / deltaTime
    : 0;
  const impulseDelta = -(relativeAxisVelocity + positionBias) / effectiveMass;

  if (Math.abs(impulseDelta) <= 1e-8) {
    return;
  }

  const impulse = scaleVec3(angularFrame.alignmentAxis, impulseDelta);
  context.joint.accumulatedAngularImpulse = addVec3(context.joint.accumulatedAngularImpulse ?? createVec3(), impulse);
  applyAngularImpulse(context.bodyA, context.bodyB, impulse);
  context.result.angularImpulse += Math.abs(impulseDelta);
  stats.solvedJointCount += 1;
  stats.jointImpulsesApplied += Math.abs(impulseDelta);
}

function solveHingeAngularLimits(context, angularFrame, deltaTime, baumgarte, allowedStretch, stats) {
  const joint = context.joint;
  const lowerAngle = toOptionalFiniteNumber(joint.lowerAngle);
  const upperAngle = toOptionalFiniteNumber(joint.upperAngle);
  if (lowerAngle === null && upperAngle === null) {
    return;
  }

  const clampedAngle = clampNumber(
    angularFrame.angularError,
    lowerAngle ?? angularFrame.angularError,
    upperAngle ?? angularFrame.angularError
  );
  solveAngularAngleTarget(context, angularFrame, clampedAngle, deltaTime, baumgarte, allowedStretch, stats);
}

function solveHingeMotor(context, angularFrame, deltaTime, stats) {
  const joint = context.joint;
  if (!joint.motorEnabled || Number(joint.maxMotorTorque ?? 0) <= 1e-8) {
    return;
  }

  const lowerAngle = toOptionalFiniteNumber(joint.lowerAngle);
  const upperAngle = toOptionalFiniteNumber(joint.upperAngle);
  let motorSpeed = Number(joint.motorSpeed ?? 0);
  if (joint.motorMode === 'servo') {
    const servoTarget = clampNumber(
      Number(joint.motorTargetAngle ?? 0),
      lowerAngle ?? Number(joint.motorTargetAngle ?? 0),
      upperAngle ?? Number(joint.motorTargetAngle ?? 0)
    );
    const servoGain = Math.max(0, Number(joint.motorServoGain ?? 8));
    const servoMaxSpeed = Math.max(0, Number(joint.motorSpeed ?? 0));
    const servoSpeed = (servoTarget - angularFrame.angularError) * servoGain;
    motorSpeed = clampNumber(servoSpeed, -servoMaxSpeed, servoMaxSpeed);
  }

  if ((upperAngle !== null && angularFrame.angularError >= upperAngle - 1e-4 && motorSpeed > 0) ||
    (lowerAngle !== null && angularFrame.angularError <= lowerAngle + 1e-4 && motorSpeed < 0)) {
    return;
  }

  const effectiveMass = computeAngularEffectiveMass(context.bodyA, context.bodyB, angularFrame.alignmentAxis);
  if (effectiveMass <= 1e-8) {
    stats.skippedJointCount += 1;
    return;
  }

  const relativeAngularVelocity = getRelativeAngularVelocity(context.bodyA, context.bodyB);
  const relativeAxisVelocity = dotVec3(relativeAngularVelocity, angularFrame.alignmentAxis);
  const maxImpulse = Number(joint.maxMotorTorque ?? 0) * deltaTime;
  const impulseDelta = (motorSpeed - relativeAxisVelocity) / effectiveMass;
  const nextImpulse = clampNumber(
    Number(joint.accumulatedMotorImpulse ?? 0) + impulseDelta,
    -maxImpulse,
    maxImpulse
  );
  const appliedImpulse = nextImpulse - Number(joint.accumulatedMotorImpulse ?? 0);

  if (Math.abs(appliedImpulse) <= 1e-8) {
    return;
  }

  joint.accumulatedMotorImpulse = nextImpulse;
  applyAngularImpulse(context.bodyA, context.bodyB, scaleVec3(angularFrame.alignmentAxis, appliedImpulse));
  context.result.motorImpulse += Math.abs(appliedImpulse);
  stats.solvedJointCount += 1;
  stats.jointImpulsesApplied += Math.abs(appliedImpulse);
}

function solveHingeAngularDamping(context, angularFrame, deltaTime, stats) {
  const damping = Math.max(0, Number(context.joint.angularDamping ?? 0));
  if (damping <= 1e-8) {
    return;
  }

  const effectiveMass = computeAngularEffectiveMass(context.bodyA, context.bodyB, angularFrame.alignmentAxis);
  if (effectiveMass <= 1e-8) {
    stats.skippedJointCount += 1;
    return;
  }

  const relativeAngularVelocity = getRelativeAngularVelocity(context.bodyA, context.bodyB);
  const relativeAxisVelocity = dotVec3(relativeAngularVelocity, angularFrame.alignmentAxis);
  const dampingFactor = 1 - Math.exp(-(damping * deltaTime));
  const impulseDelta = -((relativeAxisVelocity * dampingFactor) / effectiveMass);

  if (Math.abs(impulseDelta) <= 1e-8) {
    return;
  }

  const impulse = scaleVec3(angularFrame.alignmentAxis, impulseDelta);
  context.joint.accumulatedAngularImpulse = addVec3(context.joint.accumulatedAngularImpulse ?? createVec3(), impulse);
  applyAngularImpulse(context.bodyA, context.bodyB, impulse);
  context.result.angularImpulse += Math.abs(impulseDelta);
  stats.solvedJointCount += 1;
  stats.jointImpulsesApplied += Math.abs(impulseDelta);
}

function solveFixedJoint(context, frame, deltaTime, baumgarte, allowedStretch, stats) {
  solvePointToPointLinear(context, frame, deltaTime, baumgarte, allowedStretch, stats);
  const angularFrame = buildAngularFrame(context);
  solveAngularAxisAlignment(context, angularFrame, deltaTime, baumgarte, allowedStretch, stats);
  solveAngularAngleTarget(context, angularFrame, 0, deltaTime, baumgarte, allowedStretch, stats);
}

function solveHingeJoint(context, frame, deltaTime, baumgarte, allowedStretch, stats) {
  solvePointToPointLinear(context, frame, deltaTime, baumgarte, allowedStretch, stats);
  const angularFrame = buildAngularFrame(context);
  solveAngularAxisAlignment(context, angularFrame, deltaTime, baumgarte, allowedStretch, stats);
  solveHingeAngularLimits(context, angularFrame, deltaTime, baumgarte, allowedStretch, stats);
  solveHingeMotor(context, angularFrame, deltaTime, stats);
  solveHingeAngularDamping(context, angularFrame, deltaTime, stats);
}

function solveJoint(context, deltaTime, iterations, baumgarte, allowedStretch, stats) {
  const joint = context.joint;
  if (!context.bodyA && !context.bodyB) {
    stats.skippedJointCount += 1;
    return;
  }

  for (let iteration = 0; iteration < iterations; iteration += 1) {
    const frame = refreshJointFrame(context);

    if (joint.type === 'distance-joint') {
      solveDistanceJoint(context, frame, deltaTime, baumgarte, allowedStretch, stats);
      continue;
    }

    if (joint.type === 'point-to-point-joint') {
      solvePointToPointLinear(context, frame, deltaTime, baumgarte, allowedStretch, stats);
      continue;
    }

    if (joint.type === 'hinge-joint') {
      solveHingeJoint(context, frame, deltaTime, baumgarte, allowedStretch, stats);
      continue;
    }

    if (joint.type === 'fixed-joint') {
      solveFixedJoint(context, frame, deltaTime, baumgarte, allowedStretch, stats);
      continue;
    }

    stats.skippedJointCount += 1;
  }
}

export function solveJointConstraints(options = {}) {
  const joints = Array.isArray(options.joints) ? options.joints.filter((joint) => joint?.enabled !== false) : [];
  const bodyRegistry = options.bodyRegistry;
  const deltaTime = Math.max(Number(options.deltaTime ?? 0), 1e-8);
  const iterations = Math.max(1, Math.floor(Number(options.iterations ?? 6)));
  const baumgarte = Number(options.baumgarte ?? 0.15);
  const allowedStretch = Math.max(0, Number(options.allowedStretch ?? 0.001));
  const stats = createEmptyJointSolverStats(iterations);
  stats.jointCount = joints.length;

  const preparedJoints = joints.map((joint) => prepareJointContext(bodyRegistry, joint));

  for (const context of preparedJoints) {
    applyWarmStart(context, refreshJointFrame(context), stats);
  }

  for (const context of preparedJoints) {
    solveJoint(context, deltaTime, iterations, baumgarte, allowedStretch, stats);
  }

  stats.jointResults = preparedJoints.map((context) => ({
    ...context.result
  }));

  return stats;
}

export function solveDistanceJointConstraints(options = {}) {
  return solveJointConstraints({
    ...options,
    joints: (Array.isArray(options.joints) ? options.joints : []).filter((joint) => joint?.type === 'distance-joint')
  });
}
