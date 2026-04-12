import { createDebugLine, createDebugPoint, DEFAULT_DEBUG_COLORS } from '../debug/debug-primitives.js';
import { integrateQuat, inverseRotateVec3ByQuat, rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, addVec3, cloneVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';
import { createClothSheetDefinition } from './cloth.js';
import { cloneXpbdConstraint, resetConstraintLambdas, solveDistanceConstraint, solvePinConstraint } from './constraints.js';

function toPositiveNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
}

function toNonNegativeNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function cloneParticle(particle) {
  return {
    ...particle,
    position: cloneVec3(particle.position),
    previousPosition: cloneVec3(particle.previousPosition),
    predictedPosition: cloneVec3(particle.predictedPosition),
    velocity: cloneVec3(particle.velocity),
    pinTarget: particle.pinTarget ? cloneVec3(particle.pinTarget) : null
  };
}

function cloneCloth(cloth) {
  return {
    ...cloth,
    position: cloneVec3(cloth.position),
    particleIds: [...cloth.particleIds],
    particleIdGrid: cloth.particleIdGrid.map((row) => [...row]),
    structuralEdges: cloth.structuralEdges.map((edge) => ({ ...edge })),
    shearEdges: cloth.shearEdges.map((edge) => ({ ...edge })),
    bendEdges: cloth.bendEdges.map((edge) => ({ ...edge })),
    distanceConstraints: cloth.distanceConstraints.map((constraint) => cloneXpbdConstraint(constraint)),
    pinConstraints: cloth.pinConstraints.map((constraint) => cloneXpbdConstraint(constraint))
  };
}

function createEmptyStepStats(iterations) {
  return {
    requestedDeltaSeconds: 0,
    substeps: 0,
    iterations,
    solvedDistanceConstraints: 0,
    solvedPinConstraints: 0,
    solvedStaticCollisions: 0,
    solvedDynamicCollisions: 0,
    solvedSelfCollisions: 0,
    activeParticleCount: 0,
    activeStaticColliderCount: 0,
    activeDynamicColliderCount: 0,
    affectedDynamicBodyCount: 0
  };
}

function cloneClothCollisionContact(contact) {
  return {
    ...contact,
    point: cloneVec3(contact.point),
    normal: cloneVec3(contact.normal),
    particlePosition: cloneVec3(contact.particlePosition)
  };
}

function cloneClothSelfCollisionContact(contact) {
  return {
    ...contact,
    point: cloneVec3(contact.point),
    normal: cloneVec3(contact.normal),
    particleAPosition: cloneVec3(contact.particleAPosition),
    particleBPosition: cloneVec3(contact.particleBPosition)
  };
}

function shouldSkipSelfCollisionPair(particleA, particleB) {
  const rowDistance = Math.abs(Number(particleA?.rowIndex ?? 0) - Number(particleB?.rowIndex ?? 0));
  const columnDistance = Math.abs(Number(particleA?.columnIndex ?? 0) - Number(particleB?.columnIndex ?? 0));
  if (rowDistance <= 1 && columnDistance <= 1) {
    return true;
  }

  if ((rowDistance === 0 && columnDistance <= 2) || (columnDistance === 0 && rowDistance <= 2)) {
    return true;
  }

  return false;
}

function getColliderPose(collider) {
  if (typeof collider?.getPose === 'function') {
    return collider.getPose();
  }

  return collider?.pose ?? null;
}

function applyBodyInverseInertiaWorld(body, worldVector) {
  if (!body || body.motionType !== 'dynamic' || !body.inverseInertia) {
    return createVec3();
  }

  const localVector = inverseRotateVec3ByQuat(body.rotation, worldVector ?? createVec3());
  const localResult = createVec3(
    localVector.x * Number(body.inverseInertia.x ?? 0),
    localVector.y * Number(body.inverseInertia.y ?? 0),
    localVector.z * Number(body.inverseInertia.z ?? 0)
  );

  return rotateVec3ByQuat(body.rotation, localResult);
}

function getBodyPointVelocity(body, worldPoint) {
  if (!body) {
    return createVec3();
  }

  const offset = subtractVec3(worldPoint, body.position);
  return addVec3(body.linearVelocity ?? createVec3(), crossVec3(body.angularVelocity ?? createVec3(), offset));
}

function computeDynamicCollisionEffectiveMass(body, contactPoint, normal, particleInverseMass) {
  let inverseMassSum = Number(particleInverseMass ?? 0);
  if (!body || body.motionType !== 'dynamic') {
    return inverseMassSum;
  }

  inverseMassSum += Number(body.inverseMass ?? 0);
  const offset = subtractVec3(contactPoint, body.position);
  const angularTerm = crossVec3(offset, normal);
  const angularMass = applyBodyInverseInertiaWorld(body, angularTerm);
  inverseMassSum += dotVec3(normal, crossVec3(angularMass, offset));
  return inverseMassSum;
}

function collectConstraints(cloths) {
  const distanceConstraints = [];
  const pinConstraints = [];
  for (const cloth of cloths.values()) {
    distanceConstraints.push(...cloth.distanceConstraints);
    pinConstraints.push(...cloth.pinConstraints);
  }
  return {
    distanceConstraints,
    pinConstraints
  };
}

export class ParticleWorld {
  constructor(options = {}) {
    this.gravity = cloneVec3(options.gravity ?? createVec3(0, -9.81, 0));
    this.iterations = Math.max(1, Math.floor(toPositiveNumber(options.iterations, 6)));
    this.substeps = Math.max(1, Math.floor(toPositiveNumber(options.substeps, 1)));
    this.defaultDamping = toNonNegativeNumber(options.defaultDamping, 0.03);
    this.reset();
  }

  reset() {
    this.particles = new Map();
    this.cloths = new Map();
    this.simulationTick = 0;
    this.lastStepStats = createEmptyStepStats(this.iterations);
    this.lastCollisionContacts = [];
    this.lastSelfCollisionContacts = [];
  }

  setGravity(gravity) {
    this.gravity = cloneVec3(gravity);
  }

  createClothSheet(options = {}) {
    const cloth = createClothSheetDefinition({
      ...options,
      damping: options.damping ?? this.defaultDamping
    });

    for (const particle of cloth.particles) {
      this.particles.set(particle.id, particle);
    }

    const storedCloth = {
      ...cloth,
      particles: undefined
    };
    this.cloths.set(storedCloth.id, storedCloth);
    return this.getCloth(storedCloth.id);
  }

  configureCloth(id, options = {}) {
    const cloth = this.cloths.get(String(id ?? '').trim());
    if (!cloth) {
      return null;
    }

    if (options.damping !== undefined) {
      cloth.damping = toNonNegativeNumber(options.damping, cloth.damping);
    }
    if (options.collisionMargin !== undefined) {
      cloth.collisionMargin = toPositiveNumber(options.collisionMargin, cloth.collisionMargin);
    }
    if (options.stretchCompliance !== undefined) {
      cloth.stretchCompliance = toNonNegativeNumber(options.stretchCompliance, cloth.stretchCompliance);
      for (const constraint of cloth.distanceConstraints) {
        if (constraint.kind === 'stretch') {
          constraint.compliance = cloth.stretchCompliance;
        }
      }
    }
    if (options.shearCompliance !== undefined) {
      cloth.shearCompliance = toNonNegativeNumber(options.shearCompliance, cloth.shearCompliance);
      for (const constraint of cloth.distanceConstraints) {
        if (constraint.kind === 'shear') {
          constraint.compliance = cloth.shearCompliance;
        }
      }
    }
    if (options.bendCompliance !== undefined) {
      cloth.bendCompliance = toNonNegativeNumber(options.bendCompliance, cloth.bendCompliance);
      for (const constraint of cloth.distanceConstraints) {
        if (constraint.kind === 'bend') {
          constraint.compliance = cloth.bendCompliance;
        }
      }
    }
    if (options.selfCollisionEnabled !== undefined) {
      cloth.selfCollisionEnabled = Boolean(options.selfCollisionEnabled);
    }
    if (options.selfCollisionDistance !== undefined) {
      cloth.selfCollisionDistance = toPositiveNumber(options.selfCollisionDistance, cloth.selfCollisionDistance);
    }

    return this.getCloth(cloth.id);
  }

  hasCloth(id) {
    return this.cloths.has(id);
  }

  getCloth(id) {
    const cloth = this.cloths.get(id);
    return cloth ? cloneCloth(cloth) : null;
  }

  listCloths() {
    return Array.from(this.cloths.values(), (cloth) => cloneCloth(cloth));
  }

  getParticle(id) {
    const particle = this.particles.get(id);
    return particle ? cloneParticle(particle) : null;
  }

  listParticles() {
    return Array.from(this.particles.values(), (particle) => cloneParticle(particle));
  }

  countCloths() {
    return this.cloths.size;
  }

  countParticles() {
    return this.particles.size;
  }

  exportSceneDefinition() {
    return {
      schemaVersion: 'xpbd-scene@1',
      settings: {
        iterations: this.iterations,
        substeps: this.substeps,
        defaultDamping: this.defaultDamping
      },
      cloths: this.listCloths(),
      particles: this.listParticles()
    };
  }

  importSceneDefinition(sceneDefinition = {}) {
    this.reset();
    const scene = typeof sceneDefinition === 'object' && sceneDefinition !== null ? sceneDefinition : {};
    const settings = typeof scene.settings === 'object' && scene.settings !== null ? scene.settings : {};
    this.iterations = Math.max(1, Math.floor(toPositiveNumber(settings.iterations, this.iterations)));
    this.substeps = Math.max(1, Math.floor(toPositiveNumber(settings.substeps, this.substeps)));
    this.defaultDamping = toNonNegativeNumber(settings.defaultDamping, this.defaultDamping);

    const particlesById = new Map();
    for (const particle of Array.isArray(scene.particles) ? scene.particles : []) {
      if (!particle?.id) {
        continue;
      }
      particlesById.set(particle.id, cloneParticle({
        ...particle,
        previousPosition: particle.previousPosition ?? particle.position,
        predictedPosition: particle.predictedPosition ?? particle.position,
        velocity: particle.velocity ?? createVec3(),
        pinTarget: particle.pinTarget ?? null
      }));
    }

    for (const clothRecord of Array.isArray(scene.cloths) ? scene.cloths : []) {
      if (!clothRecord?.id) {
        continue;
      }

      const cloth = cloneCloth({
        ...clothRecord,
        position: clothRecord.position ?? createVec3(),
        particleIds: Array.isArray(clothRecord.particleIds) ? clothRecord.particleIds : [],
        particleIdGrid: Array.isArray(clothRecord.particleIdGrid) ? clothRecord.particleIdGrid : [],
        structuralEdges: Array.isArray(clothRecord.structuralEdges) ? clothRecord.structuralEdges : [],
        shearEdges: Array.isArray(clothRecord.shearEdges) ? clothRecord.shearEdges : [],
        bendEdges: Array.isArray(clothRecord.bendEdges) ? clothRecord.bendEdges : [],
        distanceConstraints: Array.isArray(clothRecord.distanceConstraints) ? clothRecord.distanceConstraints : [],
        pinConstraints: Array.isArray(clothRecord.pinConstraints) ? clothRecord.pinConstraints : []
      });

      this.cloths.set(cloth.id, cloth);
      for (const particleId of cloth.particleIds) {
        const particle = particlesById.get(particleId);
        if (particle) {
          this.particles.set(particleId, particle);
        }
      }
    }

    return {
      clothCount: this.countCloths(),
      particleCount: this.countParticles()
    };
  }

  resolveParticleCollision(particle, collisionMargin, collider) {
    const shape = collider?.shape;
    const pose = getColliderPose(collider);
    if (!particle || !shape || !pose) {
      return null;
    }

    const localPoint = inverseRotateVec3ByQuat(pose.rotation, subtractVec3(particle.predictedPosition, pose.position));

    if (shape.type === 'box') {
      const halfExtents = shape.geometry?.halfExtents;
      if (!halfExtents) {
        return null;
      }

      const expandedHalfExtents = createVec3(
        Number(halfExtents.x ?? 0) + collisionMargin,
        Number(halfExtents.y ?? 0) + collisionMargin,
        Number(halfExtents.z ?? 0) + collisionMargin
      );
      const distanceToSurface = {
        x: expandedHalfExtents.x - Math.abs(localPoint.x),
        y: expandedHalfExtents.y - Math.abs(localPoint.y),
        z: expandedHalfExtents.z - Math.abs(localPoint.z)
      };

      if (distanceToSurface.x < 0 || distanceToSurface.y < 0 || distanceToSurface.z < 0) {
        return null;
      }

      let axis = 'x';
      if (distanceToSurface.y < distanceToSurface[axis]) {
        axis = 'y';
      }
      if (distanceToSurface.z < distanceToSurface[axis]) {
        axis = 'z';
      }

      const sign = localPoint[axis] >= 0 ? 1 : -1;
      const correctedLocalPoint = cloneVec3(localPoint);
      correctedLocalPoint[axis] = sign * expandedHalfExtents[axis];
      const localNormal = createVec3(
        axis === 'x' ? sign : 0,
        axis === 'y' ? sign : 0,
        axis === 'z' ? sign : 0
      );
      const worldNormal = rotateVec3ByQuat(pose.rotation, localNormal);
      const correctedPosition = addScaledVec3(pose.position, rotateVec3ByQuat(pose.rotation, correctedLocalPoint), 1);
      const shapePoint = addScaledVec3(correctedPosition, worldNormal, -collisionMargin);

      return {
        correctedPosition,
        normal: worldNormal,
        point: shapePoint,
        penetration: distanceToSurface[axis],
        colliderId: collider.colliderId,
        bodyId: collider.bodyId ?? null,
        motionType: collider.motionType ?? 'static',
        shapeType: shape.type
      };
    }

    if (shape.type === 'convex-hull') {
      const faces = Array.isArray(shape.geometry?.faces) ? shape.geometry.faces : [];
      if (!faces.length) {
        return null;
      }

      let limitingFace = null;
      let maxSignedDistance = -Infinity;
      for (const face of faces) {
        const signedDistance = dotVec3(face.normal, localPoint) - Number(face.planeOffset ?? 0);
        if (signedDistance > maxSignedDistance) {
          maxSignedDistance = signedDistance;
          limitingFace = face;
        }
      }

      if (!limitingFace || maxSignedDistance > collisionMargin) {
        return null;
      }

      const correctionDistance = collisionMargin - maxSignedDistance;
      const correctedLocalPoint = addScaledVec3(localPoint, limitingFace.normal, correctionDistance);
      const worldNormal = rotateVec3ByQuat(pose.rotation, limitingFace.normal);
      const correctedPosition = addScaledVec3(pose.position, rotateVec3ByQuat(pose.rotation, correctedLocalPoint), 1);
      const shapePoint = addScaledVec3(correctedPosition, worldNormal, -collisionMargin);

      return {
        correctedPosition,
        normal: worldNormal,
        point: shapePoint,
        penetration: correctionDistance,
        colliderId: collider.colliderId,
        bodyId: collider.bodyId ?? null,
        motionType: collider.motionType ?? 'static',
        shapeType: shape.type
      };
    }

    return null;
  }

  solveStaticCollisions(cloths, staticColliders, stepStats) {
    if (!Array.isArray(staticColliders) || !staticColliders.length) {
      return;
    }

    stepStats.activeStaticColliderCount = staticColliders.length;

    for (const cloth of cloths) {
      for (const particleId of cloth.particleIds) {
        const particle = this.particles.get(particleId);
        if (!particle || particle.inverseMass <= 0) {
          continue;
        }

        for (const staticCollider of staticColliders) {
          const collision = this.resolveParticleCollision(
            particle,
            Number(cloth.collisionMargin ?? 0.5),
            staticCollider
          );

          if (!collision) {
            continue;
          }

          particle.predictedPosition = cloneVec3(collision.correctedPosition);
          const inwardVelocity = dotVec3(particle.velocity, collision.normal);
          if (inwardVelocity < 0) {
            particle.velocity = addScaledVec3(particle.velocity, collision.normal, -inwardVelocity);
          }

          stepStats.solvedStaticCollisions += 1;
          if (this.lastCollisionContacts.length < 256) {
            this.lastCollisionContacts.push({
              clothId: cloth.id,
              particleId: particle.id,
              colliderId: collision.colliderId,
              bodyId: collision.bodyId,
              motionType: collision.motionType,
              shapeType: collision.shapeType,
              point: cloneVec3(collision.point),
              normal: cloneVec3(collision.normal),
              particlePosition: cloneVec3(particle.predictedPosition)
            });
          }
        }
      }
    }
  }

  solveDynamicCollisions(cloths, dynamicColliders, stepStats) {
    if (!Array.isArray(dynamicColliders) || !dynamicColliders.length) {
      return;
    }

    stepStats.activeDynamicColliderCount = dynamicColliders.length;
    const affectedBodies = new Set();

    for (const cloth of cloths) {
      const collisionMargin = Number(cloth.collisionMargin ?? 0.5);
      for (const particleId of cloth.particleIds) {
        const particle = this.particles.get(particleId);
        if (!particle || particle.inverseMass <= 0) {
          continue;
        }

        for (const dynamicCollider of dynamicColliders) {
          const body = dynamicCollider?.body ?? null;
          if (!body || body.motionType !== 'dynamic' || !body.enabled) {
            continue;
          }

          const collision = this.resolveParticleCollision(particle, collisionMargin, dynamicCollider);
          if (!collision) {
            continue;
          }

          const effectiveMass = computeDynamicCollisionEffectiveMass(
            body,
            collision.point,
            collision.normal,
            particle.inverseMass
          );
          if (effectiveMass <= 1e-8) {
            continue;
          }

          const correctionImpulse = collision.penetration / effectiveMass;
          particle.predictedPosition = addScaledVec3(
            particle.predictedPosition,
            collision.normal,
            correctionImpulse * particle.inverseMass
          );

          const offset = subtractVec3(collision.point, body.position);
          const bodyCorrection = scaleVec3(collision.normal, -correctionImpulse);
          if (body.inverseMass > 0) {
            body.position = addScaledVec3(body.position, bodyCorrection, body.inverseMass);
          }

          const angularPositionDelta = applyBodyInverseInertiaWorld(body, crossVec3(offset, bodyCorrection));
          if (lengthSquaredVec3(angularPositionDelta) > 1e-12) {
            body.rotation = integrateQuat(body.rotation, angularPositionDelta, 1);
          }

          const relativeVelocity = subtractVec3(particle.velocity, getBodyPointVelocity(body, collision.point));
          const relativeNormalVelocity = dotVec3(relativeVelocity, collision.normal);
          if (relativeNormalVelocity < 0) {
            const impulseMagnitude = -relativeNormalVelocity / effectiveMass;
            const particleImpulse = scaleVec3(collision.normal, impulseMagnitude);
            particle.velocity = addScaledVec3(particle.velocity, particleImpulse, particle.inverseMass);
            body.linearVelocity = addScaledVec3(body.linearVelocity, particleImpulse, -body.inverseMass);
            body.angularVelocity = addVec3(
              body.angularVelocity,
              applyBodyInverseInertiaWorld(body, crossVec3(offset, scaleVec3(particleImpulse, -1)))
            );
          }

          body.sleeping = false;
          body.sleepTimer = 0;
          affectedBodies.add(body.id);
          stepStats.solvedDynamicCollisions += 1;

          if (this.lastCollisionContacts.length < 256) {
            this.lastCollisionContacts.push({
              clothId: cloth.id,
              particleId: particle.id,
              colliderId: collision.colliderId,
              bodyId: collision.bodyId,
              motionType: collision.motionType,
              shapeType: collision.shapeType,
              point: cloneVec3(collision.point),
              normal: cloneVec3(collision.normal),
              particlePosition: cloneVec3(particle.predictedPosition)
            });
          }
        }
      }
    }

    stepStats.affectedDynamicBodyCount = affectedBodies.size;
  }

  solveSelfCollisions(cloths, stepStats) {
    for (const cloth of cloths) {
      if (!cloth.selfCollisionEnabled) {
        continue;
      }

      const targetDistance = Number(cloth.selfCollisionDistance ?? 0);
      if (targetDistance <= 0) {
        continue;
      }

      const targetDistanceSquared = targetDistance * targetDistance;
      for (let indexA = 0; indexA < cloth.particleIds.length - 1; indexA += 1) {
        const particleA = this.particles.get(cloth.particleIds[indexA]);
        if (!particleA) {
          continue;
        }

        for (let indexB = indexA + 1; indexB < cloth.particleIds.length; indexB += 1) {
          const particleB = this.particles.get(cloth.particleIds[indexB]);
          if (!particleB) {
            continue;
          }

          if (particleA.inverseMass <= 0 && particleB.inverseMass <= 0) {
            continue;
          }

          if (shouldSkipSelfCollisionPair(particleA, particleB)) {
            continue;
          }

          const delta = subtractVec3(particleB.predictedPosition, particleA.predictedPosition);
          const distanceSquared = lengthSquaredVec3(delta);
          if (distanceSquared >= targetDistanceSquared) {
            continue;
          }

          const distance = Math.sqrt(Math.max(distanceSquared, 1e-12));
          const normal = distance > 1e-6
            ? scaleVec3(delta, 1 / distance)
            : createVec3(0, 1, 0);
          const penetration = targetDistance - distance;
          const inverseMassSum = Number(particleA.inverseMass ?? 0) + Number(particleB.inverseMass ?? 0);
          if (inverseMassSum <= 1e-8) {
            continue;
          }

          const separationImpulse = penetration / inverseMassSum;
          if (particleA.inverseMass > 0) {
            particleA.predictedPosition = addScaledVec3(
              particleA.predictedPosition,
              normal,
              -separationImpulse * particleA.inverseMass
            );
          }
          if (particleB.inverseMass > 0) {
            particleB.predictedPosition = addScaledVec3(
              particleB.predictedPosition,
              normal,
              separationImpulse * particleB.inverseMass
            );
          }

          const relativeVelocity = subtractVec3(particleB.velocity, particleA.velocity);
          const inwardSpeed = dotVec3(relativeVelocity, normal);
          if (inwardSpeed < 0) {
            const impulseMagnitude = -inwardSpeed / inverseMassSum;
            const impulse = scaleVec3(normal, impulseMagnitude);
            if (particleA.inverseMass > 0) {
              particleA.velocity = addScaledVec3(particleA.velocity, impulse, -particleA.inverseMass);
            }
            if (particleB.inverseMass > 0) {
              particleB.velocity = addScaledVec3(particleB.velocity, impulse, particleB.inverseMass);
            }
          }

          stepStats.solvedSelfCollisions += 1;
          if (this.lastSelfCollisionContacts.length < 256) {
            const point = scaleVec3(addVec3(particleA.predictedPosition, particleB.predictedPosition), 0.5);
            this.lastSelfCollisionContacts.push({
              clothId: cloth.id,
              particleAId: particleA.id,
              particleBId: particleB.id,
              point,
              normal,
              particleAPosition: cloneVec3(particleA.predictedPosition),
              particleBPosition: cloneVec3(particleB.predictedPosition)
            });
          }
        }
      }
    }
  }

  step(deltaSeconds, options = {}) {
    const requestedDeltaSeconds = toNonNegativeNumber(deltaSeconds, 0);
    if (requestedDeltaSeconds <= 0 || this.particles.size === 0 || this.cloths.size === 0) {
      this.lastStepStats = createEmptyStepStats(this.iterations);
      this.lastStepStats.requestedDeltaSeconds = requestedDeltaSeconds;
      this.lastCollisionContacts = [];
      this.lastSelfCollisionContacts = [];
      return {
        ...this.lastStepStats
      };
    }

    const substeps = Math.max(1, this.substeps);
    const substepDelta = requestedDeltaSeconds / substeps;
    const stepStats = createEmptyStepStats(this.iterations);
    stepStats.requestedDeltaSeconds = requestedDeltaSeconds;
    stepStats.substeps = substeps;
    this.lastCollisionContacts = [];
    this.lastSelfCollisionContacts = [];

    const cloths = Array.from(this.cloths.values());
    const { distanceConstraints, pinConstraints } = collectConstraints(this.cloths);
    const staticColliders = Array.isArray(options.staticColliders) ? options.staticColliders : [];
    const dynamicColliders = Array.isArray(options.dynamicColliders) ? options.dynamicColliders : [];

    for (let substepIndex = 0; substepIndex < substeps; substepIndex += 1) {
      for (const cloth of cloths) {
        const dampingFactor = Math.max(0, 1 - cloth.damping * substepDelta);
        for (const particleId of cloth.particleIds) {
          const particle = this.particles.get(particleId);
          if (!particle) {
            continue;
          }

          if (particle.inverseMass <= 0) {
            particle.velocity = createVec3();
            particle.previousPosition = cloneVec3(particle.position);
            particle.predictedPosition = particle.pinTarget ? cloneVec3(particle.pinTarget) : cloneVec3(particle.position);
            continue;
          }

          stepStats.activeParticleCount += 1;
          particle.previousPosition = cloneVec3(particle.position);
          particle.velocity = scaleVec3(addScaledVec3(particle.velocity, this.gravity, substepDelta), dampingFactor);
          particle.predictedPosition = addScaledVec3(particle.position, particle.velocity, substepDelta);
        }
      }

      resetConstraintLambdas(distanceConstraints);
      resetConstraintLambdas(pinConstraints);

      for (let iteration = 0; iteration < this.iterations; iteration += 1) {
        for (const constraint of distanceConstraints) {
          const particleA = this.particles.get(constraint.particleAId);
          const particleB = this.particles.get(constraint.particleBId);
          if (solveDistanceConstraint(constraint, particleA, particleB, substepDelta)) {
            stepStats.solvedDistanceConstraints += 1;
          }
        }

        for (const constraint of pinConstraints) {
          const particle = this.particles.get(constraint.particleId);
          if (solvePinConstraint(constraint, particle, substepDelta)) {
            stepStats.solvedPinConstraints += 1;
          }
        }

        this.solveStaticCollisions(cloths, staticColliders, stepStats);
        this.solveDynamicCollisions(cloths, dynamicColliders, stepStats);
        this.solveSelfCollisions(cloths, stepStats);
      }

      for (const cloth of cloths) {
        for (const particleId of cloth.particleIds) {
          const particle = this.particles.get(particleId);
          if (!particle) {
            continue;
          }

          if (particle.inverseMass <= 0) {
            particle.position = particle.pinTarget ? cloneVec3(particle.pinTarget) : cloneVec3(particle.position);
            particle.predictedPosition = cloneVec3(particle.position);
            particle.velocity = createVec3();
            continue;
          }

          particle.velocity = scaleVec3(addScaledVec3(particle.predictedPosition, particle.position, -1), 1 / substepDelta);
          particle.position = cloneVec3(particle.predictedPosition);
        }
      }
    }

    this.simulationTick += substeps;
    this.lastStepStats = stepStats;
    return {
      ...this.lastStepStats
    };
  }

  buildDebugPrimitives() {
    const primitives = [];

    for (const cloth of this.cloths.values()) {
      for (const edge of cloth.structuralEdges) {
        const particleA = this.particles.get(edge.particleAId);
        const particleB = this.particles.get(edge.particleBId);
        if (!particleA || !particleB) {
          continue;
        }

        primitives.push(
          createDebugLine({
            id: edge.id,
            category: 'cloth-structural-edge',
            start: particleA.position,
            end: particleB.position,
            color: DEFAULT_DEBUG_COLORS.clothStructuralEdge,
            source: {
              clothId: cloth.id,
              particleAId: edge.particleAId,
              particleBId: edge.particleBId,
              edgeKind: 'structural'
            }
          })
        );
      }

      for (const edge of cloth.shearEdges) {
        const particleA = this.particles.get(edge.particleAId);
        const particleB = this.particles.get(edge.particleBId);
        if (!particleA || !particleB) {
          continue;
        }

        primitives.push(
          createDebugLine({
            id: edge.id,
            category: 'cloth-shear-edge',
            start: particleA.position,
            end: particleB.position,
            color: DEFAULT_DEBUG_COLORS.clothShearEdge,
            source: {
              clothId: cloth.id,
              particleAId: edge.particleAId,
              particleBId: edge.particleBId,
              edgeKind: 'shear'
            }
          })
        );
      }

      for (const edge of cloth.bendEdges) {
        const particleA = this.particles.get(edge.particleAId);
        const particleB = this.particles.get(edge.particleBId);
        if (!particleA || !particleB) {
          continue;
        }

        primitives.push(
          createDebugLine({
            id: edge.id,
            category: 'cloth-bend-edge',
            start: particleA.position,
            end: particleB.position,
            color: DEFAULT_DEBUG_COLORS.clothBendEdge,
            source: {
              clothId: cloth.id,
              particleAId: edge.particleAId,
              particleBId: edge.particleBId,
              edgeKind: 'bend'
            }
          })
        );
      }

      for (const particleId of cloth.particleIds) {
        const particle = this.particles.get(particleId);
        if (!particle) {
          continue;
        }

        primitives.push(
          createDebugPoint({
            id: `${particle.id}:point`,
            category: particle.pinned ? 'cloth-pin' : 'cloth-particle',
            position: particle.position,
            color: particle.pinned ? DEFAULT_DEBUG_COLORS.clothPinnedParticle : DEFAULT_DEBUG_COLORS.clothParticle,
            size: particle.pinned ? 4.5 : 3.25,
            source: {
              clothId: cloth.id,
              particleId: particle.id,
              pinned: particle.pinned
            }
          })
        );
      }

      for (const contact of this.lastCollisionContacts) {
        if (contact.clothId !== cloth.id) {
          continue;
        }

        primitives.push(
          createDebugPoint({
            id: `${contact.particleId}:cloth-contact-point`,
            category: 'cloth-contact-point',
            position: contact.point,
            color: DEFAULT_DEBUG_COLORS.clothContactPoint,
            size: 4.25,
            source: {
              clothId: contact.clothId,
              particleId: contact.particleId,
              colliderId: contact.colliderId,
              bodyId: contact.bodyId,
              motionType: contact.motionType,
              shapeType: contact.shapeType
            }
          })
        );

        primitives.push(
          createDebugLine({
            id: `${contact.particleId}:cloth-contact-normal`,
            category: 'cloth-contact-normal',
            start: contact.point,
            end: addScaledVec3(contact.point, contact.normal, 12),
            color: DEFAULT_DEBUG_COLORS.clothContactNormal,
            source: {
              clothId: contact.clothId,
              particleId: contact.particleId,
              colliderId: contact.colliderId,
              bodyId: contact.bodyId,
              motionType: contact.motionType,
              shapeType: contact.shapeType
            }
          })
        );
      }

      for (const contact of this.lastSelfCollisionContacts) {
        if (contact.clothId !== cloth.id) {
          continue;
        }

        primitives.push(
          createDebugPoint({
            id: `${contact.particleAId}|${contact.particleBId}:cloth-self-contact-point`,
            category: 'cloth-self-contact-point',
            position: contact.point,
            color: DEFAULT_DEBUG_COLORS.clothSelfContactPoint,
            size: 3.75,
            source: {
              clothId: contact.clothId,
              particleAId: contact.particleAId,
              particleBId: contact.particleBId
            }
          })
        );

        primitives.push(
          createDebugLine({
            id: `${contact.particleAId}|${contact.particleBId}:cloth-self-contact-normal`,
            category: 'cloth-self-contact-normal',
            start: contact.point,
            end: addScaledVec3(contact.point, contact.normal, 10),
            color: DEFAULT_DEBUG_COLORS.clothSelfContactNormal,
            source: {
              clothId: contact.clothId,
              particleAId: contact.particleAId,
              particleBId: contact.particleBId
            }
          })
        );
      }
    }

    return primitives;
  }

  getSnapshot() {
    return {
      clothCount: this.countCloths(),
      particleCount: this.countParticles(),
      gravity: cloneVec3(this.gravity),
      simulationTick: this.simulationTick,
      cloths: this.listCloths(),
      particles: this.listParticles(),
      lastCollisionContacts: this.lastCollisionContacts.map((contact) => cloneClothCollisionContact(contact)),
      lastSelfCollisionContacts: this.lastSelfCollisionContacts.map((contact) => cloneClothSelfCollisionContact(contact)),
      lastStepStats: {
        ...this.lastStepStats
      }
    };
  }
}
