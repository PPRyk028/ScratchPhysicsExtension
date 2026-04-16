import { cloneVec3, createVec3 } from '../math/vec3.js';
import { BaseRegistry } from './base-registry.js';

function toNonNegativeNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function toFiniteNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

export class MaterialRegistry extends BaseRegistry {
  constructor() {
    super('material');
  }

  cloneRecord(material) {
    return {
      id: material.id,
      friction: material.friction,
      restitution: material.restitution,
      density: material.density,
      surfacePresetId: material.surfacePresetId ?? 'custom',
      surfaceTraction: material.surfaceTraction,
      surfaceJumpMultiplier: material.surfaceJumpMultiplier,
      surfaceConveyorVelocity: cloneVec3(material.surfaceConveyorVelocity ?? createVec3()),
      userData: material.userData ?? null
    };
  }

  createMaterial(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      friction: toNonNegativeNumber(options.friction, 0.5),
      restitution: toNonNegativeNumber(options.restitution, 0),
      density: toNonNegativeNumber(options.density, 1),
      surfacePresetId: String(options.surfacePresetId ?? 'custom').trim() || 'custom',
      surfaceTraction: toNonNegativeNumber(options.surfaceTraction, 1),
      surfaceJumpMultiplier: toNonNegativeNumber(options.surfaceJumpMultiplier, 1),
      surfaceConveyorVelocity: createVec3(
        toFiniteNumber(options.surfaceConveyorVelocity?.x, 0),
        toFiniteNumber(options.surfaceConveyorVelocity?.y, 0),
        toFiniteNumber(options.surfaceConveyorVelocity?.z, 0)
      ),
      userData: options.userData ?? null
    });
  }
}
