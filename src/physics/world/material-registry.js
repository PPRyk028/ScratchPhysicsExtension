import { BaseRegistry } from './base-registry.js';

function toNonNegativeNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
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
      userData: material.userData ?? null
    };
  }

  createMaterial(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      friction: toNonNegativeNumber(options.friction, 0.5),
      restitution: toNonNegativeNumber(options.restitution, 0),
      density: toNonNegativeNumber(options.density, 1),
      userData: options.userData ?? null
    });
  }
}
