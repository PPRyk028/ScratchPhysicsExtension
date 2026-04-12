const CLOTH_PRESET_LIBRARY = Object.freeze({
  'light-fabric': Object.freeze({
    id: 'light-fabric',
    label: 'light fabric',
    damping: 0.06,
    collisionMargin: 3,
    stretchCompliance: 0,
    shearCompliance: 0.0004,
    bendCompliance: 0.008,
    selfCollisionEnabled: true,
    selfCollisionDistance: 8
  }),
  'heavy-cloth': Object.freeze({
    id: 'heavy-cloth',
    label: 'heavy cloth',
    damping: 0.1,
    collisionMargin: 4,
    stretchCompliance: 0,
    shearCompliance: 0.00025,
    bendCompliance: 0.003,
    selfCollisionEnabled: true,
    selfCollisionDistance: 12
  }),
  wrinkle: Object.freeze({
    id: 'wrinkle',
    label: 'wrinkle debug',
    damping: 0.08,
    collisionMargin: 2.5,
    stretchCompliance: 0,
    shearCompliance: 0.0006,
    bendCompliance: 0.015,
    selfCollisionEnabled: true,
    selfCollisionDistance: 7
  })
});

const DEFAULT_CLOTH_PRESET_ID = 'light-fabric';

function clonePreset(preset) {
  return {
    ...preset
  };
}

function normalizeClothPresetId(value) {
  const presetId = String(value ?? '').trim().toLowerCase();
  return CLOTH_PRESET_LIBRARY[presetId] ? presetId : DEFAULT_CLOTH_PRESET_ID;
}

export function getClothPresetIds() {
  return Object.keys(CLOTH_PRESET_LIBRARY);
}

export function getDefaultClothPresetId() {
  return DEFAULT_CLOTH_PRESET_ID;
}

export function resolveClothPreset(presetId) {
  return clonePreset(CLOTH_PRESET_LIBRARY[normalizeClothPresetId(presetId)]);
}
