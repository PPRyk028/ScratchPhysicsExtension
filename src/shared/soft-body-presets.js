const SOFT_BODY_PRESET_LIBRARY = Object.freeze({
  jelly: Object.freeze({
    id: 'jelly',
    label: 'jelly',
    damping: 0.02,
    collisionMargin: 2.5,
    stretchCompliance: 0.008,
    shearCompliance: 0.015,
    bendCompliance: 0.035,
    volumeCompliance: 0.006
  }),
  foam: Object.freeze({
    id: 'foam',
    label: 'foam',
    damping: 0.035,
    collisionMargin: 2.5,
    stretchCompliance: 0.002,
    shearCompliance: 0.004,
    bendCompliance: 0.01,
    volumeCompliance: 0.001
  }),
  'firm-rubber': Object.freeze({
    id: 'firm-rubber',
    label: 'firm rubber',
    damping: 0.045,
    collisionMargin: 3,
    stretchCompliance: 0.0006,
    shearCompliance: 0.0012,
    bendCompliance: 0.003,
    volumeCompliance: 0.0003
  })
});

const DEFAULT_SOFT_BODY_PRESET_ID = 'foam';

function clonePreset(preset) {
  return {
    ...preset
  };
}

function normalizeSoftBodyPresetId(value) {
  const presetId = String(value ?? '').trim().toLowerCase();
  return SOFT_BODY_PRESET_LIBRARY[presetId] ? presetId : DEFAULT_SOFT_BODY_PRESET_ID;
}

export function getSoftBodyPresetIds() {
  return Object.keys(SOFT_BODY_PRESET_LIBRARY);
}

export function getDefaultSoftBodyPresetId() {
  return DEFAULT_SOFT_BODY_PRESET_ID;
}

export function resolveSoftBodyPreset(presetId) {
  return clonePreset(SOFT_BODY_PRESET_LIBRARY[normalizeSoftBodyPresetId(presetId)]);
}
