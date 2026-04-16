const SURFACE_PRESETS = Object.freeze({
  ice: Object.freeze({
    id: 'ice',
    friction: 0.05,
    restitution: 0,
    surfaceTraction: 0.18,
    surfaceJumpMultiplier: 1,
    surfaceConveyorVelocity: Object.freeze({ x: 0, y: 0, z: 0 })
  }),
  sticky: Object.freeze({
    id: 'sticky',
    friction: 1.2,
    restitution: 0,
    surfaceTraction: 1.35,
    surfaceJumpMultiplier: 0.9,
    surfaceConveyorVelocity: Object.freeze({ x: 0, y: 0, z: 0 })
  }),
  'bounce-pad': Object.freeze({
    id: 'bounce-pad',
    friction: 0.6,
    restitution: 0.1,
    surfaceTraction: 1,
    surfaceJumpMultiplier: 1.85,
    surfaceConveyorVelocity: Object.freeze({ x: 0, y: 0, z: 0 })
  }),
  conveyor: Object.freeze({
    id: 'conveyor',
    friction: 0.45,
    restitution: 0,
    surfaceTraction: 1,
    surfaceJumpMultiplier: 1,
    surfaceConveyorVelocity: Object.freeze({ x: 0, y: 0, z: 0 })
  })
});

function clonePreset(preset) {
  return {
    ...preset,
    surfaceConveyorVelocity: { ...preset.surfaceConveyorVelocity }
  };
}

export function resolveSurfacePreset(presetId) {
  const resolvedId = String(presetId ?? '').trim().toLowerCase();
  return clonePreset(SURFACE_PRESETS[resolvedId] ?? SURFACE_PRESETS.ice);
}

export function getSurfacePresetIds() {
  return Object.keys(SURFACE_PRESETS);
}
