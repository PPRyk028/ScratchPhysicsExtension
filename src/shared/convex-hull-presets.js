const PRESET_LIBRARY = {
  pyramid: [
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [0.5, -0.5, 0.5],
    [-0.5, -0.5, 0.5],
    [0, 0.55, 0]
  ],
  'skew-prism': [
    [-0.5, -0.35, -0.4],
    [0.55, -0.25, -0.15],
    [-0.1, -0.2, 0.6],
    [-0.3, 0.55, -0.3],
    [0.75, 0.65, -0.05],
    [0.1, 0.7, 0.7]
  ],
  'skew-hexahedron': [
    [-0.5, -0.35, -0.3],
    [0.6, -0.25, -0.15],
    [-0.3, 0.5, -0.25],
    [0.8, 0.6, -0.1],
    [-0.45, -0.2, 0.65],
    [0.65, -0.1, 0.8],
    [-0.25, 0.65, 0.7],
    [0.85, 0.75, 0.85]
  ],
  'skew-frustum': [
    [-0.6, -0.4, -0.5],
    [0.5, -0.35, -0.45],
    [0.6, -0.3, 0.55],
    [-0.55, -0.45, 0.6],
    [-0.2, 0.45, -0.15],
    [0.3, 0.5, -0.1],
    [0.35, 0.48, 0.3],
    [-0.25, 0.42, 0.35]
  ],
  wedge: [
    [-0.55, -0.45, -0.5],
    [0.55, -0.45, -0.5],
    [0.55, -0.45, 0.5],
    [-0.55, -0.45, 0.5],
    [-0.55, 0.45, -0.5],
    [-0.55, 0.45, 0.5]
  ],
  octahedron: [
    [0, 0.7, 0],
    [0.6, 0, 0],
    [0, 0, 0.6],
    [-0.6, 0, 0],
    [0, 0, -0.6],
    [0, -0.7, 0]
  ]
};

const DEFAULT_PRESET_ID = 'pyramid';

function toPositiveNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
}

function normalizePresetId(value) {
  const presetId = String(value ?? '').trim().toLowerCase();
  return PRESET_LIBRARY[presetId] ? presetId : DEFAULT_PRESET_ID;
}

function formatCoordinate(value) {
  const rounded = Math.round(Number(value) * 1000) / 1000;
  return Number.isInteger(rounded) ? `${rounded}` : `${rounded}`;
}

export function getConvexHullPresetIds() {
  return Object.keys(PRESET_LIBRARY);
}

export function getDefaultConvexHullPresetId() {
  return DEFAULT_PRESET_ID;
}

export function resolveConvexHullPresetVertices(presetId, scale = 100) {
  const resolvedPresetId = normalizePresetId(presetId);
  const resolvedScale = toPositiveNumber(scale, 100);
  return PRESET_LIBRARY[resolvedPresetId].map(([x, y, z]) => ({
    x: x * resolvedScale,
    y: y * resolvedScale,
    z: z * resolvedScale
  }));
}

export function formatConvexHullVertices(vertices) {
  return (Array.isArray(vertices) ? vertices : [])
    .map((vertex) => `${formatCoordinate(vertex.x)} ${formatCoordinate(vertex.y)} ${formatCoordinate(vertex.z)}`)
    .join('; ');
}

export function getConvexHullPresetVertexText(presetId, scale = 100) {
  return formatConvexHullVertices(resolveConvexHullPresetVertices(presetId, scale));
}
