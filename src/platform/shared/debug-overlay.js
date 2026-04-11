import { rotateVec3ByQuat } from '../../physics/math/quat.js';
import { addVec3, createVec3, crossVec3, dotVec3, normalizeVec3, subtractVec3 } from '../../physics/math/vec3.js';

function rgba(color) {
  const r = Math.max(0, Math.min(255, Math.round(Number(color?.r ?? 255))));
  const g = Math.max(0, Math.min(255, Math.round(Number(color?.g ?? 255))));
  const b = Math.max(0, Math.min(255, Math.round(Number(color?.b ?? 255))));
  const a = Math.max(0, Math.min(1, Number(color?.a ?? 1)));
  return `rgba(${r}, ${g}, ${b}, ${a})`;
}

function getDomGlobals() {
  const root = typeof window !== 'undefined' ? window : globalThis;
  return {
    window: root,
    document: root?.document ?? null
  };
}

function ensureCanvasSize(canvas) {
  const devicePixelRatio = Number(getDomGlobals().window?.devicePixelRatio ?? 1) || 1;
  const bounds = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(bounds.width * devicePixelRatio));
  const height = Math.max(1, Math.round(bounds.height * devicePixelRatio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

function buildViewBasis(cameraPosition, cameraTarget) {
  const target = cameraTarget ?? createVec3(0, 0, 0);
  const forward = normalizeVec3(subtractVec3(target, cameraPosition), createVec3(0, 0, -1));
  const worldUp = Math.abs(forward.y) > 0.98 ? createVec3(0, 0, 1) : createVec3(0, 1, 0);
  const right = normalizeVec3(crossVec3(forward, worldUp), createVec3(1, 0, 0));
  const up = normalizeVec3(crossVec3(right, forward), createVec3(0, 1, 0));
  return { forward, right, up };
}

function projectPoint(point, cameraPosition, basis, width, height) {
  const relative = subtractVec3(point, cameraPosition);
  const viewX = dotVec3(relative, basis.right);
  const viewY = dotVec3(relative, basis.up);
  const viewZ = dotVec3(relative, basis.forward);
  if (viewZ <= 1) {
    return null;
  }

  const focalLength = height / (2 * Math.tan((50 * Math.PI) / 360));
  return {
    x: width / 2 + (viewX * focalLength) / viewZ,
    y: height / 2 - (viewY * focalLength) / viewZ,
    depth: viewZ
  };
}

function buildWireBoxSegments(primitive) {
  const localCorners = [
    createVec3(-1, -1, -1),
    createVec3(1, -1, -1),
    createVec3(1, 1, -1),
    createVec3(-1, 1, -1),
    createVec3(-1, -1, 1),
    createVec3(1, -1, 1),
    createVec3(1, 1, 1),
    createVec3(-1, 1, 1)
  ].map((corner) => createVec3(
    corner.x * primitive.halfExtents.x,
    corner.y * primitive.halfExtents.y,
    corner.z * primitive.halfExtents.z
  ));

  const worldCorners = localCorners.map((corner) => addVec3(primitive.center, rotateVec3ByQuat(primitive.rotation, corner)));
  const edgeIndices = [
    [0, 1], [1, 2], [2, 3], [3, 0],
    [4, 5], [5, 6], [6, 7], [7, 4],
    [0, 4], [1, 5], [2, 6], [3, 7]
  ];

  return edgeIndices.map(([startIndex, endIndex]) => ({
    start: worldCorners[startIndex],
    end: worldCorners[endIndex]
  }));
}

function renderLinePrimitive(context2d, start, end, color, lineWidth = 1.5) {
  context2d.strokeStyle = color;
  context2d.lineWidth = lineWidth;
  context2d.beginPath();
  context2d.moveTo(start.x, start.y);
  context2d.lineTo(end.x, end.y);
  context2d.stroke();
}

function renderPointPrimitive(context2d, point, color, size) {
  context2d.fillStyle = color;
  context2d.beginPath();
  context2d.arc(point.x, point.y, Math.max(1.5, size), 0, Math.PI * 2);
  context2d.fill();
}

export function createDebugOverlay(displayName) {
  const state = {
    available: false,
    visible: false,
    lastSummary: 'overlay unavailable',
    container: null,
    canvas: null,
    label: null
  };

  function ensureDom() {
    if (state.available) {
      return true;
    }

    const { document } = getDomGlobals();
    if (!document?.body) {
      state.lastSummary = `${displayName} overlay unavailable`;
      return false;
    }

    const container = document.createElement('div');
    container.style.position = 'fixed';
    container.style.right = '16px';
    container.style.bottom = '16px';
    container.style.width = 'min(38vw, 520px)';
    container.style.aspectRatio = '4 / 3';
    container.style.minWidth = '320px';
    container.style.maxWidth = '520px';
    container.style.display = 'none';
    container.style.zIndex = '2147483647';
    container.style.pointerEvents = 'none';
    container.style.border = '1px solid rgba(255,255,255,0.18)';
    container.style.borderRadius = '12px';
    container.style.background = 'rgba(10, 16, 24, 0.78)';
    container.style.boxShadow = '0 12px 30px rgba(0,0,0,0.28)';
    container.style.backdropFilter = 'blur(4px)';

    const label = document.createElement('div');
    label.style.position = 'absolute';
    label.style.left = '10px';
    label.style.top = '8px';
    label.style.color = '#d9e8ff';
    label.style.font = '12px/1.2 Consolas, "Courier New", monospace';
    label.style.letterSpacing = '0.04em';
    label.textContent = `${displayName} debug overlay`;

    const canvas = document.createElement('canvas');
    canvas.style.width = '100%';
    canvas.style.height = '100%';
    canvas.style.display = 'block';

    container.appendChild(canvas);
    container.appendChild(label);
    document.body.appendChild(container);

    state.available = true;
    state.container = container;
    state.canvas = canvas;
    state.label = label;
    state.lastSummary = `${displayName} overlay hidden`;
    return true;
  }

  function show() {
    if (!ensureDom()) {
      return false;
    }

    state.visible = true;
    state.container.style.display = 'block';
    state.lastSummary = `${displayName} overlay visible`;
    return true;
  }

  function hide() {
    if (!ensureDom()) {
      return false;
    }

    state.visible = false;
    state.container.style.display = 'none';
    state.lastSummary = `${displayName} overlay hidden`;
    return true;
  }

  function clearCanvas(context2d, width, height) {
    context2d.clearRect(0, 0, width, height);
    context2d.fillStyle = 'rgba(8, 12, 18, 0.92)';
    context2d.fillRect(0, 0, width, height);
    context2d.strokeStyle = 'rgba(255,255,255,0.08)';
    context2d.strokeRect(0.5, 0.5, width - 1, height - 1);
  }

  function render(frame) {
    if (!state.visible || !ensureDom()) {
      return false;
    }

    ensureCanvasSize(state.canvas);
    const context2d = state.canvas.getContext('2d');
    const width = state.canvas.width;
    const height = state.canvas.height;
    clearCanvas(context2d, width, height);

    const cameraPosition = frame?.debugFrame?.camera?.position ?? createVec3(0, 0, 400);
    const cameraTarget = frame?.debugFrame?.camera?.target ?? createVec3(0, 0, 0);
    const basis = buildViewBasis(cameraPosition, cameraTarget);
    const primitives = Array.isArray(frame?.debugFrame?.primitives) ? frame.debugFrame.primitives : [];

    for (const primitive of primitives) {
      const color = rgba(primitive.color);

      if (primitive.type === 'line') {
        const start = projectPoint(primitive.start, cameraPosition, basis, width, height);
        const end = projectPoint(primitive.end, cameraPosition, basis, width, height);
        if (start && end) {
          renderLinePrimitive(context2d, start, end, color, 1.75);
        }
        continue;
      }

      if (primitive.type === 'point') {
        const point = projectPoint(primitive.position, cameraPosition, basis, width, height);
        if (point) {
          renderPointPrimitive(context2d, point, color, Number(primitive.size ?? 4));
        }
        continue;
      }

      if (primitive.type === 'wireBox') {
        for (const segment of buildWireBoxSegments(primitive)) {
          const start = projectPoint(segment.start, cameraPosition, basis, width, height);
          const end = projectPoint(segment.end, cameraPosition, basis, width, height);
          if (start && end) {
            renderLinePrimitive(context2d, start, end, color, 1.2);
          }
        }
      }
    }

    state.label.textContent = `${displayName} debug overlay | ${primitives.length} primitives | frame ${frame?.frameNumber ?? 0}`;
    state.lastSummary = `${displayName} overlay visible | ${primitives.length} primitives | frame ${frame?.frameNumber ?? 0}`;
    return true;
  }

  return {
    show,
    hide,
    render,
    isAvailable() {
      return ensureDom();
    },
    isVisible() {
      return state.visible;
    },
    getSummary() {
      return state.lastSummary;
    }
  };
}
