import { createDebugOverlay } from '../shared/debug-overlay.js';

function createHost(displayName, runtime, renderer, sandbox) {
  const overlay = createDebugOverlay(displayName);

  return {
    id: displayName.toLowerCase().replace(/\s+/g, '-'),
    getDisplayName() {
      return displayName;
    },
    getCapabilities() {
      return {
        runtime: Boolean(runtime),
        renderer: Boolean(renderer),
        sandbox
      };
    },
    emitFrame(frame) {
      overlay.render(frame);
      console.info(`[${displayName} 3D]`, frame.summary, frame);
    },
    showDebugOverlay() {
      overlay.show();
    },
    hideDebugOverlay() {
      overlay.hide();
    },
    getDebugOverlaySummary() {
      return overlay.getSummary();
    },
    log(message) {
      console.info(`[${displayName} 3D]`, message);
    }
  };
}

export function createGandiApprovedHost(runtime) {
  const renderer = runtime?.renderer ?? null;
  return createHost('Gandi Approved', runtime, renderer, false);
}

export function createGandiRemoteHost() {
  return createHost('Gandi Remote', null, null, true);
}
