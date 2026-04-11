import { createDebugOverlay } from '../shared/debug-overlay.js';

export function createTurboWarpHost(scratchApi) {
  const runtime = scratchApi?.vm?.runtime ?? null;
  const renderer = runtime?.renderer ?? scratchApi?.vm?.renderer ?? null;
  const overlay = createDebugOverlay('TurboWarp');

  return {
    id: 'turbowarp',
    getDisplayName() {
      return 'TurboWarp';
    },
    getCapabilities() {
      return {
        runtime: Boolean(runtime),
        renderer: Boolean(renderer),
        sandbox: false
      };
    },
    emitFrame(frame) {
      overlay.render(frame);
      console.info('[TurboWarp 3D]', frame.summary, frame);
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
      console.info('[TurboWarp 3D]', message);
    }
  };
}
