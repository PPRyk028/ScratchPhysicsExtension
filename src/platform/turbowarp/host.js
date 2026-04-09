export function createTurboWarpHost(scratchApi) {
  const runtime = scratchApi?.vm?.runtime ?? null;
  const renderer = runtime?.renderer ?? scratchApi?.vm?.renderer ?? null;

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
      console.info('[TurboWarp 3D]', frame.summary, frame);
    },
    log(message) {
      console.info('[TurboWarp 3D]', message);
    }
  };
}

