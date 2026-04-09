import { Base3DExtension } from '../../shared/base-3d-extension.js';
import { createTurboWarpHost } from './host.js';

const ScratchApi = globalThis.Scratch;

if (!ScratchApi) {
  throw new Error('TurboWarp Scratch API not found.');
}

if (!ScratchApi.extensions?.unsandboxed) {
  throw new Error('This build must be loaded as an unsandboxed TurboWarp extension.');
}

class TurboWarp3DExtension extends Base3DExtension {
  constructor() {
    super(createTurboWarpHost(ScratchApi));
  }
}

ScratchApi.extensions.register(new TurboWarp3DExtension());

