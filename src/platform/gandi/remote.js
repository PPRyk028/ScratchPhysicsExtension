import { Base3DExtension } from '../../shared/base-3d-extension.js';
import { createGandiRemoteHost } from './host.js';

const ScratchApi = globalThis.Scratch;

if (!ScratchApi) {
  throw new Error('Gandi Scratch API not found.');
}

class GandiRemote3DExtension extends Base3DExtension {
  constructor() {
    super(createGandiRemoteHost());
  }
}

ScratchApi.extensions.register(new GandiRemote3DExtension());
