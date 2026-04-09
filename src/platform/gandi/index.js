import { Base3DExtension } from '../../shared/base-3d-extension.js';
import { GANDI_EXTENSION_METADATA, GANDI_L10N } from '../../shared/extension-metadata.js';
import { createGandiApprovedHost } from './host.js';

class Gandi3DExtension extends Base3DExtension {
  constructor(runtime) {
    super(createGandiApprovedHost(runtime));
    this.runtime = runtime;
  }
}

const root = typeof window !== 'undefined' ? window : globalThis;

root.tempExt = {
  Extension: Gandi3DExtension,
  info: GANDI_EXTENSION_METADATA,
  l10n: GANDI_L10N
};
