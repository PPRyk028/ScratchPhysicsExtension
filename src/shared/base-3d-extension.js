import { Engine3D } from '../core/engine3d.js';
import { toNumber, toString } from './coerce.js';
import { createExtensionInfo } from './extension-metadata.js';

export class Base3DExtension {
  constructor(hostBridge) {
    this.hostBridge = hostBridge;
    this.engine = new Engine3D(hostBridge);
  }

  getInfo() {
    return createExtensionInfo();
  }

  resetScene() {
    this.engine.resetScene();
  }

  setCameraPosition(args) {
    this.engine.setCameraPosition(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z, 400)
    );
  }

  addCube(args) {
    this.engine.addCube(
      toString(args.ID, 'cube'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.SIZE, 100)
    );
  }

  renderDebugFrame() {
    this.engine.renderDebugFrame();
  }

  sceneSummary() {
    return this.engine.getSceneSummary();
  }

  lastFrameSummary() {
    return this.engine.getLastFrameSummary();
  }

  hostSummary() {
    return this.engine.getHostSummary();
  }
}

