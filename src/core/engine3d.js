import { buildCubeModelMatrix } from './math/mat4.js';
import { createVec3 } from './math/vec3.js';
import { Scene3D } from './scene.js';

function formatVector(vector) {
  return `${vector.x}, ${vector.y}, ${vector.z}`;
}

function summarizeSnapshot(snapshot) {
  return `${snapshot.objectCount} objects | camera ${formatVector(snapshot.camera.position)} | frames ${snapshot.frameCount}`;
}

export class Engine3D {
  constructor(hostBridge) {
    this.hostBridge = hostBridge;
    this.scene = new Scene3D();
    this.lastFrame = null;
  }

  resetScene() {
    this.scene.reset();
    this.lastFrame = null;
    this.hostBridge.log('Scene reset');
  }

  setCameraPosition(x, y, z) {
    this.scene.setCameraPosition(createVec3(x, y, z));
    this.hostBridge.log(`Camera moved to ${x}, ${y}, ${z}`);
  }

  addCube(id, x, y, z, size) {
    const cube = this.scene.addCube({
      id,
      position: createVec3(x, y, z),
      size
    });

    this.hostBridge.log(`Cube ${cube.id} registered`);
    return cube;
  }

  renderDebugFrame() {
    const frameNumber = this.scene.nextFrame();
    const snapshot = this.scene.snapshot();
    const plannedDrawCalls = snapshot.objects.map((object) => ({
      id: object.id,
      kind: object.kind,
      modelMatrix: buildCubeModelMatrix(object.position, object.size)
    }));

    this.lastFrame = {
      frameNumber,
      plannedDrawCalls,
      snapshot,
      summary: `${this.hostBridge.getDisplayName()} frame ${frameNumber} | ${plannedDrawCalls.length} draw calls`
    };

    this.hostBridge.emitFrame(this.lastFrame);
    return this.lastFrame;
  }

  getSceneSummary() {
    return summarizeSnapshot(this.scene.snapshot());
  }

  getLastFrameSummary() {
    if (!this.lastFrame) {
      return 'No frame rendered yet';
    }

    return this.lastFrame.summary;
  }

  getHostSummary() {
    const capabilities = this.hostBridge.getCapabilities();
    return `${this.hostBridge.getDisplayName()} | runtime:${capabilities.runtime ? 'yes' : 'no'} | renderer:${capabilities.renderer ? 'yes' : 'no'} | sandbox:${capabilities.sandbox ? 'yes' : 'no'}`;
  }
}

