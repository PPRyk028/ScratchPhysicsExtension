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

  resetWorld() {
    this.engine.resetWorld();
  }

  setCameraPosition(args) {
    this.engine.setCameraPosition(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z, 400)
    );
  }

  setGravity(args) {
    this.engine.setGravity(
      toNumber(args.X),
      toNumber(args.Y, -9.81),
      toNumber(args.Z)
    );
  }

  createMaterial(args) {
    this.engine.createMaterial(
      toString(args.ID, 'material-1'),
      toNumber(args.FRICTION, 0.5),
      toNumber(args.RESTITUTION, 0),
      toNumber(args.DENSITY, 1)
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

  createBoxRigidBody(args) {
    this.engine.createBoxRigidBody(
      toString(args.ID, 'body'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.SIZE, 100),
      toNumber(args.MASS, 1),
      toString(args.MATERIAL, '')
    );
  }

  createStaticBoxCollider(args) {
    this.engine.createStaticBoxCollider(
      toString(args.ID, 'collider'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.SIZE, 100),
      toString(args.MATERIAL, '')
    );
  }

  stepWorld(args) {
    this.engine.stepWorld(
      toNumber(args.SECONDS, 1 / 60)
    );
  }

  renderDebugFrame() {
    this.engine.renderDebugFrame();
  }

  sceneSummary() {
    return this.engine.getSceneSummary();
  }

  worldSummary() {
    return this.engine.getWorldSummary();
  }

  rigidBodySummary(args) {
    return this.engine.getRigidBodySummary(
      toString(args.ID, 'body')
    );
  }

  colliderSummary(args) {
    return this.engine.getColliderSummary(
      toString(args.ID, 'collider')
    );
  }

  materialSummary(args) {
    return this.engine.getMaterialSummary(
      toString(args.ID, 'material-1')
    );
  }

  queryPointBodies(args) {
    return this.engine.queryPointBodies(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z)
    );
  }

  queryPointColliders(args) {
    return this.engine.queryPointColliders(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z)
    );
  }

  queryAabbBodies(args) {
    return this.engine.queryAabbBodies(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.HX, 0.5),
      toNumber(args.HY, 0.5),
      toNumber(args.HZ, 0.5)
    );
  }

  queryAabbColliders(args) {
    return this.engine.queryAabbColliders(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.HX, 0.5),
      toNumber(args.HY, 0.5),
      toNumber(args.HZ, 0.5)
    );
  }

  debugFrameSummary() {
    return this.engine.getLastFrameSummary();
  }

  lastFrameSummary() {
    return this.engine.getLastFrameSummary();
  }

  hostSummary() {
    return this.engine.getHostSummary();
  }
}
