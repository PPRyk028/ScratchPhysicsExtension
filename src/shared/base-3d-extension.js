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

  setCameraTarget(args) {
    this.engine.setCameraTarget(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z)
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

  createConvexHullRigidBody(args) {
    this.engine.createConvexHullRigidBody(
      toString(args.ID, 'body'),
      toString(args.VERTICES, ''),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.MASS, 1),
      toString(args.MATERIAL, '')
    );
  }

  createStaticConvexHullCollider(args) {
    this.engine.createStaticConvexHullCollider(
      toString(args.ID, 'collider'),
      toString(args.VERTICES, ''),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toString(args.MATERIAL, '')
    );
  }

  createDistanceJoint(args) {
    this.engine.createDistanceJoint(
      toString(args.ID, 'joint'),
      toString(args.BODY_A, 'body-a'),
      toString(args.BODY_B, 'body-b'),
      toNumber(args.LENGTH, 0)
    );
  }

  createPointToPointJoint(args) {
    this.engine.createPointToPointJoint(
      toString(args.ID, 'joint'),
      toString(args.BODY_A, 'body-a'),
      toString(args.BODY_B, 'body-b'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z)
    );
  }

  createHingeJoint(args) {
    this.engine.createHingeJoint(
      toString(args.ID, 'joint'),
      toString(args.BODY_A, 'body-a'),
      toString(args.BODY_B, 'body-b'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.AX, 0),
      toNumber(args.AY, 1),
      toNumber(args.AZ, 0)
    );
  }

  createFixedJoint(args) {
    this.engine.createFixedJoint(
      toString(args.ID, 'joint'),
      toString(args.BODY_A, 'body-a'),
      toString(args.BODY_B, 'body-b'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z)
    );
  }

  configureDistanceJoint(args) {
    this.engine.configureDistanceJoint(
      toString(args.ID, 'joint'),
      toNumber(args.MIN, 0),
      toNumber(args.MAX, 100),
      toNumber(args.SPRING, 0),
      toNumber(args.DAMPING, 0)
    );
  }

  configureHingeJoint(args) {
    this.engine.configureHingeJoint(
      toString(args.ID, 'joint'),
      toNumber(args.LOWER, -45),
      toNumber(args.UPPER, 45),
      toNumber(args.DAMPING, 0)
    );
  }

  configureFixedJoint(args) {
    this.engine.configureFixedJoint(
      toString(args.ID, 'joint'),
      toNumber(args.BREAK_FORCE, 0),
      toNumber(args.BREAK_TORQUE, 0)
    );
  }

  configureHingeMotor(args) {
    this.engine.configureHingeMotor(
      toString(args.ID, 'joint'),
      toNumber(args.SPEED, 0),
      toNumber(args.MAX_TORQUE, 0)
    );
  }

  configureHingeServo(args) {
    this.engine.configureHingeServo(
      toString(args.ID, 'joint'),
      toNumber(args.TARGET, 0),
      toNumber(args.MAX_SPEED, 180),
      toNumber(args.MAX_TORQUE, 0)
    );
  }

  raycast(args) {
    this.engine.raycast(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.DX, 0),
      toNumber(args.DY, -1),
      toNumber(args.DZ, 0),
      toNumber(args.LENGTH, 100)
    );
  }

  stepWorld(args) {
    this.engine.stepWorld(
      toNumber(args.SECONDS, 1 / 60)
    );
  }

  sphereCast(args) {
    this.engine.sphereCast(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.RADIUS, 10),
      toNumber(args.DX, 0),
      toNumber(args.DY, -1),
      toNumber(args.DZ, 0),
      toNumber(args.LENGTH, 100)
    );
  }

  capsuleCast(args) {
    this.engine.capsuleCast(
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.RADIUS, 10),
      toNumber(args.HALF_HEIGHT, 20),
      toNumber(args.DX, 0),
      toNumber(args.DY, -1),
      toNumber(args.DZ, 0),
      toNumber(args.LENGTH, 100)
    );
  }

  renderDebugFrame() {
    this.engine.renderDebugFrame();
  }

  showDebugOverlay() {
    this.engine.showDebugOverlay();
  }

  hideDebugOverlay() {
    this.engine.hideDebugOverlay();
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

  jointSummary(args) {
    return this.engine.getJointSummary(
      toString(args.ID, 'joint')
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

  raycastSummary() {
    return this.engine.getLastRaycastSummary();
  }

  shapeCastSummary() {
    return this.engine.getLastShapeCastSummary();
  }

  ccdSummary() {
    return this.engine.getCcdSummary();
  }

  debugFrameSummary() {
    return this.engine.getLastFrameSummary();
  }

  debugOverlaySummary() {
    return this.engine.getDebugOverlaySummary();
  }

  lastFrameSummary() {
    return this.engine.getLastFrameSummary();
  }

  hostSummary() {
    return this.engine.getHostSummary();
  }
}
