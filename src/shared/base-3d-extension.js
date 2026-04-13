import { Engine3D } from '../core/engine3d.js';
import { toNumber, toString } from './coerce.js';
import { createExtensionInfo } from './extension-metadata.js';

function resolveBlockType(blockType) {
  if (typeof blockType !== 'string' || blockType.length === 0) {
    return blockType;
  }

  const scratchBlockType = globalThis.Scratch?.BlockType ?? null;
  if (!scratchBlockType || typeof scratchBlockType !== 'object') {
    return blockType;
  }

  const normalizedKey = blockType.trim().toUpperCase().replace(/[\s-]+/g, '_');
  return scratchBlockType[normalizedKey] ?? blockType;
}

function normalizeExtensionInfo(info) {
  if (!info || !Array.isArray(info.blocks)) {
    return info;
  }

  return {
    ...info,
    blocks: info.blocks.map((block) => {
      if (!block || typeof block !== 'object') {
        return block;
      }

      return {
        ...block,
        blockType: resolveBlockType(block.blockType)
      };
    })
  };
}

export class Base3DExtension {
  constructor(hostBridge) {
    this.hostBridge = hostBridge;
    this.engine = new Engine3D(hostBridge);
  }

  getInfo() {
    return normalizeExtensionInfo(createExtensionInfo());
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

  createStaticBoxOneWayPlatform(args) {
    this.engine.createStaticBoxOneWayPlatform(
      toString(args.ID, 'one-way-box'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.SIZE, 100),
      toString(args.MATERIAL, '')
    );
  }

  createStaticBoxSensor(args) {
    this.engine.createStaticBoxSensor(
      toString(args.ID, 'sensor'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.SIZE, 100),
      toNumber(args.LAYER, 1),
      toNumber(args.MASK, 2147483647)
    );
  }

  createKinematicCapsule(args) {
    this.engine.createKinematicCapsule(
      toString(args.ID, 'character'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.RADIUS, 12),
      toNumber(args.HALF_HEIGHT, 24),
      toNumber(args.LAYER, 1),
      toNumber(args.MASK, 2147483647)
    );
  }

  configureKinematicCapsule(args) {
    this.engine.configureKinematicCapsule(
      toString(args.ID, 'character'),
      toNumber(args.SKIN, 0.5),
      toNumber(args.PROBE, 4),
      toNumber(args.MAX_SLOPE, 55)
    );
  }

  configureKinematicController(args) {
    this.engine.configureKinematicController(
      toString(args.ID, 'character'),
      toNumber(args.JUMP_SPEED, 8),
      toNumber(args.GRAVITY_SCALE, 1),
      toNumber(args.STEP_OFFSET, 6),
      toNumber(args.GROUND_SNAP, 2)
    );
  }

  configureKinematicControllerAdvanced(args) {
    this.engine.configureKinematicControllerAdvanced(
      toString(args.ID, 'character'),
      toNumber(args.AIR_CONTROL, 1),
      toNumber(args.COYOTE, 0.1),
      toNumber(args.BUFFER, 0.1),
      toString(args.PLATFORMS, 'on').toLowerCase() === 'on'
    );
  }

  setKinematicCapsuleMoveIntent(args) {
    this.engine.setKinematicCapsuleMoveIntent(
      toString(args.ID, 'character'),
      toNumber(args.DX),
      toNumber(args.DY),
      toNumber(args.DZ)
    );
  }

  jumpKinematicCapsule(args) {
    this.engine.jumpKinematicCapsule(
      toString(args.ID, 'character')
    );
  }

  moveKinematicCapsule(args) {
    this.engine.moveKinematicCapsule(
      toString(args.ID, 'character'),
      toNumber(args.DX),
      toNumber(args.DY),
      toNumber(args.DZ)
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

  createPresetConvexHullRigidBody(args) {
    this.engine.createPresetConvexHullRigidBody(
      toString(args.ID, 'body'),
      toString(args.PRESET, 'pyramid'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.SCALE, 100),
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

  createStaticConvexHullOneWayPlatform(args) {
    this.engine.createStaticConvexHullOneWayPlatform(
      toString(args.ID, 'one-way-hull'),
      toString(args.VERTICES, ''),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toString(args.MATERIAL, '')
    );
  }

  createStaticConvexHullSensor(args) {
    this.engine.createStaticConvexHullSensor(
      toString(args.ID, 'sensor'),
      toString(args.VERTICES, ''),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.LAYER, 1),
      toNumber(args.MASK, 2147483647)
    );
  }

  createPresetStaticConvexHullCollider(args) {
    this.engine.createPresetStaticConvexHullCollider(
      toString(args.ID, 'collider'),
      toString(args.PRESET, 'pyramid'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.SCALE, 100),
      toString(args.MATERIAL, '')
    );
  }

  createPresetStaticConvexHullSensor(args) {
    this.engine.createPresetStaticConvexHullSensor(
      toString(args.ID, 'sensor'),
      toString(args.PRESET, 'pyramid'),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toNumber(args.SCALE, 100),
      toNumber(args.LAYER, 1),
      toNumber(args.MASK, 2147483647)
    );
  }

  createClothSheet(args) {
    this.engine.createClothSheet(
      toString(args.ID, 'cloth'),
      toNumber(args.ROWS, 6),
      toNumber(args.COLUMNS, 8),
      toNumber(args.SPACING, 20),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toString(args.PIN_MODE, 'top-row')
    );
  }

  createSoftBodyCube(args) {
    this.engine.createSoftBodyCube(
      toString(args.ID, 'soft-body'),
      toNumber(args.ROWS, 4),
      toNumber(args.COLUMNS, 4),
      toNumber(args.LAYERS, 4),
      toNumber(args.SPACING, 20),
      toNumber(args.X),
      toNumber(args.Y),
      toNumber(args.Z),
      toString(args.PIN_MODE, 'none')
    );
  }

  configureCloth(args) {
    this.engine.configureCloth(
      toString(args.ID, 'cloth'),
      toNumber(args.DAMPING, 0.03),
      toNumber(args.MARGIN, 1),
      toNumber(args.STRETCH, 0),
      toNumber(args.SHEAR, 0.00015),
      toNumber(args.BEND, 0.0005),
      toString(args.SELF_COLLISION, 'off').toLowerCase() === 'on',
      toNumber(args.SELF_DISTANCE, 6)
    );
  }

  configureClothLightPreset(args) {
    this.engine.configureClothPreset(
      toString(args.ID, 'cloth'),
      'light-fabric'
    );
  }

  configureClothHeavyPreset(args) {
    this.engine.configureClothPreset(
      toString(args.ID, 'cloth'),
      'heavy-cloth'
    );
  }

  configureClothWrinklePreset(args) {
    this.engine.configureClothPreset(
      toString(args.ID, 'cloth'),
      'wrinkle'
    );
  }

  configureSoftBody(args) {
    this.engine.configureSoftBody(
      toString(args.ID, 'soft-body'),
      toNumber(args.DAMPING, 0.04),
      toNumber(args.MARGIN, 3),
      toNumber(args.STRETCH, 0),
      toNumber(args.SHEAR, 0.0002),
      toNumber(args.BEND, 0.0008),
      toNumber(args.VOLUME, 0.00008)
    );
  }

  configureSoftBodyJellyPreset(args) {
    this.engine.configureSoftBodyPreset(
      toString(args.ID, 'soft-body'),
      'jelly'
    );
  }

  configureSoftBodyFoamPreset(args) {
    this.engine.configureSoftBodyPreset(
      toString(args.ID, 'soft-body'),
      'foam'
    );
  }

  configureSoftBodyFirmRubberPreset(args) {
    this.engine.configureSoftBodyPreset(
      toString(args.ID, 'soft-body'),
      'firm-rubber'
    );
  }

  configureBodyCollision(args) {
    this.engine.configureBodyCollision(
      toString(args.ID, 'body'),
      toNumber(args.LAYER, 1),
      toNumber(args.MASK, 2147483647)
    );
  }

  configureColliderCollision(args) {
    this.engine.configureColliderCollision(
      toString(args.ID, 'collider'),
      toNumber(args.LAYER, 1),
      toNumber(args.MASK, 2147483647),
      toString(args.SENSOR, 'off').toLowerCase() === 'on'
    );
  }

  configureColliderOneWay(args) {
    this.engine.configureColliderOneWay(
      toString(args.ID, 'collider'),
      toString(args.ONE_WAY, 'on').toLowerCase() === 'on'
    );
  }

  convexHullPresetVertices(args) {
    return this.engine.getConvexHullPresetVertices(
      toString(args.PRESET, 'pyramid'),
      toNumber(args.SCALE, 100)
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

  loadSceneJson(args) {
    this.engine.loadSceneJson(
      toString(args.SCENE_JSON, '{}')
    );
  }

  setDebugOverlayLayers(args) {
    this.engine.setDebugOverlayLayers(
      toString(args.LAYERS, 'all')
    );
  }

  resetDebugOverlayLayers() {
    this.engine.resetDebugOverlayLayers();
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

  kinematicCapsuleSummary(args) {
    return this.engine.getKinematicCapsuleSummary(
      toString(args.ID, 'character')
    );
  }

  kinematicGroundSummary(args) {
    return this.engine.getKinematicGroundSummary(
      toString(args.ID, 'character')
    );
  }

  kinematicCapsuleHitSummary(args) {
    return this.engine.getKinematicCapsuleHitSummary(
      toString(args.ID, 'character')
    );
  }

  isKinematicCapsuleGrounded(args) {
    return this.engine.isKinematicCapsuleGrounded(
      toString(args.ID, 'character')
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

  clothSummary(args) {
    return this.engine.getClothSummary(
      toString(args.ID, 'cloth')
    );
  }

  softBodySummary(args) {
    return this.engine.getSoftBodySummary(
      toString(args.ID, 'soft-body')
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

  queryBodyContacts(args) {
    return this.engine.queryBodyContacts(
      toString(args.ID, 'body')
    );
  }

  queryColliderContacts(args) {
    return this.engine.queryColliderContacts(
      toString(args.ID, 'collider')
    );
  }

  queryBodyContactEvents(args) {
    return this.engine.queryBodyContactEvents(
      toString(args.ID, 'body'),
      toString(args.PHASE, 'stay')
    );
  }

  queryBodyTriggerEvents(args) {
    return this.engine.queryBodyTriggerEvents(
      toString(args.ID, 'body'),
      toString(args.PHASE, 'stay')
    );
  }

  queryKinematicCapsuleBodyHitEvents(args) {
    return this.engine.queryKinematicCapsuleBodyHitEvents(
      toString(args.ID, 'character'),
      toString(args.PHASE, 'stay')
    );
  }

  queryKinematicCapsuleColliderHitEvents(args) {
    return this.engine.queryKinematicCapsuleColliderHitEvents(
      toString(args.ID, 'character'),
      toString(args.PHASE, 'stay')
    );
  }

  queryColliderContactEvents(args) {
    return this.engine.queryColliderContactEvents(
      toString(args.ID, 'collider'),
      toString(args.PHASE, 'stay')
    );
  }

  queryColliderTriggerEvents(args) {
    return this.engine.queryColliderTriggerEvents(
      toString(args.ID, 'collider'),
      toString(args.PHASE, 'stay')
    );
  }

  sceneJson() {
    return this.engine.exportSceneJson();
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

  sceneIoSummary() {
    return this.engine.getSceneIoSummary();
  }

  contactEventsSummary() {
    return this.engine.getContactEventsSummary();
  }

  triggerEventsSummary() {
    return this.engine.getTriggerEventsSummary();
  }

  controllerHitEventsSummary() {
    return this.engine.getControllerHitEventsSummary();
  }

  lastFrameSummary() {
    return this.engine.getLastFrameSummary();
  }

  hostSummary() {
    return this.engine.getHostSummary();
  }
}
