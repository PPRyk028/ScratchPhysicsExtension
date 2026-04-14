import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';
import { buildAll } from '../scripts/build.mjs';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const projectRoot = path.resolve(__dirname, '..');
const distDir = path.join(projectRoot, 'dist');
let built = false;

const REQUIRED_OPCODES = [
  'resetWorld',
  'setGravity',
  'createMaterial',
  'setCameraPosition',
  'setCameraTarget',
  'createBoxRigidBody',
  'createConvexHullRigidBody',
  'createPresetConvexHullRigidBody',
  'createStaticBoxCollider',
  'createStaticBoxOneWayPlatform',
  'createStaticBoxSensor',
  'createStaticConvexHullCollider',
  'createStaticConvexHullOneWayPlatform',
  'createStaticConvexHullSensor',
  'createPresetStaticConvexHullCollider',
  'createPresetStaticConvexHullSensor',
  'createKinematicCapsule',
  'configureKinematicCapsule',
  'configureKinematicController',
  'configureKinematicControllerAdvanced',
  'setKinematicCapsuleMoveIntent',
  'setKinematicCapsuleCrouch',
  'setKinematicCapsuleVerticalVelocity',
  'jumpKinematicCapsule',
  'dropThroughOneWayPlatforms',
  'launchKinematicCapsule',
  'moveKinematicCapsule',
  'convexHullPresetVertices',
  'createClothSheet',
  'createSoftBodyCube',
  'configureCloth',
  'configureClothLightPreset',
  'configureClothHeavyPreset',
  'configureClothWrinklePreset',
  'configureSoftBody',
  'configureSoftBodyJellyPreset',
  'configureSoftBodyFoamPreset',
  'configureSoftBodyFirmRubberPreset',
  'configureBodyCollision',
  'configureColliderCollision',
  'configureColliderOneWay',
  'createDistanceJoint',
  'createPointToPointJoint',
  'createHingeJoint',
  'createFixedJoint',
  'configureDistanceJoint',
  'configureHingeJoint',
  'configureFixedJoint',
  'configureHingeMotor',
  'configureHingeServo',
  'stepWorld',
  'sphereCast',
  'capsuleCast',
  'raycast',
  'renderDebugFrame',
  'loadSceneJson',
  'showDebugOverlay',
  'hideDebugOverlay',
  'setDebugOverlayLayers',
  'resetDebugOverlayLayers',
  'worldSummary',
  'sceneJson',
  'rigidBodySummary',
  'colliderSummary',
  'materialSummary',
  'jointSummary',
  'clothSummary',
  'softBodySummary',
  'kinematicCapsuleSummary',
  'kinematicGroundSummary',
  'kinematicGroundCollider',
  'kinematicCapsuleHitSummary',
  'kinematicBlockingCollider',
  'isKinematicCapsuleGrounded',
  'queryPointBodies',
  'queryPointColliders',
  'queryAabbBodies',
  'queryAabbColliders',
  'queryBodyContacts',
  'queryColliderContacts',
  'queryBodyContactEvents',
  'queryBodyTriggerEvents',
  'queryKinematicCapsuleBodyHitEvents',
  'queryKinematicCapsuleColliderHitEvents',
  'queryColliderContactEvents',
  'queryColliderTriggerEvents',
  'raycastSummary',
  'shapeCastSummary',
  'ccdSummary',
  'debugFrameSummary',
  'debugOverlaySummary',
  'sceneIoSummary',
  'contactEventsSummary',
  'triggerEventsSummary',
  'controllerHitEventsSummary',
  'hostSummary',
  'resetScene',
  'addCube',
  'sceneSummary',
  'lastFrameSummary'
];

function ensureBuild() {
  if (built) {
    return;
  }

  buildAll();
  built = true;
}

function readBundle(name) {
  return fs.readFileSync(path.join(distDir, name), 'utf8');
}

function runBundle(code, contextValues) {
  const context = vm.createContext({
    console,
    ...contextValues
  });

  context.globalThis = context;
  if (!context.window) {
    context.window = context;
  }

  const script = new vm.Script(code, {
    filename: 'bundle.js'
  });

  script.runInContext(context);
  return context;
}

function createTurboWarpContext({ unsandboxed = true, withRenderer = true } = {}) {
  const registrations = [];
  const renderer = withRenderer ? { tag: 'renderer' } : null;
  const runtime = { renderer };
  const Scratch = {
    BlockType: {
      COMMAND: 'command',
      REPORTER: 'reporter',
      BOOLEAN: 'Boolean'
    },
    vm: {
      runtime,
      renderer
    },
    extensions: {
      unsandboxed,
      register(extensionInstance) {
        registrations.push(extensionInstance);
      }
    }
  };

  return {
    Scratch,
    registrations,
    runtime,
    renderer
  };
}

function createGandiContext({ withRenderer = true } = {}) {
  const renderer = withRenderer ? { tag: 'renderer' } : null;
  const runtime = { renderer };
  return {
    Scratch: {
      BlockType: {
        COMMAND: 'command',
        REPORTER: 'reporter',
        BOOLEAN: 'Boolean'
      }
    },
    runtime,
    window: {}
  };
}

function getOpcodes(info) {
  return info.blocks
    .filter((block) => typeof block === 'object' && block !== null && block.opcode)
    .map((block) => block.opcode);
}

function normalizeRealmValue(value) {
  return JSON.parse(JSON.stringify(value));
}

function assertHasRequiredOpcodes(info) {
  const opcodes = new Set(normalizeRealmValue(getOpcodes(info)));
  for (const opcode of REQUIRED_OPCODES) {
    assert.equal(opcodes.has(opcode), true, `Missing opcode: ${opcode}`);
  }
}

test('build emits both platform bundles', () => {
  ensureBuild();

  assert.equal(fs.existsSync(path.join(distDir, 'turbowarp-unsandboxed.js')), true);
  assert.equal(fs.existsSync(path.join(distDir, 'gandi-normal-remote.js')), true);
  assert.equal(fs.existsSync(path.join(distDir, 'gandi-approved.js')), true);
});

test('TurboWarp bundle registers an extension in unsandboxed mode', () => {
  ensureBuild();
  const code = readBundle('turbowarp-unsandboxed.js');
  const turboWarp = createTurboWarpContext({ unsandboxed: true, withRenderer: true });
  const context = runBundle(code, turboWarp);

  assert.equal(context.registrations.length, 1);

  const extension = context.registrations[0];
  const info = normalizeRealmValue(extension.getInfo());
  const groundedBlock = info.blocks.find((block) => block?.opcode === 'isKinematicCapsuleGrounded');

  assert.equal(info.id, 'engine3d');
  assertHasRequiredOpcodes(info);
  assert.equal(groundedBlock?.blockType, turboWarp.Scratch.BlockType.BOOLEAN);

  extension.resetWorld();
  extension.createMaterial({ ID: 'ice', FRICTION: 0.05, RESTITUTION: 0.2, DENSITY: 0.9 });
  extension.setGravity({ X: 0, Y: -10, Z: 0 });
  extension.setCameraPosition({ X: 10, Y: 20, Z: 300 });
  extension.setCameraTarget({ X: 10, Y: 0, Z: 0 });
  extension.createStaticBoxCollider({ ID: 'floor', X: 0, Y: -10, Z: 0, SIZE: 100, MATERIAL: 'ice' });
  extension.createStaticBoxSensor({ ID: 'sensor-zone', X: 5, Y: 6, Z: 7, SIZE: 60, LAYER: 8, MASK: 1 });
  extension.createStaticConvexHullCollider({ ID: 'ramp', VERTICES: '-10 -10 -10; 10 -10 -10; 10 -10 10; -10 -10 10; 0 10 0', X: 0, Y: -20, Z: 40, MATERIAL: 'ice' });
  extension.createPresetStaticConvexHullSensor({ ID: 'sensor-ramp', PRESET: 'wedge', X: 30, Y: 0, Z: 0, SCALE: 20, LAYER: 16, MASK: 1 });
  extension.createBoxRigidBody({ ID: 'probe', X: 5, Y: 6, Z: 7, SIZE: 42, MASS: 2, MATERIAL: 'ice' });
  extension.createBoxRigidBody({ ID: 'probe-2', X: 40, Y: 6, Z: 7, SIZE: 42, MASS: 2, MATERIAL: 'ice' });
  extension.configureBodyCollision({ ID: 'probe', LAYER: 1, MASK: 2147483647 });
  extension.configureColliderCollision({ ID: 'probe-2:collider', LAYER: 4, MASK: 2147483647, SENSOR: 'off' });
  extension.configureColliderOneWay({ ID: 'ramp:collider', ONE_WAY: 'on' });
  assert.match(extension.colliderSummary({ ID: 'ramp:collider' }), /one-way:on/);
  extension.createKinematicCapsule({ ID: 'player', X: 0, Y: 15, Z: 0, RADIUS: 5, HALF_HEIGHT: 10, LAYER: 2, MASK: 2147483647 });
  extension.configureKinematicCapsule({ ID: 'player', SKIN: 1.25, PROBE: 6, MAX_SLOPE: 60 });
  extension.configureKinematicController({ ID: 'player', JUMP_SPEED: 9, GRAVITY_SCALE: 1.2, STEP_OFFSET: 7, GROUND_SNAP: 3 });
  extension.configureKinematicControllerAdvanced({ ID: 'player', AIR_CONTROL: 0.6, COYOTE: 0.12, BUFFER: 0.14, PLATFORMS: 'on' });
  extension.setKinematicCapsuleMoveIntent({ ID: 'player', DX: 3, DY: 0, DZ: 0 });
  extension.createConvexHullRigidBody({ ID: 'probe-hull', VERTICES: '-8 -8 -8; 8 -8 -8; 8 -8 8; -8 -8 8; 0 8 0', X: -20, Y: 30, Z: 0, MASS: 1.5, MATERIAL: 'ice' });
  extension.createPresetConvexHullRigidBody({ ID: 'preset-hull', PRESET: 'skew-prism', X: -50, Y: 20, Z: 0, SCALE: 40, MASS: 1, MATERIAL: 'ice' });
  extension.createPresetStaticConvexHullCollider({ ID: 'preset-ramp', PRESET: 'skew-frustum', X: 60, Y: -30, Z: 0, SCALE: 30, MATERIAL: 'ice' });
  extension.createClothSheet({ ID: 'cloth-1', ROWS: 4, COLUMNS: 5, SPACING: 10, X: -20, Y: 80, Z: 0, PIN_MODE: 'top-row' });
  extension.createSoftBodyCube({ ID: 'soft-1', ROWS: 3, COLUMNS: 3, LAYERS: 3, SPACING: 10, X: -10, Y: 90, Z: -10, PIN_MODE: 'none' });
  extension.configureCloth({ ID: 'cloth-1', DAMPING: 0.08, MARGIN: 4, STRETCH: 0.001, SHEAR: 0.002, BEND: 0.003, SELF_COLLISION: 'on', SELF_DISTANCE: 9 });
  extension.configureClothLightPreset({ ID: 'cloth-1' });
  extension.configureClothHeavyPreset({ ID: 'cloth-1' });
  extension.configureClothWrinklePreset({ ID: 'cloth-1' });
  extension.configureSoftBody({ ID: 'soft-1', DAMPING: 0.07, MARGIN: 3.5, STRETCH: 0, SHEAR: 0.0003, BEND: 0.0012, VOLUME: 0.00011 });
  extension.configureSoftBodyJellyPreset({ ID: 'soft-1' });
  assert.match(extension.softBodySummary({ ID: 'soft-1' }), /damping:0.02/);
  assert.match(extension.softBodySummary({ ID: 'soft-1' }), /volume:0.006/);
  extension.configureSoftBodyFoamPreset({ ID: 'soft-1' });
  assert.match(extension.softBodySummary({ ID: 'soft-1' }), /damping:0.035/);
  assert.match(extension.softBodySummary({ ID: 'soft-1' }), /volume:0.001/);
  extension.configureSoftBodyFirmRubberPreset({ ID: 'soft-1' });
  extension.createDistanceJoint({ ID: 'joint-1', BODY_A: 'probe', BODY_B: 'probe-2', LENGTH: 35 });
  extension.createPointToPointJoint({ ID: 'joint-2', BODY_A: 'probe', BODY_B: 'probe-2', X: 20, Y: 6, Z: 7 });
  extension.createHingeJoint({ ID: 'joint-3', BODY_A: 'probe', BODY_B: 'probe-2', X: 20, Y: 6, Z: 7, AX: 0, AY: 1, AZ: 0 });
  extension.createFixedJoint({ ID: 'joint-4', BODY_A: 'probe', BODY_B: 'probe-2', X: 20, Y: 6, Z: 7 });
  extension.configureDistanceJoint({ ID: 'joint-1', MIN: 20, MAX: 45, SPRING: 2, DAMPING: 0.8 });
  extension.configureHingeJoint({ ID: 'joint-3', LOWER: -30, UPPER: 30, DAMPING: 2 });
  extension.configureFixedJoint({ ID: 'joint-4', BREAK_FORCE: 2500, BREAK_TORQUE: 1800 });
  extension.configureHingeMotor({ ID: 'joint-3', SPEED: 180, MAX_TORQUE: 25 });
  extension.configureHingeServo({ ID: 'joint-3', TARGET: 30, MAX_SPEED: 240, MAX_TORQUE: 25 });
  const exportedScene = extension.sceneJson();
  assert.match(exportedScene, /physics-scene@1/);
  extension.loadSceneJson({ SCENE_JSON: exportedScene });
  extension.setDebugOverlayLayers({ LAYERS: 'contacts joints' });
  extension.sphereCast({ X: 0, Y: 120, Z: 0, RADIUS: 10, DX: 0, DY: -1, DZ: 0, LENGTH: 300 });
  extension.capsuleCast({ X: 0, Y: 120, Z: 0, RADIUS: 8, HALF_HEIGHT: 20, DX: 0, DY: -1, DZ: 0, LENGTH: 300 });
  extension.raycast({ X: 0, Y: 120, Z: 0, DX: 0, DY: -1, DZ: 0, LENGTH: 300 });
  extension.showDebugOverlay();
  assert.match(extension.queryPointBodies({ X: 5, Y: 6, Z: 7 }), /1 bodies/);
  assert.match(extension.queryAabbColliders({ X: 0, Y: 0, Z: 0, HX: 100, HY: 100, HZ: 100 }), /10 colliders/);
  assert.match(extension.queryBodyContacts({ ID: 'probe' }), /(bodies touching|not found)/);
  assert.match(extension.queryColliderContacts({ ID: 'probe:collider' }), /(colliders touching|not found)/);
  assert.match(extension.queryBodyTriggerEvents({ ID: 'probe', PHASE: 'enter' }), /bodies in enter trigger events/);
  assert.match(extension.queryColliderTriggerEvents({ ID: 'sensor-zone:collider', PHASE: 'enter' }), /colliders in enter trigger events/);
  assert.match(extension.queryBodyContactEvents({ ID: 'probe', PHASE: 'enter' }), /(bodies in enter contact events|not found)/);
  assert.match(extension.queryColliderContactEvents({ ID: 'probe:collider', PHASE: 'enter' }), /(colliders in enter contact events|not found)/);
  assert.match(extension.convexHullPresetVertices({ PRESET: 'pyramid', SCALE: 50 }), /;/);
  extension.moveKinematicCapsule({ ID: 'player', DX: 6, DY: 0, DZ: 0 });
  extension.stepWorld({ SECONDS: 1 / 60 });
  extension.renderDebugFrame();
  extension.hideDebugOverlay();
  extension.resetDebugOverlayLayers();

  assert.match(extension.worldSummary(), /5 bodies/);
  assert.match(extension.worldSummary(), /1 characters/);
  assert.match(extension.worldSummary(), /1 cloths/);
  assert.match(extension.worldSummary(), /1 soft bodies/);
  assert.match(extension.jointSummary({ ID: 'joint-1' }), /distance-joint/);
  assert.match(extension.jointSummary({ ID: 'joint-3' }), /motor mode:servo/);
  assert.match(extension.jointSummary({ ID: 'joint-4' }), /break force:/);
  assert.match(extension.clothSummary({ ID: 'cloth-1' }), /rows:4/);
  assert.match(extension.clothSummary({ ID: 'cloth-1' }), /self:on/);
  assert.match(extension.clothSummary({ ID: 'cloth-1' }), /damping:0.08/);
  assert.match(extension.clothSummary({ ID: 'cloth-1' }), /margin:2.5/);
  assert.match(extension.clothSummary({ ID: 'cloth-1' }), /bend:0.015/);
  assert.match(extension.clothSummary({ ID: 'cloth-1' }), /self distance:7/);
  assert.match(extension.softBodySummary({ ID: 'soft-1' }), /layers:3/);
  assert.match(extension.softBodySummary({ ID: 'soft-1' }), /damping:0.045/);
  assert.match(extension.softBodySummary({ ID: 'soft-1' }), /volume:0.0003/);
  assert.match(extension.softBodySummary({ ID: 'soft-1' }), /dynamic contacts:/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /radius:5/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /grounded:yes/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /jump:9/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /step offset:7/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /air:0.6/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /coyote:0.12/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /buffer:0.14/);
  assert.match(extension.kinematicGroundSummary({ ID: 'player' }), /grounded:(yes|no)/);
  assert.equal(extension.kinematicGroundCollider({ ID: 'player' }), 'floor:collider');
  assert.match(extension.kinematicCapsuleHitSummary({ ID: 'player' }), /last hit:/);
  assert.equal(extension.kinematicBlockingCollider({ ID: 'player' }), 'none');
  assert.equal(typeof extension.isKinematicCapsuleGrounded({ ID: 'player' }), 'boolean');
  assert.match(extension.queryKinematicCapsuleBodyHitEvents({ ID: 'player', PHASE: 'stay' }), /bodies in stay controller hit events/);
  assert.match(extension.queryKinematicCapsuleColliderHitEvents({ ID: 'player', PHASE: 'stay' }), /colliders in stay controller hit events/);
  extension.setKinematicCapsuleCrouch({ ID: 'player', HALF_HEIGHT: 8, RADIUS: 4, SPEED: 15 });
  extension.dropThroughOneWayPlatforms({ ID: 'player', SECONDS: 0.25 });
  extension.setKinematicCapsuleVerticalVelocity({ ID: 'player', VELOCITY: 5 });
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /vertical velocity:5/);
  extension.launchKinematicCapsule({ ID: 'player', DX: 2, DY: 7, DZ: 0 });
  assert.match(extension.kinematicCapsuleSummary({ ID: 'player' }), /vertical velocity:5/);
  extension.jumpKinematicCapsule({ ID: 'player' });
  assert.match(extension.colliderSummary({ ID: 'floor:collider' }), /body:static/);
  assert.match(extension.materialSummary({ ID: 'ice' }), /friction:0.05/);
  assert.match(extension.raycastSummary(), /Ray hit/);
  assert.match(extension.shapeCastSummary(), /cast hit/);
  assert.match(extension.ccdSummary(), /(No CCD events|CCD events)/);
  assert.match(extension.debugFrameSummary(), /TurboWarp frame 1/);
  assert.match(extension.debugOverlaySummary(), /overlay/);
  assert.match(extension.sceneIoSummary(), /Scene loaded/);
  assert.match(extension.contactEventsSummary(), /contact events/);
  assert.match(extension.triggerEventsSummary(), /trigger events/);
  assert.match(extension.controllerHitEventsSummary(), /controller hit events/);
  assert.match(extension.hostSummary(), /TurboWarp/);
  assert.match(extension.hostSummary(), /runtime:yes/);
  assert.match(extension.hostSummary(), /renderer:yes/);
});

test('TurboWarp bundle rejects sandboxed loading', () => {
  ensureBuild();
  const code = readBundle('turbowarp-unsandboxed.js');
  const turboWarp = createTurboWarpContext({ unsandboxed: false, withRenderer: true });

  assert.throws(
    () => runBundle(code, turboWarp),
    /unsandboxed TurboWarp extension/
  );
});

test('Gandi bundle exposes approved-extension tempExt contract', () => {
  ensureBuild();
  const code = readBundle('gandi-approved.js');
  const gandi = createGandiContext({ withRenderer: true });
  const context = runBundle(code, gandi);

  assert.ok(context.window.tempExt);
  assert.equal(typeof context.window.tempExt.Extension, 'function');
  assert.equal(context.window.tempExt.info.extensionId, 'engine3d');
  assert.ok(context.window.tempExt.l10n.en);
});

test('Gandi normal remote bundle registers in custom-extension flow', () => {
  ensureBuild();
  const code = readBundle('gandi-normal-remote.js');
  const gandiRemote = createTurboWarpContext({ unsandboxed: false, withRenderer: false });
  const context = runBundle(code, gandiRemote);

  assert.equal(context.registrations.length, 1);

  const extension = context.registrations[0];
  const info = normalizeRealmValue(extension.getInfo());
  const groundedBlock = info.blocks.find((block) => block?.opcode === 'isKinematicCapsuleGrounded');

  assert.equal(info.id, 'engine3d');
  assertHasRequiredOpcodes(info);
  assert.equal(groundedBlock?.blockType, gandiRemote.Scratch.BlockType.BOOLEAN);
  extension.resetWorld();
  extension.setCameraTarget({ X: 0, Y: -5, Z: 0 });
  extension.createStaticBoxCollider({ ID: 'remote-floor', X: 0, Y: -5, Z: 0, SIZE: 10, MATERIAL: 'material-default' });
  extension.createStaticBoxSensor({ ID: 'remote-sensor', X: 0, Y: 0, Z: 0, SIZE: 12, LAYER: 8, MASK: 1 });
  extension.createStaticConvexHullCollider({ ID: 'remote-ramp', VERTICES: '-4 -4 -4; 4 -4 -4; 4 -4 4; -4 -4 4; 0 4 0', X: 0, Y: -12, Z: 15, MATERIAL: 'material-default' });
  extension.createPresetStaticConvexHullSensor({ ID: 'remote-sensor-ramp', PRESET: 'wedge', X: 18, Y: -5, Z: 0, SCALE: 12, LAYER: 16, MASK: 1 });
  extension.createBoxRigidBody({ ID: 'remote-probe', X: 0, Y: 0, Z: 0, SIZE: 10, MASS: 1, MATERIAL: 'material-default' });
  extension.createBoxRigidBody({ ID: 'remote-probe-2', X: 20, Y: 0, Z: 0, SIZE: 10, MASS: 1, MATERIAL: 'material-default' });
  extension.configureBodyCollision({ ID: 'remote-probe', LAYER: 1, MASK: 2147483647 });
  extension.configureColliderCollision({ ID: 'remote-probe-2:collider', LAYER: 4, MASK: 2147483647, SENSOR: 'off' });
  extension.configureColliderOneWay({ ID: 'remote-ramp:collider', ONE_WAY: 'on' });
  assert.match(extension.colliderSummary({ ID: 'remote-ramp:collider' }), /one-way:on/);
  extension.createKinematicCapsule({ ID: 'remote-player', X: 0, Y: 15, Z: 0, RADIUS: 5, HALF_HEIGHT: 10, LAYER: 2, MASK: 2147483647 });
  extension.configureKinematicCapsule({ ID: 'remote-player', SKIN: 1.25, PROBE: 6, MAX_SLOPE: 60 });
  extension.configureKinematicController({ ID: 'remote-player', JUMP_SPEED: 9, GRAVITY_SCALE: 1.2, STEP_OFFSET: 7, GROUND_SNAP: 3 });
  extension.configureKinematicControllerAdvanced({ ID: 'remote-player', AIR_CONTROL: 0.55, COYOTE: 0.11, BUFFER: 0.13, PLATFORMS: 'on' });
  extension.setKinematicCapsuleMoveIntent({ ID: 'remote-player', DX: 2, DY: 0, DZ: 0 });
  extension.createConvexHullRigidBody({ ID: 'remote-hull', VERTICES: '-3 -3 -3; 3 -3 -3; 3 -3 3; -3 -3 3; 0 3 0', X: -15, Y: 6, Z: 0, MASS: 1, MATERIAL: 'material-default' });
  extension.createPresetConvexHullRigidBody({ ID: 'remote-preset-hull', PRESET: 'wedge', X: -25, Y: 12, Z: 0, SCALE: 20, MASS: 1, MATERIAL: 'material-default' });
  extension.createPresetStaticConvexHullCollider({ ID: 'remote-preset-ramp', PRESET: 'skew-frustum', X: 30, Y: -16, Z: 0, SCALE: 20, MATERIAL: 'material-default' });
  extension.createClothSheet({ ID: 'remote-cloth', ROWS: 3, COLUMNS: 4, SPACING: 8, X: -12, Y: 40, Z: 0, PIN_MODE: 'top-corners' });
  extension.createSoftBodyCube({ ID: 'remote-soft', ROWS: 3, COLUMNS: 3, LAYERS: 3, SPACING: 8, X: -8, Y: 48, Z: -8, PIN_MODE: 'none' });
  extension.configureCloth({ ID: 'remote-cloth', DAMPING: 0.04, MARGIN: 2.5, STRETCH: 0.0005, SHEAR: 0.001, BEND: 0.002, SELF_COLLISION: 'on', SELF_DISTANCE: 7 });
  extension.configureClothLightPreset({ ID: 'remote-cloth' });
  extension.configureClothHeavyPreset({ ID: 'remote-cloth' });
  extension.configureClothWrinklePreset({ ID: 'remote-cloth' });
  extension.configureSoftBody({ ID: 'remote-soft', DAMPING: 0.05, MARGIN: 2.5, STRETCH: 0, SHEAR: 0.00025, BEND: 0.001, VOLUME: 0.00009 });
  extension.configureSoftBodyJellyPreset({ ID: 'remote-soft' });
  assert.match(extension.softBodySummary({ ID: 'remote-soft' }), /damping:0.02/);
  extension.configureSoftBodyFoamPreset({ ID: 'remote-soft' });
  assert.match(extension.softBodySummary({ ID: 'remote-soft' }), /volume:0.001/);
  extension.configureSoftBodyFirmRubberPreset({ ID: 'remote-soft' });
  extension.createDistanceJoint({ ID: 'remote-joint', BODY_A: 'remote-probe', BODY_B: 'remote-probe-2', LENGTH: 20 });
  extension.createPointToPointJoint({ ID: 'remote-ball', BODY_A: 'remote-probe', BODY_B: 'remote-probe-2', X: 10, Y: 0, Z: 0 });
  extension.createHingeJoint({ ID: 'remote-hinge', BODY_A: 'remote-probe', BODY_B: 'remote-probe-2', X: 10, Y: 0, Z: 0, AX: 0, AY: 1, AZ: 0 });
  extension.createFixedJoint({ ID: 'remote-fixed', BODY_A: 'remote-probe', BODY_B: 'remote-probe-2', X: 10, Y: 0, Z: 0 });
  extension.configureDistanceJoint({ ID: 'remote-joint', MIN: 15, MAX: 25, SPRING: 1.5, DAMPING: 0.6 });
  extension.configureHingeJoint({ ID: 'remote-hinge', LOWER: -20, UPPER: 20, DAMPING: 1.5 });
  extension.configureFixedJoint({ ID: 'remote-fixed', BREAK_FORCE: 1200, BREAK_TORQUE: 900 });
  extension.configureHingeMotor({ ID: 'remote-hinge', SPEED: 90, MAX_TORQUE: 10 });
  extension.configureHingeServo({ ID: 'remote-hinge', TARGET: 15, MAX_SPEED: 120, MAX_TORQUE: 10 });
  extension.sphereCast({ X: 0, Y: 40, Z: 0, RADIUS: 5, DX: 0, DY: -1, DZ: 0, LENGTH: 100 });
  extension.capsuleCast({ X: 0, Y: 40, Z: 0, RADIUS: 4, HALF_HEIGHT: 8, DX: 0, DY: -1, DZ: 0, LENGTH: 100 });
  extension.raycast({ X: 0, Y: 40, Z: 0, DX: 0, DY: -1, DZ: 0, LENGTH: 100 });
  extension.setDebugOverlayLayers({ LAYERS: 'contacts queries' });
  extension.showDebugOverlay();
  extension.moveKinematicCapsule({ ID: 'remote-player', DX: 4, DY: 0, DZ: 0 });
  extension.renderDebugFrame();
  extension.hideDebugOverlay();
  extension.resetDebugOverlayLayers();

  assert.match(extension.worldSummary(), /5 bodies/);
  assert.match(extension.worldSummary(), /1 characters/);
  assert.match(extension.worldSummary(), /1 soft bodies/);
  assert.match(extension.clothSummary({ ID: 'remote-cloth' }), /pin:top-corners/);
  assert.match(extension.clothSummary({ ID: 'remote-cloth' }), /self:on/);
  assert.match(extension.clothSummary({ ID: 'remote-cloth' }), /margin:2.5/);
  assert.match(extension.clothSummary({ ID: 'remote-cloth' }), /bend:0.015/);
  assert.match(extension.softBodySummary({ ID: 'remote-soft' }), /layers:3/);
  assert.match(extension.softBodySummary({ ID: 'remote-soft' }), /damping:0.045/);
  assert.match(extension.softBodySummary({ ID: 'remote-soft' }), /volume:0.0003/);
  assert.match(extension.softBodySummary({ ID: 'remote-soft' }), /dynamic contacts:/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'remote-player' }), /radius:5/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'remote-player' }), /jump:9/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'remote-player' }), /air:0.55/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'remote-player' }), /buffer:0.13/);
  assert.match(extension.kinematicGroundSummary({ ID: 'remote-player' }), /grounded:yes/);
  assert.equal(extension.kinematicGroundCollider({ ID: 'remote-player' }), 'remote-floor:collider');
  assert.match(extension.kinematicCapsuleHitSummary({ ID: 'remote-player' }), /last hit:/);
  assert.equal(extension.kinematicBlockingCollider({ ID: 'remote-player' }), 'none');
  assert.equal(extension.isKinematicCapsuleGrounded({ ID: 'remote-player' }), true);
  assert.match(extension.queryKinematicCapsuleBodyHitEvents({ ID: 'remote-player', PHASE: 'stay' }), /bodies in stay controller hit events/);
  assert.match(extension.queryKinematicCapsuleColliderHitEvents({ ID: 'remote-player', PHASE: 'stay' }), /colliders in stay controller hit events/);
  extension.setKinematicCapsuleCrouch({ ID: 'remote-player', HALF_HEIGHT: 8, RADIUS: 4, SPEED: 15 });
  extension.dropThroughOneWayPlatforms({ ID: 'remote-player', SECONDS: 0.25 });
  extension.setKinematicCapsuleVerticalVelocity({ ID: 'remote-player', VELOCITY: 4 });
  assert.match(extension.kinematicCapsuleSummary({ ID: 'remote-player' }), /vertical velocity:4/);
  extension.launchKinematicCapsule({ ID: 'remote-player', DX: 2, DY: 6, DZ: 0 });
  assert.match(extension.kinematicCapsuleSummary({ ID: 'remote-player' }), /vertical velocity:4/);
  extension.jumpKinematicCapsule({ ID: 'remote-player' });
  assert.match(extension.jointSummary({ ID: 'remote-joint' }), /distance-joint/);
  assert.match(extension.jointSummary({ ID: 'remote-hinge' }), /motor mode:servo/);
  assert.match(extension.jointSummary({ ID: 'remote-fixed' }), /break force:/);
  assert.match(extension.queryPointColliders({ X: 0, Y: 0, Z: 0 }), /4 colliders/);
  assert.match(extension.queryBodyContacts({ ID: 'remote-probe' }), /(bodies touching|not found)/);
  assert.match(extension.queryColliderContacts({ ID: 'remote-probe:collider' }), /(colliders touching|not found)/);
  assert.match(extension.queryBodyTriggerEvents({ ID: 'remote-probe', PHASE: 'enter' }), /bodies in enter trigger events/);
  assert.match(extension.queryColliderTriggerEvents({ ID: 'remote-sensor:collider', PHASE: 'enter' }), /colliders in enter trigger events/);
  assert.match(extension.queryBodyContactEvents({ ID: 'remote-probe', PHASE: 'enter' }), /(bodies in enter contact events|not found)/);
  assert.match(extension.queryColliderContactEvents({ ID: 'remote-probe:collider', PHASE: 'enter' }), /(colliders in enter contact events|not found)/);
  assert.match(extension.raycastSummary(), /Ray hit/);
  assert.match(extension.shapeCastSummary(), /cast hit/);
  assert.match(extension.debugFrameSummary(), /Gandi Remote frame 1/);
  assert.match(extension.debugOverlaySummary(), /overlay/);
  assert.match(extension.contactEventsSummary(), /contact events/);
  assert.match(extension.triggerEventsSummary(), /trigger events/);
  assert.match(extension.controllerHitEventsSummary(), /controller hit events/);
  assert.match(extension.hostSummary(), /Gandi Remote/);
  assert.match(extension.hostSummary(), /sandbox:yes/);
});

test('Gandi extension instance runs shared blocks against the same core contract', () => {
  ensureBuild();
  const code = readBundle('gandi-approved.js');
  const gandi = createGandiContext({ withRenderer: true });
  const context = runBundle(code, gandi);
  const Extension = context.window.tempExt.Extension;
  const extension = new Extension(gandi.runtime);
  const info = normalizeRealmValue(extension.getInfo());

  assert.equal(info.id, 'engine3d');
  assertHasRequiredOpcodes(info);

  extension.resetWorld();
  extension.createMaterial({ ID: 'rubber', FRICTION: 1, RESTITUTION: 0.8, DENSITY: 1.2 });
  extension.setGravity({ X: 0, Y: -12, Z: 0 });
  extension.setCameraPosition({ X: -12, Y: 0, Z: 512 });
  extension.setCameraTarget({ X: 0, Y: 20, Z: 0 });
  extension.createStaticBoxCollider({ ID: 'gandi-floor', X: 1, Y: -10, Z: 3, SIZE: 64, MATERIAL: 'rubber' });
  extension.createStaticBoxSensor({ ID: 'gandi-sensor', X: 1, Y: 2, Z: 3, SIZE: 20, LAYER: 8, MASK: 1 });
  extension.createStaticConvexHullCollider({ ID: 'gandi-ramp', VERTICES: '-12 -12 -12; 12 -12 -12; 12 -12 12; -12 -12 12; 0 12 0', X: 1, Y: -24, Z: 28, MATERIAL: 'rubber' });
  extension.createPresetStaticConvexHullSensor({ ID: 'gandi-sensor-ramp', PRESET: 'wedge', X: 30, Y: -10, Z: 3, SCALE: 20, LAYER: 16, MASK: 1 });
  extension.createBoxRigidBody({ ID: 'gandi-probe', X: 1, Y: 2, Z: 3, SIZE: 64, MASS: 3, MATERIAL: 'rubber' });
  extension.createBoxRigidBody({ ID: 'gandi-link', X: 40, Y: 2, Z: 3, SIZE: 64, MASS: 3, MATERIAL: 'rubber' });
  extension.configureBodyCollision({ ID: 'gandi-probe', LAYER: 1, MASK: 2147483647 });
  extension.configureColliderCollision({ ID: 'gandi-link:collider', LAYER: 4, MASK: 2147483647, SENSOR: 'off' });
  extension.configureColliderOneWay({ ID: 'gandi-ramp:collider', ONE_WAY: 'on' });
  assert.match(extension.colliderSummary({ ID: 'gandi-ramp:collider' }), /one-way:on/);
  extension.createKinematicCapsule({ ID: 'gandi-player', X: 1, Y: 15, Z: 3, RADIUS: 5, HALF_HEIGHT: 10, LAYER: 2, MASK: 2147483647 });
  extension.configureKinematicCapsule({ ID: 'gandi-player', SKIN: 1.25, PROBE: 6, MAX_SLOPE: 60 });
  extension.configureKinematicController({ ID: 'gandi-player', JUMP_SPEED: 9, GRAVITY_SCALE: 1.2, STEP_OFFSET: 7, GROUND_SNAP: 3 });
  extension.configureKinematicControllerAdvanced({ ID: 'gandi-player', AIR_CONTROL: 0.65, COYOTE: 0.1, BUFFER: 0.12, PLATFORMS: 'on' });
  extension.setKinematicCapsuleMoveIntent({ ID: 'gandi-player', DX: 2, DY: 0, DZ: 0 });
  extension.createConvexHullRigidBody({ ID: 'gandi-hull', VERTICES: '-10 -10 -10; 10 -10 -10; 10 -10 10; -10 -10 10; 0 10 0', X: -30, Y: 20, Z: 3, MASS: 2, MATERIAL: 'rubber' });
  extension.createPresetConvexHullRigidBody({ ID: 'gandi-preset-hull', PRESET: 'skew-hexahedron', X: -50, Y: 30, Z: 3, SCALE: 24, MASS: 2, MATERIAL: 'rubber' });
  extension.createPresetStaticConvexHullCollider({ ID: 'gandi-preset-ramp', PRESET: 'wedge', X: 50, Y: -16, Z: 3, SCALE: 30, MATERIAL: 'rubber' });
  extension.createClothSheet({ ID: 'gandi-cloth', ROWS: 4, COLUMNS: 5, SPACING: 10, X: -20, Y: 70, Z: 3, PIN_MODE: 'top-row' });
  extension.createSoftBodyCube({ ID: 'gandi-soft', ROWS: 3, COLUMNS: 3, LAYERS: 3, SPACING: 10, X: -15, Y: 85, Z: -8, PIN_MODE: 'none' });
  extension.configureCloth({ ID: 'gandi-cloth', DAMPING: 0.05, MARGIN: 3, STRETCH: 0.0008, SHEAR: 0.0012, BEND: 0.0025, SELF_COLLISION: 'on', SELF_DISTANCE: 8 });
  extension.configureClothLightPreset({ ID: 'gandi-cloth' });
  extension.configureClothHeavyPreset({ ID: 'gandi-cloth' });
  extension.configureClothWrinklePreset({ ID: 'gandi-cloth' });
  extension.configureSoftBody({ ID: 'gandi-soft', DAMPING: 0.06, MARGIN: 3, STRETCH: 0, SHEAR: 0.0003, BEND: 0.0011, VOLUME: 0.0001 });
  extension.configureSoftBodyJellyPreset({ ID: 'gandi-soft' });
  assert.match(extension.softBodySummary({ ID: 'gandi-soft' }), /volume:0.006/);
  extension.configureSoftBodyFoamPreset({ ID: 'gandi-soft' });
  assert.match(extension.softBodySummary({ ID: 'gandi-soft' }), /damping:0.035/);
  extension.configureSoftBodyFirmRubberPreset({ ID: 'gandi-soft' });
  extension.createDistanceJoint({ ID: 'gandi-joint', BODY_A: 'gandi-probe', BODY_B: 'gandi-link', LENGTH: 40 });
  extension.createPointToPointJoint({ ID: 'gandi-ball', BODY_A: 'gandi-probe', BODY_B: 'gandi-link', X: 20, Y: 2, Z: 3 });
  extension.createHingeJoint({ ID: 'gandi-hinge', BODY_A: 'gandi-probe', BODY_B: 'gandi-link', X: 20, Y: 2, Z: 3, AX: 0, AY: 1, AZ: 0 });
  extension.createFixedJoint({ ID: 'gandi-fixed', BODY_A: 'gandi-probe', BODY_B: 'gandi-link', X: 20, Y: 2, Z: 3 });
  extension.configureDistanceJoint({ ID: 'gandi-joint', MIN: 30, MAX: 50, SPRING: 2.5, DAMPING: 0.75 });
  extension.configureHingeJoint({ ID: 'gandi-hinge', LOWER: -35, UPPER: 35, DAMPING: 2.5 });
  extension.configureFixedJoint({ ID: 'gandi-fixed', BREAK_FORCE: 2000, BREAK_TORQUE: 1500 });
  extension.configureHingeMotor({ ID: 'gandi-hinge', SPEED: 120, MAX_TORQUE: 12 });
  extension.configureHingeServo({ ID: 'gandi-hinge', TARGET: 20, MAX_SPEED: 180, MAX_TORQUE: 12 });
  const gandiScene = extension.sceneJson();
  extension.loadSceneJson({ SCENE_JSON: gandiScene });
  extension.sphereCast({ X: 1, Y: 80, Z: 3, RADIUS: 10, DX: 0, DY: -1, DZ: 0, LENGTH: 200 });
  extension.capsuleCast({ X: 1, Y: 80, Z: 3, RADIUS: 8, HALF_HEIGHT: 16, DX: 0, DY: -1, DZ: 0, LENGTH: 200 });
  extension.raycast({ X: 1, Y: 80, Z: 3, DX: 0, DY: -1, DZ: 0, LENGTH: 200 });
  assert.match(extension.queryAabbColliders({ X: 1, Y: 0, Z: 3, HX: 90, HY: 90, HZ: 90 }), /10 colliders/);
  extension.setDebugOverlayLayers({ LAYERS: 'bodies contacts joints' });
  extension.showDebugOverlay();
  extension.moveKinematicCapsule({ ID: 'gandi-player', DX: 4, DY: 0, DZ: 0 });
  extension.stepWorld({ SECONDS: 1 / 30 });
  extension.renderDebugFrame();
  extension.hideDebugOverlay();
  extension.resetDebugOverlayLayers();

  assert.match(extension.worldSummary(), /5 bodies/);
  assert.match(extension.worldSummary(), /1 characters/);
  assert.match(extension.worldSummary(), /1 cloths/);
  assert.match(extension.worldSummary(), /1 soft bodies/);
  assert.match(extension.rigidBodySummary({ ID: 'gandi-probe' }), /gandi-probe/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'gandi-player' }), /radius:5/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'gandi-player' }), /jump:9/);
  assert.match(extension.kinematicCapsuleSummary({ ID: 'gandi-player' }), /air:0.65/);
  assert.match(extension.kinematicGroundSummary({ ID: 'gandi-player' }), /grounded:(yes|no)/);
  assert.match(extension.kinematicGroundCollider({ ID: 'gandi-player' }), /^(gandi-floor:collider|none)$/);
  assert.match(extension.kinematicCapsuleHitSummary({ ID: 'gandi-player' }), /last hit:/);
  assert.match(extension.kinematicBlockingCollider({ ID: 'gandi-player' }), /^(gandi-link:collider|none)$/);
  assert.equal(typeof extension.isKinematicCapsuleGrounded({ ID: 'gandi-player' }), 'boolean');
  assert.match(extension.queryKinematicCapsuleBodyHitEvents({ ID: 'gandi-player', PHASE: 'stay' }), /bodies in stay controller hit events/);
  assert.match(extension.queryKinematicCapsuleColliderHitEvents({ ID: 'gandi-player', PHASE: 'stay' }), /colliders in stay controller hit events/);
  extension.setKinematicCapsuleCrouch({ ID: 'gandi-player', HALF_HEIGHT: 8, RADIUS: 4, SPEED: 15 });
  extension.dropThroughOneWayPlatforms({ ID: 'gandi-player', SECONDS: 0.25 });
  extension.setKinematicCapsuleVerticalVelocity({ ID: 'gandi-player', VELOCITY: 4 });
  assert.match(extension.kinematicCapsuleSummary({ ID: 'gandi-player' }), /vertical velocity:4/);
  extension.launchKinematicCapsule({ ID: 'gandi-player', DX: 2, DY: 6, DZ: 0 });
  assert.match(extension.kinematicCapsuleSummary({ ID: 'gandi-player' }), /vertical velocity:4/);
  extension.jumpKinematicCapsule({ ID: 'gandi-player' });
  assert.match(extension.clothSummary({ ID: 'gandi-cloth' }), /particles:20/);
  assert.match(extension.clothSummary({ ID: 'gandi-cloth' }), /self:on/);
  assert.match(extension.clothSummary({ ID: 'gandi-cloth' }), /damping:0.08/);
  assert.match(extension.clothSummary({ ID: 'gandi-cloth' }), /margin:2.5/);
  assert.match(extension.softBodySummary({ ID: 'gandi-soft' }), /layers:3/);
  assert.match(extension.softBodySummary({ ID: 'gandi-soft' }), /damping:0.045/);
  assert.match(extension.softBodySummary({ ID: 'gandi-soft' }), /volume:0.0003/);
  assert.match(extension.softBodySummary({ ID: 'gandi-soft' }), /dynamic contacts:/);
  assert.match(extension.materialSummary({ ID: 'rubber' }), /restitution:0.8/);
  assert.match(extension.jointSummary({ ID: 'gandi-joint' }), /distance-joint/);
  assert.match(extension.jointSummary({ ID: 'gandi-hinge' }), /motor mode:servo/);
  assert.match(extension.jointSummary({ ID: 'gandi-fixed' }), /break force:/);
  assert.match(extension.queryBodyContacts({ ID: 'gandi-probe' }), /(bodies touching|not found)/);
  assert.match(extension.queryColliderContacts({ ID: 'gandi-probe:collider' }), /(colliders touching|not found)/);
  assert.match(extension.queryBodyTriggerEvents({ ID: 'gandi-probe', PHASE: 'enter' }), /bodies in enter trigger events/);
  assert.match(extension.queryColliderTriggerEvents({ ID: 'gandi-sensor:collider', PHASE: 'enter' }), /colliders in enter trigger events/);
  assert.match(extension.queryBodyContactEvents({ ID: 'gandi-probe', PHASE: 'enter' }), /(bodies in enter contact events|not found)/);
  assert.match(extension.queryColliderContactEvents({ ID: 'gandi-probe:collider', PHASE: 'enter' }), /(colliders in enter contact events|not found)/);
  assert.match(extension.raycastSummary(), /Ray hit/);
  assert.match(extension.shapeCastSummary(), /cast hit/);
  assert.match(extension.debugFrameSummary(), /Gandi Approved frame 1/);
  assert.match(extension.debugOverlaySummary(), /overlay/);
  assert.match(extension.sceneIoSummary(), /Scene loaded/);
  assert.match(extension.contactEventsSummary(), /contact events/);
  assert.match(extension.triggerEventsSummary(), /trigger events/);
  assert.match(extension.controllerHitEventsSummary(), /controller hit events/);
  assert.match(extension.hostSummary(), /Gandi Approved/);
  assert.match(extension.hostSummary(), /runtime:yes/);
  assert.match(extension.hostSummary(), /renderer:yes/);
});

test('both adapters expose identical public block contracts', () => {
  ensureBuild();

  const turboWarpContext = runBundle(
    readBundle('turbowarp-unsandboxed.js'),
    createTurboWarpContext({ unsandboxed: true, withRenderer: false })
  );
  const turboWarpInfo = normalizeRealmValue(turboWarpContext.registrations[0].getInfo());

  const gandiContext = runBundle(
    readBundle('gandi-approved.js'),
    createGandiContext({ withRenderer: false })
  );
  const GandiExtension = gandiContext.window.tempExt.Extension;
  const gandiInfo = normalizeRealmValue(new GandiExtension({ renderer: null }).getInfo());

  assert.deepEqual(turboWarpInfo, gandiInfo);
});
