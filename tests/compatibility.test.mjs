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
  'createBoxRigidBody',
  'createStaticBoxCollider',
  'stepWorld',
  'renderDebugFrame',
  'showDebugOverlay',
  'hideDebugOverlay',
  'worldSummary',
  'rigidBodySummary',
  'colliderSummary',
  'materialSummary',
  'queryPointBodies',
  'queryPointColliders',
  'queryAabbBodies',
  'queryAabbColliders',
  'debugFrameSummary',
  'debugOverlaySummary',
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

  assert.equal(info.id, 'engine3d');
  assertHasRequiredOpcodes(info);

  extension.resetWorld();
  extension.createMaterial({ ID: 'ice', FRICTION: 0.05, RESTITUTION: 0.2, DENSITY: 0.9 });
  extension.setGravity({ X: 0, Y: -10, Z: 0 });
  extension.setCameraPosition({ X: 10, Y: 20, Z: 300 });
  extension.createStaticBoxCollider({ ID: 'floor', X: 0, Y: -10, Z: 0, SIZE: 100, MATERIAL: 'ice' });
  extension.createBoxRigidBody({ ID: 'probe', X: 5, Y: 6, Z: 7, SIZE: 42, MASS: 2, MATERIAL: 'ice' });
  extension.showDebugOverlay();
  assert.match(extension.queryPointBodies({ X: 5, Y: 6, Z: 7 }), /1 bodies/);
  assert.match(extension.queryAabbColliders({ X: 0, Y: 0, Z: 0, HX: 100, HY: 100, HZ: 100 }), /2 colliders/);
  extension.stepWorld({ SECONDS: 1 / 60 });
  extension.renderDebugFrame();
  extension.hideDebugOverlay();

  assert.match(extension.worldSummary(), /1 bodies/);
  assert.match(extension.colliderSummary({ ID: 'floor:collider' }), /body:static/);
  assert.match(extension.materialSummary({ ID: 'ice' }), /friction:0.05/);
  assert.match(extension.debugFrameSummary(), /TurboWarp frame 1/);
  assert.match(extension.debugOverlaySummary(), /overlay/);
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

  assert.equal(info.id, 'engine3d');
  assertHasRequiredOpcodes(info);
  extension.resetWorld();
  extension.createStaticBoxCollider({ ID: 'remote-floor', X: 0, Y: -5, Z: 0, SIZE: 10, MATERIAL: 'material-default' });
  extension.createBoxRigidBody({ ID: 'remote-probe', X: 0, Y: 0, Z: 0, SIZE: 10, MASS: 1, MATERIAL: 'material-default' });
  extension.showDebugOverlay();
  extension.renderDebugFrame();
  extension.hideDebugOverlay();

  assert.match(extension.worldSummary(), /1 bodies/);
  assert.match(extension.queryPointColliders({ X: 0, Y: 0, Z: 0 }), /2 colliders/);
  assert.match(extension.debugFrameSummary(), /Gandi Remote frame 1/);
  assert.match(extension.debugOverlaySummary(), /overlay/);
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
  extension.createStaticBoxCollider({ ID: 'gandi-floor', X: 1, Y: -10, Z: 3, SIZE: 64, MATERIAL: 'rubber' });
  extension.createBoxRigidBody({ ID: 'gandi-probe', X: 1, Y: 2, Z: 3, SIZE: 64, MASS: 3, MATERIAL: 'rubber' });
  assert.match(extension.queryAabbColliders({ X: 1, Y: 0, Z: 3, HX: 40, HY: 40, HZ: 40 }), /2 colliders/);
  extension.showDebugOverlay();
  extension.stepWorld({ SECONDS: 1 / 30 });
  extension.renderDebugFrame();
  extension.hideDebugOverlay();

  assert.match(extension.worldSummary(), /1 bodies/);
  assert.match(extension.rigidBodySummary({ ID: 'gandi-probe' }), /gandi-probe/);
  assert.match(extension.materialSummary({ ID: 'rubber' }), /restitution:0.8/);
  assert.match(extension.debugFrameSummary(), /Gandi Approved frame 1/);
  assert.match(extension.debugOverlaySummary(), /overlay/);
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
