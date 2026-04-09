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
  return info.blocks.map((block) => block.opcode);
}

function normalizeRealmValue(value) {
  return JSON.parse(JSON.stringify(value));
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
  assert.deepEqual(normalizeRealmValue(getOpcodes(info)), [
    'resetScene',
    'setCameraPosition',
    'addCube',
    'renderDebugFrame',
    'sceneSummary',
    'lastFrameSummary',
    'hostSummary'
  ]);

  extension.resetScene();
  extension.setCameraPosition({ X: 10, Y: 20, Z: 300 });
  extension.addCube({ ID: 'probe', X: 5, Y: 6, Z: 7, SIZE: 42 });
  extension.renderDebugFrame();

  assert.match(extension.sceneSummary(), /1 objects/);
  assert.match(extension.lastFrameSummary(), /TurboWarp frame 1/);
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
  extension.resetScene();
  extension.addCube({ ID: 'remote-probe', X: 0, Y: 0, Z: 0, SIZE: 10 });
  extension.renderDebugFrame();

  assert.match(extension.sceneSummary(), /1 objects/);
  assert.match(extension.lastFrameSummary(), /Gandi Remote frame 1/);
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
  assert.deepEqual(normalizeRealmValue(getOpcodes(info)), [
    'resetScene',
    'setCameraPosition',
    'addCube',
    'renderDebugFrame',
    'sceneSummary',
    'lastFrameSummary',
    'hostSummary'
  ]);

  extension.resetScene();
  extension.setCameraPosition({ X: -12, Y: 0, Z: 512 });
  extension.addCube({ ID: 'gandi-probe', X: 1, Y: 2, Z: 3, SIZE: 64 });
  extension.renderDebugFrame();

  assert.match(extension.sceneSummary(), /1 objects/);
  assert.match(extension.lastFrameSummary(), /Gandi Approved frame 1/);
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
