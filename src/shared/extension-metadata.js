export const EXTENSION_ID = 'engine3d';

export const GANDI_EXTENSION_METADATA = {
  name: 'engine3d.meta.name',
  description: 'engine3d.meta.description',
  extensionId: EXTENSION_ID,
  collaborator: 'Codex scaffold',
  featured: false
};

export const GANDI_L10N = {
  en: {
    'engine3d.meta.name': 'Physics Engine Kit',
    'engine3d.meta.description': 'Shared 3D physics engine scaffold for Gandi and TurboWarp.'
  },
  'zh-cn': {
    'engine3d.meta.name': 'Physics Engine Kit',
    'engine3d.meta.description': 'Shared 3D physics engine scaffold for Gandi and TurboWarp.'
  }
};

export function createExtensionInfo() {
  return {
    id: EXTENSION_ID,
    name: 'Physics Engine Kit',
    color1: '#0B84FF',
    color2: '#0861C5',
    color3: '#0A3A73',
    blocks: [
      {
        opcode: 'resetWorld',
        blockType: 'command',
        text: 'reset physics world'
      },
      {
        opcode: 'setGravity',
        blockType: 'command',
        text: 'set gravity x:[X] y:[Y] z:[Z]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: -9.81 },
          Z: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'createMaterial',
        blockType: 'command',
        text: 'create material [ID] friction:[FRICTION] restitution:[RESTITUTION] density:[DENSITY]',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-1' },
          FRICTION: { type: 'number', defaultValue: 0.5 },
          RESTITUTION: { type: 'number', defaultValue: 0 },
          DENSITY: { type: 'number', defaultValue: 1 }
        }
      },
      {
        opcode: 'setCameraPosition',
        blockType: 'command',
        text: 'set debug camera position x:[X] y:[Y] z:[Z]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 400 }
        }
      },
      {
        opcode: 'setCameraTarget',
        blockType: 'command',
        text: 'set debug camera target pitch:[X] yaw:[Y] roll:[Z]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'createBoxRigidBody',
        blockType: 'command',
        text: 'create box rigid body [ID] at x:[X] y:[Y] z:[Z] size:[SIZE] mass:[MASS] material:[MATERIAL]',
        arguments: {
          ID: { type: 'string', defaultValue: 'body-1' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          SIZE: { type: 'number', defaultValue: 100 },
          MASS: { type: 'number', defaultValue: 1 },
          MATERIAL: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'createConvexHullRigidBody',
        blockType: 'command',
        text: 'create convex hull rigid body [ID] vertices:[VERTICES] at x:[X] y:[Y] z:[Z] mass:[MASS] material:[MATERIAL]',
        arguments: {
          ID: { type: 'string', defaultValue: 'hull-1' },
          VERTICES: { type: 'string', defaultValue: '-50 -50 -50; 50 -50 -50; 50 -50 50; -50 -50 50; 0 50 0' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          MASS: { type: 'number', defaultValue: 1 },
          MATERIAL: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'createStaticBoxCollider',
        blockType: 'command',
        text: 'create static box collider [ID] at x:[X] y:[Y] z:[Z] size:[SIZE] material:[MATERIAL]',
        arguments: {
          ID: { type: 'string', defaultValue: 'collider-1' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: -100 },
          Z: { type: 'number', defaultValue: 0 },
          SIZE: { type: 'number', defaultValue: 100 },
          MATERIAL: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'createStaticConvexHullCollider',
        blockType: 'command',
        text: 'create static convex hull collider [ID] vertices:[VERTICES] at x:[X] y:[Y] z:[Z] material:[MATERIAL]',
        arguments: {
          ID: { type: 'string', defaultValue: 'hull-collider-1' },
          VERTICES: { type: 'string', defaultValue: '-50 -50 -50; 50 -50 -50; 50 -50 50; -50 -50 50; 0 50 0' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: -100 },
          Z: { type: 'number', defaultValue: 0 },
          MATERIAL: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'createDistanceJoint',
        blockType: 'command',
        text: 'create distance joint [ID] body a:[BODY_A] body b:[BODY_B] length:[LENGTH]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-1' },
          BODY_A: { type: 'string', defaultValue: 'body-1' },
          BODY_B: { type: 'string', defaultValue: 'body-2' },
          LENGTH: { type: 'number', defaultValue: 100 }
        }
      },
      {
        opcode: 'createPointToPointJoint',
        blockType: 'command',
        text: 'create point-to-point joint [ID] body a:[BODY_A] body b:[BODY_B] anchor x:[X] y:[Y] z:[Z]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-2' },
          BODY_A: { type: 'string', defaultValue: 'body-1' },
          BODY_B: { type: 'string', defaultValue: 'body-2' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'createHingeJoint',
        blockType: 'command',
        text: 'create hinge joint [ID] body a:[BODY_A] body b:[BODY_B] anchor x:[X] y:[Y] z:[Z] axis x:[AX] y:[AY] z:[AZ]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-3' },
          BODY_A: { type: 'string', defaultValue: 'body-1' },
          BODY_B: { type: 'string', defaultValue: 'body-2' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          AX: { type: 'number', defaultValue: 0 },
          AY: { type: 'number', defaultValue: 1 },
          AZ: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'createFixedJoint',
        blockType: 'command',
        text: 'create fixed joint [ID] body a:[BODY_A] body b:[BODY_B] anchor x:[X] y:[Y] z:[Z]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-4' },
          BODY_A: { type: 'string', defaultValue: 'body-1' },
          BODY_B: { type: 'string', defaultValue: 'body-2' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'configureDistanceJoint',
        blockType: 'command',
        text: 'configure distance joint [ID] min:[MIN] max:[MAX] spring:[SPRING] damping:[DAMPING]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-1' },
          MIN: { type: 'number', defaultValue: 50 },
          MAX: { type: 'number', defaultValue: 150 },
          SPRING: { type: 'number', defaultValue: 0 },
          DAMPING: { type: 'number', defaultValue: 0.5 }
        }
      },
      {
        opcode: 'configureHingeJoint',
        blockType: 'command',
        text: 'configure hinge joint [ID] lower:[LOWER] upper:[UPPER] damping:[DAMPING]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-3' },
          LOWER: { type: 'number', defaultValue: -45 },
          UPPER: { type: 'number', defaultValue: 45 },
          DAMPING: { type: 'number', defaultValue: 4 }
        }
      },
      {
        opcode: 'configureFixedJoint',
        blockType: 'command',
        text: 'configure fixed joint [ID] break force:[BREAK_FORCE] break torque:[BREAK_TORQUE]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-4' },
          BREAK_FORCE: { type: 'number', defaultValue: 0 },
          BREAK_TORQUE: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'configureHingeMotor',
        blockType: 'command',
        text: 'configure hinge motor [ID] speed:[SPEED] max torque:[MAX_TORQUE]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-3' },
          SPEED: { type: 'number', defaultValue: 180 },
          MAX_TORQUE: { type: 'number', defaultValue: 20 }
        }
      },
      {
        opcode: 'configureHingeServo',
        blockType: 'command',
        text: 'configure hinge servo [ID] target angle:[TARGET] max speed:[MAX_SPEED] max torque:[MAX_TORQUE]',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-3' },
          TARGET: { type: 'number', defaultValue: 45 },
          MAX_SPEED: { type: 'number', defaultValue: 180 },
          MAX_TORQUE: { type: 'number', defaultValue: 20 }
        }
      },
      {
        opcode: 'stepWorld',
        blockType: 'command',
        text: 'step physics world by [SECONDS] seconds',
        arguments: {
          SECONDS: { type: 'number', defaultValue: 0.016666666666666666 }
        }
      },
      {
        opcode: 'sphereCast',
        blockType: 'command',
        text: 'sphere cast from x:[X] y:[Y] z:[Z] radius:[RADIUS] dir x:[DX] y:[DY] z:[DZ] length:[LENGTH]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 120 },
          Z: { type: 'number', defaultValue: 0 },
          RADIUS: { type: 'number', defaultValue: 25 },
          DX: { type: 'number', defaultValue: 0 },
          DY: { type: 'number', defaultValue: -1 },
          DZ: { type: 'number', defaultValue: 0 },
          LENGTH: { type: 'number', defaultValue: 300 }
        }
      },
      {
        opcode: 'capsuleCast',
        blockType: 'command',
        text: 'capsule cast from x:[X] y:[Y] z:[Z] radius:[RADIUS] half height:[HALF_HEIGHT] dir x:[DX] y:[DY] z:[DZ] length:[LENGTH]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 120 },
          Z: { type: 'number', defaultValue: 0 },
          RADIUS: { type: 'number', defaultValue: 20 },
          HALF_HEIGHT: { type: 'number', defaultValue: 40 },
          DX: { type: 'number', defaultValue: 0 },
          DY: { type: 'number', defaultValue: -1 },
          DZ: { type: 'number', defaultValue: 0 },
          LENGTH: { type: 'number', defaultValue: 300 }
        }
      },
      {
        opcode: 'raycast',
        blockType: 'command',
        text: 'raycast from x:[X] y:[Y] z:[Z] dir x:[DX] y:[DY] z:[DZ] length:[LENGTH]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 100 },
          Z: { type: 'number', defaultValue: 0 },
          DX: { type: 'number', defaultValue: 0 },
          DY: { type: 'number', defaultValue: -1 },
          DZ: { type: 'number', defaultValue: 0 },
          LENGTH: { type: 'number', defaultValue: 300 }
        }
      },
      {
        opcode: 'renderDebugFrame',
        blockType: 'command',
        text: 'render debug frame'
      },
      {
        opcode: 'showDebugOverlay',
        blockType: 'command',
        text: 'show debug overlay'
      },
      {
        opcode: 'hideDebugOverlay',
        blockType: 'command',
        text: 'hide debug overlay'
      },
      {
        opcode: 'worldSummary',
        blockType: 'reporter',
        text: 'physics world summary'
      },
      {
        opcode: 'rigidBodySummary',
        blockType: 'reporter',
        text: 'rigid body [ID] summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'body-1' }
        }
      },
      {
        opcode: 'colliderSummary',
        blockType: 'reporter',
        text: 'collider [ID] summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'collider-1:collider' }
        }
      },
      {
        opcode: 'materialSummary',
        blockType: 'reporter',
        text: 'material [ID] summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'jointSummary',
        blockType: 'reporter',
        text: 'joint [ID] summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'joint-1' }
        }
      },
      {
        opcode: 'queryPointBodies',
        blockType: 'reporter',
        text: 'bodies at point x:[X] y:[Y] z:[Z]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'queryPointColliders',
        blockType: 'reporter',
        text: 'colliders at point x:[X] y:[Y] z:[Z]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'queryAabbBodies',
        blockType: 'reporter',
        text: 'bodies in box center x:[X] y:[Y] z:[Z] half x:[HX] y:[HY] z:[HZ]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          HX: { type: 'number', defaultValue: 50 },
          HY: { type: 'number', defaultValue: 50 },
          HZ: { type: 'number', defaultValue: 50 }
        }
      },
      {
        opcode: 'queryAabbColliders',
        blockType: 'reporter',
        text: 'colliders in box center x:[X] y:[Y] z:[Z] half x:[HX] y:[HY] z:[HZ]',
        arguments: {
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          HX: { type: 'number', defaultValue: 50 },
          HY: { type: 'number', defaultValue: 50 },
          HZ: { type: 'number', defaultValue: 50 }
        }
      },
      {
        opcode: 'raycastSummary',
        blockType: 'reporter',
        text: 'last raycast summary'
      },
      {
        opcode: 'shapeCastSummary',
        blockType: 'reporter',
        text: 'last shape cast summary'
      },
      {
        opcode: 'ccdSummary',
        blockType: 'reporter',
        text: 'ccd summary'
      },
      {
        opcode: 'debugFrameSummary',
        blockType: 'reporter',
        text: 'debug frame summary'
      },
      {
        opcode: 'debugOverlaySummary',
        blockType: 'reporter',
        text: 'debug overlay summary'
      },
      {
        opcode: 'hostSummary',
        blockType: 'reporter',
        text: 'host summary'
      },
      {
        opcode: 'resetScene',
        blockType: 'command',
        text: 'reset 3D scene',
        hideFromPalette: true
      },
      {
        opcode: 'addCube',
        blockType: 'command',
        text: 'add cube [ID] at x:[X] y:[Y] z:[Z] size:[SIZE]',
        hideFromPalette: true,
        arguments: {
          ID: { type: 'string', defaultValue: 'cube-1' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          SIZE: { type: 'number', defaultValue: 100 }
        }
      },
      {
        opcode: 'sceneSummary',
        blockType: 'reporter',
        text: 'scene summary',
        hideFromPalette: true
      },
      {
        opcode: 'lastFrameSummary',
        blockType: 'reporter',
        text: 'last frame summary',
        hideFromPalette: true
      }
    ]
  };
}
