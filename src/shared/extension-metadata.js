import { getConvexHullPresetIds, getDefaultConvexHullPresetId } from './convex-hull-presets.js';

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
        opcode: 'configureMaterialSurface',
        blockType: 'command',
        text: 'configure material [ID] surface traction:[TRACTION] jump:[JUMP_MULTIPLIER] conveyor x:[CONVEYOR_X] y:[CONVEYOR_Y] z:[CONVEYOR_Z]',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-1' },
          TRACTION: { type: 'number', defaultValue: 1 },
          JUMP_MULTIPLIER: { type: 'number', defaultValue: 1 },
          CONVEYOR_X: { type: 'number', defaultValue: 0 },
          CONVEYOR_Y: { type: 'number', defaultValue: 0 },
          CONVEYOR_Z: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'configureMaterialIcePreset',
        blockType: 'command',
        text: 'configure material [ID] ice preset',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-1' }
        }
      },
      {
        opcode: 'configureMaterialStickyPreset',
        blockType: 'command',
        text: 'configure material [ID] sticky preset',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-1' }
        }
      },
      {
        opcode: 'configureMaterialBouncePadPreset',
        blockType: 'command',
        text: 'configure material [ID] bounce pad preset jump:[JUMP_MULTIPLIER]',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-1' },
          JUMP_MULTIPLIER: { type: 'number', defaultValue: 1.85 }
        }
      },
      {
        opcode: 'configureMaterialConveyorPreset',
        blockType: 'command',
        text: 'configure material [ID] conveyor preset traction:[TRACTION] x:[CONVEYOR_X] y:[CONVEYOR_Y] z:[CONVEYOR_Z]',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-1' },
          TRACTION: { type: 'number', defaultValue: 1 },
          CONVEYOR_X: { type: 'number', defaultValue: 4 },
          CONVEYOR_Y: { type: 'number', defaultValue: 0 },
          CONVEYOR_Z: { type: 'number', defaultValue: 0 }
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
        opcode: 'createKinematicCapsule',
        blockType: 'command',
        text: 'create kinematic capsule [ID] at x:[X] y:[Y] z:[Z] radius:[RADIUS] half height:[HALF_HEIGHT] layer:[LAYER] mask:[MASK]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 40 },
          Z: { type: 'number', defaultValue: 0 },
          RADIUS: { type: 'number', defaultValue: 12 },
          HALF_HEIGHT: { type: 'number', defaultValue: 24 },
          LAYER: { type: 'number', defaultValue: 1 },
          MASK: { type: 'number', defaultValue: 2147483647 }
        }
      },
      {
        opcode: 'configureKinematicCapsule',
        blockType: 'command',
        text: 'configure kinematic capsule [ID] skin:[SKIN] probe:[PROBE] max slope:[MAX_SLOPE]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          SKIN: { type: 'number', defaultValue: 0.5 },
          PROBE: { type: 'number', defaultValue: 4 },
          MAX_SLOPE: { type: 'number', defaultValue: 55 }
        }
      },
      {
        opcode: 'configureKinematicController',
        blockType: 'command',
        text: 'configure kinematic controller [ID] jump:[JUMP_SPEED] gravity:[GRAVITY_SCALE] step:[STEP_OFFSET] snap:[GROUND_SNAP]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          JUMP_SPEED: { type: 'number', defaultValue: 8 },
          GRAVITY_SCALE: { type: 'number', defaultValue: 1 },
          STEP_OFFSET: { type: 'number', defaultValue: 6 },
          GROUND_SNAP: { type: 'number', defaultValue: 2 }
        }
      },
      {
        opcode: 'configureKinematicControllerAdvanced',
        blockType: 'command',
        text: 'configure kinematic controller advanced [ID] air:[AIR_CONTROL] coyote:[COYOTE] buffer:[BUFFER] platforms [PLATFORMS]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          AIR_CONTROL: { type: 'number', defaultValue: 1 },
          COYOTE: { type: 'number', defaultValue: 0.1 },
          BUFFER: { type: 'number', defaultValue: 0.1 },
          PLATFORMS: { type: 'string', menu: 'ON_OFF', defaultValue: 'on' }
        }
      },
      {
        opcode: 'setKinematicCapsuleMoveIntent',
        blockType: 'command',
        text: 'set kinematic capsule [ID] move intent x:[DX] y:[DY] z:[DZ]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          DX: { type: 'number', defaultValue: 0 },
          DY: { type: 'number', defaultValue: 0 },
          DZ: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'setKinematicCapsuleCrouch',
        blockType: 'command',
        text: 'set kinematic capsule [ID] crouch half height [HALF_HEIGHT] radius [RADIUS] speed [SPEED]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          HALF_HEIGHT: { type: 'number', defaultValue: 12 },
          RADIUS: { type: 'number', defaultValue: 6 },
          SPEED: { type: 'number', defaultValue: 20 }
        }
      },
      {
        opcode: 'setKinematicCapsuleVerticalVelocity',
        blockType: 'command',
        text: 'set kinematic capsule [ID] vertical velocity [VELOCITY]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          VELOCITY: { type: 'number', defaultValue: 10 }
        }
      },
      {
        opcode: 'jumpKinematicCapsule',
        blockType: 'command',
        text: 'jump kinematic capsule [ID]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'dropThroughOneWayPlatforms',
        blockType: 'command',
        text: 'drop through one-way platforms for kinematic capsule [ID] duration [SECONDS]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          SECONDS: { type: 'number', defaultValue: 0.2 }
        }
      },
      {
        opcode: 'launchKinematicCapsule',
        blockType: 'command',
        text: 'launch kinematic capsule [ID] x:[DX] y:[DY] z:[DZ]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          DX: { type: 'number', defaultValue: 0 },
          DY: { type: 'number', defaultValue: 12 },
          DZ: { type: 'number', defaultValue: 0 }
        }
      },
      {
        opcode: 'moveKinematicCapsule',
        blockType: 'command',
        text: 'move kinematic capsule [ID] by x:[DX] y:[DY] z:[DZ]',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' },
          DX: { type: 'number', defaultValue: 10 },
          DY: { type: 'number', defaultValue: 0 },
          DZ: { type: 'number', defaultValue: 0 }
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
        opcode: 'createPresetConvexHullRigidBody',
        blockType: 'command',
        text: 'create preset convex hull rigid body [ID] preset [PRESET] at x:[X] y:[Y] z:[Z] scale:[SCALE] mass:[MASS] material:[MATERIAL]',
        arguments: {
          ID: { type: 'string', defaultValue: 'preset-hull-1' },
          PRESET: { type: 'string', menu: 'CONVEX_HULL_PRESETS', defaultValue: getDefaultConvexHullPresetId() },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          SCALE: { type: 'number', defaultValue: 100 },
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
        opcode: 'createStaticBoxOneWayPlatform',
        blockType: 'command',
        text: 'create static box one-way platform [ID] at x:[X] y:[Y] z:[Z] size:[SIZE] material:[MATERIAL]',
        arguments: {
          ID: { type: 'string', defaultValue: 'one-way-box-1' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          SIZE: { type: 'number', defaultValue: 100 },
          MATERIAL: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'createStaticBoxSensor',
        blockType: 'command',
        text: 'create static box sensor [ID] at x:[X] y:[Y] z:[Z] size:[SIZE] layer:[LAYER] mask:[MASK]',
        arguments: {
          ID: { type: 'string', defaultValue: 'sensor-1' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          SIZE: { type: 'number', defaultValue: 100 },
          LAYER: { type: 'number', defaultValue: 2 },
          MASK: { type: 'number', defaultValue: 2147483647 }
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
        opcode: 'createStaticConvexHullOneWayPlatform',
        blockType: 'command',
        text: 'create static convex hull one-way platform [ID] vertices:[VERTICES] at x:[X] y:[Y] z:[Z] material:[MATERIAL]',
        arguments: {
          ID: { type: 'string', defaultValue: 'one-way-hull-1' },
          VERTICES: { type: 'string', defaultValue: '-50 -50 -50; 50 -50 -50; 50 -50 50; -50 -50 50; 0 50 0' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          MATERIAL: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'createStaticConvexHullSensor',
        blockType: 'command',
        text: 'create static convex hull sensor [ID] vertices:[VERTICES] at x:[X] y:[Y] z:[Z] layer:[LAYER] mask:[MASK]',
        arguments: {
          ID: { type: 'string', defaultValue: 'sensor-hull-1' },
          VERTICES: { type: 'string', defaultValue: '-50 -50 -50; 50 -50 -50; 50 -50 50; -50 -50 50; 0 50 0' },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          LAYER: { type: 'number', defaultValue: 2 },
          MASK: { type: 'number', defaultValue: 2147483647 }
        }
      },
      {
        opcode: 'createPresetStaticConvexHullCollider',
        blockType: 'command',
        text: 'create preset static convex hull collider [ID] preset [PRESET] at x:[X] y:[Y] z:[Z] scale:[SCALE] material:[MATERIAL]',
        arguments: {
          ID: { type: 'string', defaultValue: 'preset-collider-1' },
          PRESET: { type: 'string', menu: 'CONVEX_HULL_PRESETS', defaultValue: getDefaultConvexHullPresetId() },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: -100 },
          Z: { type: 'number', defaultValue: 0 },
          SCALE: { type: 'number', defaultValue: 100 },
          MATERIAL: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'createPresetStaticConvexHullSensor',
        blockType: 'command',
        text: 'create preset static convex hull sensor [ID] preset [PRESET] at x:[X] y:[Y] z:[Z] scale:[SCALE] layer:[LAYER] mask:[MASK]',
        arguments: {
          ID: { type: 'string', defaultValue: 'preset-sensor-1' },
          PRESET: { type: 'string', menu: 'CONVEX_HULL_PRESETS', defaultValue: getDefaultConvexHullPresetId() },
          X: { type: 'number', defaultValue: 0 },
          Y: { type: 'number', defaultValue: 0 },
          Z: { type: 'number', defaultValue: 0 },
          SCALE: { type: 'number', defaultValue: 100 },
          LAYER: { type: 'number', defaultValue: 2 },
          MASK: { type: 'number', defaultValue: 2147483647 }
        }
      },
      {
        opcode: 'convexHullPresetVertices',
        blockType: 'reporter',
        text: 'convex hull preset [PRESET] vertices scale:[SCALE]',
        arguments: {
          PRESET: { type: 'string', menu: 'CONVEX_HULL_PRESETS', defaultValue: getDefaultConvexHullPresetId() },
          SCALE: { type: 'number', defaultValue: 100 }
        }
      },
      {
        opcode: 'createClothSheet',
        blockType: 'command',
        text: 'create cloth sheet [ID] rows:[ROWS] columns:[COLUMNS] spacing:[SPACING] at x:[X] y:[Y] z:[Z] pin [PIN_MODE]',
        arguments: {
          ID: { type: 'string', defaultValue: 'cloth-1' },
          ROWS: { type: 'number', defaultValue: 6 },
          COLUMNS: { type: 'number', defaultValue: 8 },
          SPACING: { type: 'number', defaultValue: 20 },
          X: { type: 'number', defaultValue: -70 },
          Y: { type: 'number', defaultValue: 120 },
          Z: { type: 'number', defaultValue: 0 },
          PIN_MODE: { type: 'string', menu: 'CLOTH_PIN_MODES', defaultValue: 'top-row' }
        }
      },
      {
        opcode: 'createSoftBodyCube',
        blockType: 'command',
        text: 'create soft body cube [ID] rows:[ROWS] columns:[COLUMNS] layers:[LAYERS] spacing:[SPACING] at x:[X] y:[Y] z:[Z] pin [PIN_MODE]',
        arguments: {
          ID: { type: 'string', defaultValue: 'soft-1' },
          ROWS: { type: 'number', defaultValue: 4 },
          COLUMNS: { type: 'number', defaultValue: 4 },
          LAYERS: { type: 'number', defaultValue: 4 },
          SPACING: { type: 'number', defaultValue: 20 },
          X: { type: 'number', defaultValue: -30 },
          Y: { type: 'number', defaultValue: 120 },
          Z: { type: 'number', defaultValue: -30 },
          PIN_MODE: { type: 'string', menu: 'SOFT_BODY_PIN_MODES', defaultValue: 'none' }
        }
      },
      {
        opcode: 'configureCloth',
        blockType: 'command',
        text: 'configure cloth [ID] damping:[DAMPING] margin:[MARGIN] stretch:[STRETCH] shear:[SHEAR] bend:[BEND] self collision [SELF_COLLISION] thickness:[SELF_DISTANCE]',
        arguments: {
          ID: { type: 'string', defaultValue: 'cloth-1' },
          DAMPING: { type: 'number', defaultValue: 0.03 },
          MARGIN: { type: 'number', defaultValue: 3 },
          STRETCH: { type: 'number', defaultValue: 0 },
          SHEAR: { type: 'number', defaultValue: 0.00015 },
          BEND: { type: 'number', defaultValue: 0.0005 },
          SELF_COLLISION: { type: 'string', menu: 'ON_OFF', defaultValue: 'off' },
          SELF_DISTANCE: { type: 'number', defaultValue: 8 }
        }
      },
      {
        opcode: 'configureSoftBody',
        blockType: 'command',
        text: 'configure soft body [ID] damping:[DAMPING] margin:[MARGIN] stretch:[STRETCH] shear:[SHEAR] bend:[BEND] volume:[VOLUME]',
        arguments: {
          ID: { type: 'string', defaultValue: 'soft-1' },
          DAMPING: { type: 'number', defaultValue: 0.04 },
          MARGIN: { type: 'number', defaultValue: 3 },
          STRETCH: { type: 'number', defaultValue: 0 },
          SHEAR: { type: 'number', defaultValue: 0.0002 },
          BEND: { type: 'number', defaultValue: 0.0008 },
          VOLUME: { type: 'number', defaultValue: 0.00008 }
        }
      },
      {
        opcode: 'configureSoftBodyJellyPreset',
        blockType: 'command',
        text: 'configure soft body [ID] as jelly',
        arguments: {
          ID: { type: 'string', defaultValue: 'soft-1' }
        }
      },
      {
        opcode: 'configureSoftBodyFoamPreset',
        blockType: 'command',
        text: 'configure soft body [ID] as foam',
        arguments: {
          ID: { type: 'string', defaultValue: 'soft-1' }
        }
      },
      {
        opcode: 'configureSoftBodyFirmRubberPreset',
        blockType: 'command',
        text: 'configure soft body [ID] as firm rubber',
        arguments: {
          ID: { type: 'string', defaultValue: 'soft-1' }
        }
      },
      {
        opcode: 'configureBodyCollision',
        blockType: 'command',
        text: 'configure rigid body [ID] collision layer:[LAYER] mask:[MASK]',
        arguments: {
          ID: { type: 'string', defaultValue: 'body-1' },
          LAYER: { type: 'number', defaultValue: 1 },
          MASK: { type: 'number', defaultValue: 2147483647 }
        }
      },
      {
        opcode: 'configureColliderCollision',
        blockType: 'command',
        text: 'configure collider [ID] layer:[LAYER] mask:[MASK] sensor [SENSOR]',
        arguments: {
          ID: { type: 'string', defaultValue: 'body-1:collider' },
          LAYER: { type: 'number', defaultValue: 1 },
          MASK: { type: 'number', defaultValue: 2147483647 },
          SENSOR: { type: 'string', menu: 'ON_OFF', defaultValue: 'off' }
        }
      },
      {
        opcode: 'configureColliderOneWay',
        blockType: 'command',
        text: 'configure collider [ID] one-way [ONE_WAY]',
        arguments: {
          ID: { type: 'string', defaultValue: 'one-way-box-1:collider' },
          ONE_WAY: { type: 'string', menu: 'ON_OFF', defaultValue: 'on' }
        }
      },
      {
        opcode: 'configureClothLightPreset',
        blockType: 'command',
        text: 'configure cloth [ID] as light fabric',
        arguments: {
          ID: { type: 'string', defaultValue: 'cloth-1' }
        }
      },
      {
        opcode: 'configureClothHeavyPreset',
        blockType: 'command',
        text: 'configure cloth [ID] as heavy cloth',
        arguments: {
          ID: { type: 'string', defaultValue: 'cloth-1' }
        }
      },
      {
        opcode: 'configureClothWrinklePreset',
        blockType: 'command',
        text: 'configure cloth [ID] as wrinkle debug cloth',
        arguments: {
          ID: { type: 'string', defaultValue: 'cloth-1' }
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
        opcode: 'loadSceneJson',
        blockType: 'command',
        text: 'load physics scene json [SCENE_JSON]',
        arguments: {
          SCENE_JSON: { type: 'string', defaultValue: '{}' }
        }
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
        opcode: 'setDebugOverlayLayers',
        blockType: 'command',
        text: 'set debug overlay layers [LAYERS]',
        arguments: {
          LAYERS: { type: 'string', defaultValue: 'bodies static contacts joints queries' }
        }
      },
      {
        opcode: 'resetDebugOverlayLayers',
        blockType: 'command',
        text: 'reset debug overlay layers'
      },
      {
        opcode: 'worldSummary',
        blockType: 'reporter',
        text: 'physics world summary'
      },
      {
        opcode: 'sceneJson',
        blockType: 'reporter',
        text: 'scene json'
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
        opcode: 'kinematicCapsuleSummary',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicGroundSummary',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] ground summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicGroundCollider',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] ground collider',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicGroundBody',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] ground body',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicGroundMaterial',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] ground material',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicGroundAngle',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] ground angle',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicGroundNormal',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] ground normal',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicSupportBody',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] support body',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicSupportMaterial',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] support material',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicSupportVelocity',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] support velocity',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicSupportTraction',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] support traction',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicSupportJumpMultiplier',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] support jump multiplier',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicSupportConveyorVelocity',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] support conveyor velocity',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicCapsuleHitSummary',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] hit summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'kinematicBlockingCollider',
        blockType: 'reporter',
        text: 'kinematic capsule [ID] blocking collider',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'isKinematicCapsuleGrounded',
        blockType: 'boolean',
        text: 'kinematic capsule [ID] grounded?',
        arguments: {
          ID: { type: 'string', defaultValue: 'player-1' }
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
        opcode: 'materialSurfaceTraction',
        blockType: 'reporter',
        text: 'material [ID] surface traction',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'materialSurfaceJumpMultiplier',
        blockType: 'reporter',
        text: 'material [ID] surface jump multiplier',
        arguments: {
          ID: { type: 'string', defaultValue: 'material-default' }
        }
      },
      {
        opcode: 'materialConveyorVelocity',
        blockType: 'reporter',
        text: 'material [ID] conveyor velocity',
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
        opcode: 'clothSummary',
        blockType: 'reporter',
        text: 'cloth [ID] summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'cloth-1' }
        }
      },
      {
        opcode: 'softBodySummary',
        blockType: 'reporter',
        text: 'soft body [ID] summary',
        arguments: {
          ID: { type: 'string', defaultValue: 'soft-1' }
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
        opcode: 'queryBodyContacts',
        blockType: 'reporter',
        text: 'bodies touching body [ID]',
        arguments: {
          ID: { type: 'string', defaultValue: 'body-1' }
        }
      },
      {
        opcode: 'queryColliderContacts',
        blockType: 'reporter',
        text: 'colliders touching collider [ID]',
        arguments: {
          ID: { type: 'string', defaultValue: 'body-1:collider' }
        }
      },
      {
        opcode: 'queryBodyContactEvents',
        blockType: 'reporter',
        text: 'bodies in [PHASE] contact events for body [ID]',
        arguments: {
          PHASE: { type: 'string', menu: 'EVENT_PHASES', defaultValue: 'stay' },
          ID: { type: 'string', defaultValue: 'body-1' }
        }
      },
      {
        opcode: 'queryBodyTriggerEvents',
        blockType: 'reporter',
        text: 'bodies in [PHASE] trigger events for body [ID]',
        arguments: {
          PHASE: { type: 'string', menu: 'EVENT_PHASES', defaultValue: 'stay' },
          ID: { type: 'string', defaultValue: 'body-1' }
        }
      },
      {
        opcode: 'queryKinematicCapsuleBodyHitEvents',
        blockType: 'reporter',
        text: 'bodies in [PHASE] controller hit events for kinematic capsule [ID]',
        arguments: {
          PHASE: { type: 'string', menu: 'EVENT_PHASES', defaultValue: 'stay' },
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'queryKinematicCapsuleColliderHitEvents',
        blockType: 'reporter',
        text: 'colliders in [PHASE] controller hit events for kinematic capsule [ID]',
        arguments: {
          PHASE: { type: 'string', menu: 'EVENT_PHASES', defaultValue: 'stay' },
          ID: { type: 'string', defaultValue: 'player-1' }
        }
      },
      {
        opcode: 'queryColliderContactEvents',
        blockType: 'reporter',
        text: 'colliders in [PHASE] contact events for collider [ID]',
        arguments: {
          PHASE: { type: 'string', menu: 'EVENT_PHASES', defaultValue: 'stay' },
          ID: { type: 'string', defaultValue: 'body-1:collider' }
        }
      },
      {
        opcode: 'queryColliderTriggerEvents',
        blockType: 'reporter',
        text: 'colliders in [PHASE] trigger events for collider [ID]',
        arguments: {
          PHASE: { type: 'string', menu: 'EVENT_PHASES', defaultValue: 'stay' },
          ID: { type: 'string', defaultValue: 'body-1:collider' }
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
        opcode: 'sceneIoSummary',
        blockType: 'reporter',
        text: 'scene io summary'
      },
      {
        opcode: 'contactEventsSummary',
        blockType: 'reporter',
        text: 'contact events summary'
      },
      {
        opcode: 'triggerEventsSummary',
        blockType: 'reporter',
        text: 'trigger events summary'
      },
      {
        opcode: 'controllerHitEventsSummary',
        blockType: 'reporter',
        text: 'controller hit events summary'
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
    ],
    menus: {
      CONVEX_HULL_PRESETS: {
        acceptReporters: true,
        items: getConvexHullPresetIds()
      },
      CLOTH_PIN_MODES: {
        acceptReporters: true,
        items: ['top-row', 'top-corners', 'corners', 'left-edge', 'right-edge', 'none']
      },
      SOFT_BODY_PIN_MODES: {
        acceptReporters: true,
        items: ['none', 'top-layer', 'top-corners', 'corners']
      },
      ON_OFF: {
        acceptReporters: true,
        items: ['off', 'on']
      },
      EVENT_PHASES: {
        acceptReporters: true,
        items: ['enter', 'stay', 'exit']
      }
    }
  };
}
