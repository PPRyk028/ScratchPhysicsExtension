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
    'engine3d.meta.name': '3D Engine Kit',
    'engine3d.meta.description': 'Shared 3D engine scaffold for Gandi and TurboWarp.'
  },
  'zh-cn': {
    'engine3d.meta.name': '3D 引擎脚手架',
    'engine3d.meta.description': '面向 Gandi 与 TurboWarp 的共享 3D 引擎扩展骨架。'
  }
};

export function createExtensionInfo() {
  return {
    id: EXTENSION_ID,
    name: '3D Engine Kit',
    color1: '#0B84FF',
    color2: '#0861C5',
    color3: '#0A3A73',
    blocks: [
      {
        opcode: 'resetScene',
        blockType: 'command',
        text: 'reset 3D scene'
      },
      {
        opcode: 'setCameraPosition',
        blockType: 'command',
        text: 'set camera position x:[X] y:[Y] z:[Z]',
        arguments: {
          X: {
            type: 'number',
            defaultValue: 0
          },
          Y: {
            type: 'number',
            defaultValue: 0
          },
          Z: {
            type: 'number',
            defaultValue: 400
          }
        }
      },
      {
        opcode: 'addCube',
        blockType: 'command',
        text: 'add cube [ID] at x:[X] y:[Y] z:[Z] size:[SIZE]',
        arguments: {
          ID: {
            type: 'string',
            defaultValue: 'cube-1'
          },
          X: {
            type: 'number',
            defaultValue: 0
          },
          Y: {
            type: 'number',
            defaultValue: 0
          },
          Z: {
            type: 'number',
            defaultValue: 0
          },
          SIZE: {
            type: 'number',
            defaultValue: 100
          }
        }
      },
      {
        opcode: 'renderDebugFrame',
        blockType: 'command',
        text: 'render debug frame'
      },
      {
        opcode: 'sceneSummary',
        blockType: 'reporter',
        text: 'scene summary'
      },
      {
        opcode: 'lastFrameSummary',
        blockType: 'reporter',
        text: 'last frame summary'
      },
      {
        opcode: 'hostSummary',
        blockType: 'reporter',
        text: 'host summary'
      }
    ]
  };
}

