import { cloneQuat, createIdentityQuat } from '../math/quat.js';
import { cloneVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';
import { BaseRegistry } from './base-registry.js';

const HULL_PLANE_EPSILON = 1e-5;

function toPositiveNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
}

function cloneVertices(vertices) {
  return Array.isArray(vertices) ? vertices.map((vertex) => cloneVec3(vertex)) : [];
}

function cloneEdges(edges) {
  return Array.isArray(edges)
    ? edges.map((edge) => ({
      startIndex: edge.startIndex,
      endIndex: edge.endIndex
    }))
    : [];
}

function cloneFaces(faces) {
  return Array.isArray(faces)
    ? faces.map((face) => ({
      normal: cloneVec3(face.normal),
      planeOffset: Number(face.planeOffset ?? 0),
      boundaryIndices: Array.isArray(face.boundaryIndices) ? face.boundaryIndices.slice() : [],
      edges: cloneEdges(face.edges)
    }))
    : [];
}

function createPlaneKey(normal, offset) {
  let sign = 1;
  if (
    normal.x < -HULL_PLANE_EPSILON ||
    (Math.abs(normal.x) <= HULL_PLANE_EPSILON && normal.y < -HULL_PLANE_EPSILON) ||
    (Math.abs(normal.x) <= HULL_PLANE_EPSILON && Math.abs(normal.y) <= HULL_PLANE_EPSILON && normal.z < -HULL_PLANE_EPSILON)
  ) {
    sign = -1;
  }

  const resolvedNormal = sign === 1 ? normal : scaleVec3(normal, -1);
  const resolvedOffset = sign === 1 ? offset : -offset;
  return `${resolvedNormal.x.toFixed(5)}|${resolvedNormal.y.toFixed(5)}|${resolvedNormal.z.toFixed(5)}|${resolvedOffset.toFixed(5)}`;
}

function choosePlaneBasis(normal) {
  const helperAxis = Math.abs(normal.y) < 0.9 ? createVec3(0, 1, 0) : createVec3(1, 0, 0);
  const tangentA = normalizeVec3(crossVec3(helperAxis, normal), createVec3(1, 0, 0));
  const tangentB = normalizeVec3(crossVec3(normal, tangentA), createVec3(0, 0, 1));
  return { tangentA, tangentB };
}

function computeFaceLoop(indices, vertices, normal) {
  const uniqueIndices = Array.from(new Set(indices));
  if (uniqueIndices.length < 2) {
    return [];
  }

  if (uniqueIndices.length === 2) {
    return uniqueIndices.slice();
  }

  const centroid = uniqueIndices.reduce((sum, index) => ({
    x: sum.x + vertices[index].x,
    y: sum.y + vertices[index].y,
    z: sum.z + vertices[index].z
  }), createVec3());
  centroid.x /= uniqueIndices.length;
  centroid.y /= uniqueIndices.length;
  centroid.z /= uniqueIndices.length;

  const basis = choosePlaneBasis(normal);
  const ordered = uniqueIndices
    .map((index) => {
      const relative = subtractVec3(vertices[index], centroid);
      return {
        index,
        angle: Math.atan2(
          dotVec3(relative, basis.tangentB),
          dotVec3(relative, basis.tangentA)
        )
      };
    })
    .sort((left, right) => left.angle - right.angle);

  return ordered.map((entry) => entry.index);
}

function computeFaceBoundary(indices, vertices, normal) {
  const loop = computeFaceLoop(indices, vertices, normal);
  if (loop.length < 2) {
    return [];
  }

  if (loop.length === 2) {
    return [{
      startIndex: Math.min(loop[0], loop[1]),
      endIndex: Math.max(loop[0], loop[1])
    }];
  }

  const edges = [];
  for (let index = 0; index < loop.length; index += 1) {
    const startIndex = loop[index];
    const endIndex = loop[(index + 1) % loop.length];
    if (startIndex === endIndex) {
      continue;
    }

    edges.push({
      startIndex: Math.min(startIndex, endIndex),
      endIndex: Math.max(startIndex, endIndex)
    });
  }

  return edges;
}

function computeConvexHullFaces(vertices) {
  if (!Array.isArray(vertices) || vertices.length < 2) {
    return [];
  }

  if (vertices.length === 2) {
    const edge = { startIndex: 0, endIndex: 1 };
    return [{
      normal: createVec3(1, 0, 0),
      planeOffset: dotVec3(createVec3(1, 0, 0), vertices[0]),
      boundaryIndices: [0, 1],
      edges: [edge]
    }];
  }

  const faces = new Map();
  for (let indexA = 0; indexA < vertices.length - 2; indexA += 1) {
    for (let indexB = indexA + 1; indexB < vertices.length - 1; indexB += 1) {
      for (let indexC = indexB + 1; indexC < vertices.length; indexC += 1) {
        const pointA = vertices[indexA];
        const pointB = vertices[indexB];
        const pointC = vertices[indexC];
        let normal = crossVec3(subtractVec3(pointB, pointA), subtractVec3(pointC, pointA));
        if (lengthSquaredVec3(normal) <= 1e-10) {
          continue;
        }

        normal = normalizeVec3(normal, createVec3(0, 1, 0));
        let offset = dotVec3(normal, pointA);
        let positiveCount = 0;
        let negativeCount = 0;
        const coplanarIndices = [indexA, indexB, indexC];

        for (let testIndex = 0; testIndex < vertices.length; testIndex += 1) {
          if (testIndex === indexA || testIndex === indexB || testIndex === indexC) {
            continue;
          }

          const distance = dotVec3(normal, vertices[testIndex]) - offset;
          if (distance > HULL_PLANE_EPSILON) {
            positiveCount += 1;
          } else if (distance < -HULL_PLANE_EPSILON) {
            negativeCount += 1;
          } else {
            coplanarIndices.push(testIndex);
          }

          if (positiveCount > 0 && negativeCount > 0) {
            break;
          }
        }

        if (positiveCount > 0 && negativeCount > 0) {
          continue;
        }

        if (positiveCount > 0) {
          normal = scaleVec3(normal, -1);
          offset = -offset;
        }

        const key = createPlaneKey(normal, offset);
        const existing = faces.get(key);
        if (existing) {
          existing.indices.push(...coplanarIndices);
          continue;
        }

        faces.set(key, {
          normal,
          planeOffset: offset,
          indices: coplanarIndices.slice()
        });
      }
    }
  }

  return Array.from(faces.values(), (face) => ({
    normal: cloneVec3(face.normal),
    planeOffset: Number(face.planeOffset ?? 0),
    boundaryIndices: computeFaceLoop(face.indices, vertices, face.normal),
    edges: computeFaceBoundary(face.indices, vertices, face.normal)
  })).filter((face) => face.boundaryIndices.length >= 2);
}

function computeConvexHullDebugEdges(vertices, faces) {
  if (!Array.isArray(vertices) || vertices.length < 2) {
    return [];
  }

  if (vertices.length === 2) {
    return [{ startIndex: 0, endIndex: 1 }];
  }

  const edgeKeys = new Set();
  const edges = [];
  for (const face of Array.isArray(faces) ? faces : []) {
    for (const edge of face.edges ?? []) {
      const key = `${edge.startIndex}|${edge.endIndex}`;
      if (edgeKeys.has(key)) {
        continue;
      }

      edgeKeys.add(key);
      edges.push(edge);
    }
  }

  return edges;
}

function createLocalPose(localPose) {
  return {
    position: cloneVec3(localPose?.position ?? createVec3()),
    rotation: cloneQuat(localPose?.rotation ?? createIdentityQuat())
  };
}

function cloneGeometry(shape) {
  if (shape.type === 'box') {
    return {
      halfExtents: cloneVec3(shape.geometry.halfExtents)
    };
  }

  if (shape.type === 'sphere') {
    return {
      radius: shape.geometry.radius
    };
  }

  if (shape.type === 'capsule') {
    return {
      radius: shape.geometry.radius,
      halfHeight: shape.geometry.halfHeight
    };
  }

  if (shape.type === 'convex-hull') {
    return {
      vertices: cloneVertices(shape.geometry.vertices),
      faces: cloneFaces(shape.geometry.faces),
      debugEdges: cloneEdges(shape.geometry.debugEdges)
    };
  }

  return { ...shape.geometry };
}

export class ShapeRegistry extends BaseRegistry {
  constructor() {
    super('shape');
  }

  cloneRecord(shape) {
    return {
      id: shape.id,
      type: shape.type,
      geometry: cloneGeometry(shape),
      localPose: createLocalPose(shape.localPose),
      userData: shape.userData ?? null
    };
  }

  createBoxShape(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      type: 'box',
      geometry: {
        halfExtents: cloneVec3(options.halfExtents ?? createVec3(0.5, 0.5, 0.5))
      },
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }

  createSphereShape(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      type: 'sphere',
      geometry: {
        radius: toPositiveNumber(options.radius, 0.5)
      },
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }

  createCapsuleShape(options = {}) {
    return this.store({
      id: this.allocateId(options.id),
      type: 'capsule',
      geometry: {
        radius: toPositiveNumber(options.radius, 0.5),
        halfHeight: toPositiveNumber(options.halfHeight, 0.5)
      },
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }

  createConvexHullShape(options = {}) {
    const vertices = cloneVertices(options.vertices);
    const faces = computeConvexHullFaces(vertices);
    return this.store({
      id: this.allocateId(options.id),
      type: 'convex-hull',
      geometry: {
        vertices,
        faces,
        debugEdges: computeConvexHullDebugEdges(vertices, faces)
      },
      localPose: createLocalPose(options.localPose),
      userData: options.userData ?? null
    });
  }
}
