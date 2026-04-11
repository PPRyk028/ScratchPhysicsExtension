import { addScaledVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, negateVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';
import { createTangentBasis, getShapeSupportFeature, getShapeWorldCenter } from './support.js';

const GJK_MAX_ITERATIONS = 24;
const EPA_MAX_ITERATIONS = 32;
const EPA_TOLERANCE = 1e-4;

function createSupportVertex(shapeA, poseA, shapeB, poseB, direction) {
  const supportFeatureA = getShapeSupportFeature(shapeA, poseA, direction);
  const supportFeatureB = getShapeSupportFeature(shapeB, poseB, negateVec3(direction));
  const supportA = supportFeatureA.worldPoint;
  const supportB = supportFeatureB.worldPoint;
  return {
    supportA,
    supportB,
    supportFeatureA,
    supportFeatureB,
    point: subtractVec3(supportA, supportB)
  };
}

function tripleCross(a, b, c) {
  return crossVec3(crossVec3(a, b), c);
}

function choosePerpendicular(vector) {
  const axis = Math.abs(vector.x) < 0.7 ? createVec3(1, 0, 0) : createVec3(0, 1, 0);
  return normalizeVec3(crossVec3(vector, axis), createVec3(0, 0, 1));
}

function computeBarycentric(point, a, b, c) {
  const v0 = subtractVec3(b, a);
  const v1 = subtractVec3(c, a);
  const v2 = subtractVec3(point, a);
  const d00 = dotVec3(v0, v0);
  const d01 = dotVec3(v0, v1);
  const d11 = dotVec3(v1, v1);
  const d20 = dotVec3(v2, v0);
  const d21 = dotVec3(v2, v1);
  const denominator = d00 * d11 - d01 * d01;

  if (Math.abs(denominator) <= 1e-12) {
    return { u: 1 / 3, v: 1 / 3, w: 1 / 3 };
  }

  const v = (d11 * d20 - d01 * d21) / denominator;
  const w = (d00 * d21 - d01 * d20) / denominator;
  const u = 1 - v - w;
  const clamped = {
    u: Math.max(0, u),
    v: Math.max(0, v),
    w: Math.max(0, w)
  };
  const sum = clamped.u + clamped.v + clamped.w;
  if (sum <= 1e-12) {
    return { u: 1 / 3, v: 1 / 3, w: 1 / 3 };
  }

  return {
    u: clamped.u / sum,
    v: clamped.v / sum,
    w: clamped.w / sum
  };
}

function blendSupportVertex(faceVertices, weights, key) {
  let result = createVec3();
  result = addScaledVec3(result, faceVertices[0][key], weights.u);
  result = addScaledVec3(result, faceVertices[1][key], weights.v);
  result = addScaledVec3(result, faceVertices[2][key], weights.w);
  return result;
}

function sameDirection(direction, toward) {
  return dotVec3(direction, toward) > 1e-8;
}

function updateSimplex(simplex) {
  const a = simplex[0];
  const ao = negateVec3(a.point);

  if (simplex.length === 2) {
    const b = simplex[1];
    const ab = subtractVec3(b.point, a.point);
    if (sameDirection(ab, ao)) {
      let direction = tripleCross(ab, ao, ab);
      if (lengthSquaredVec3(direction) <= 1e-12) {
        direction = choosePerpendicular(ab);
      }

      return {
        hit: false,
        direction
      };
    }

    simplex.splice(1, 1);
    return {
      hit: false,
      direction: ao
    };
  }

  if (simplex.length === 3) {
    const b = simplex[1];
    const c = simplex[2];
    const ab = subtractVec3(b.point, a.point);
    const ac = subtractVec3(c.point, a.point);
    const abc = crossVec3(ab, ac);
    const acPerp = crossVec3(abc, ac);

    if (sameDirection(acPerp, ao)) {
      if (sameDirection(ac, ao)) {
        simplex.splice(1, 1);
        let direction = tripleCross(ac, ao, ac);
        if (lengthSquaredVec3(direction) <= 1e-12) {
          direction = choosePerpendicular(ac);
        }

        return {
          hit: false,
          direction
        };
      }

      simplex.splice(2, 1);
      return updateSimplex(simplex);
    }

    const abPerp = crossVec3(ab, abc);
    if (sameDirection(abPerp, ao)) {
      simplex.splice(2, 1);
      return updateSimplex(simplex);
    }

    if (sameDirection(abc, ao)) {
      return {
        hit: false,
        direction: abc
      };
    }

    simplex[1] = c;
    simplex[2] = b;
    return {
      hit: false,
      direction: negateVec3(abc)
    };
  }

  if (simplex.length === 4) {
    const b = simplex[1];
    const c = simplex[2];
    const d = simplex[3];
    const ab = subtractVec3(b.point, a.point);
    const ac = subtractVec3(c.point, a.point);
    const ad = subtractVec3(d.point, a.point);
    const abc = crossVec3(ab, ac);
    const acd = crossVec3(ac, ad);
    const adb = crossVec3(ad, ab);

    if (sameDirection(abc, ao)) {
      simplex.splice(3, 1);
      return {
        hit: false,
        direction: abc
      };
    }

    if (sameDirection(acd, ao)) {
      simplex[1] = c;
      simplex[2] = d;
      simplex.splice(3, 1);
      return {
        hit: false,
        direction: acd
      };
    }

    if (sameDirection(adb, ao)) {
      simplex[1] = d;
      simplex[2] = b;
      simplex.splice(3, 1);
      return {
        hit: false,
        direction: adb
      };
    }

    return {
      hit: true,
      direction: createVec3()
    };
  }

  return {
    hit: false,
    direction: ao
  };
}

function createFace(a, b, c, vertices) {
  const pointA = vertices[a].point;
  const pointB = vertices[b].point;
  const pointC = vertices[c].point;
  let normal = crossVec3(subtractVec3(pointB, pointA), subtractVec3(pointC, pointA));
  if (lengthSquaredVec3(normal) <= 1e-12) {
    return null;
  }

  normal = normalizeVec3(normal, createVec3(0, 1, 0));
  let distance = dotVec3(normal, pointA);
  if (distance < 0) {
    normal = scaleVec3(normal, -1);
    distance = -distance;
    return {
      a,
      b: c,
      c: b,
      normal,
      distance
    };
  }

  return {
    a,
    b,
    c,
    normal,
    distance
  };
}

function addHorizonEdge(edges, start, end) {
  const reverseKey = `${end}|${start}`;
  if (edges.has(reverseKey)) {
    edges.delete(reverseKey);
    return;
  }

  edges.set(`${start}|${end}`, { start, end });
}

function buildContactFromFace(face, vertices) {
  const closestPoint = scaleVec3(face.normal, face.distance);
  const faceVertices = [vertices[face.a], vertices[face.b], vertices[face.c]];
  const weights = computeBarycentric(closestPoint, faceVertices[0].point, faceVertices[1].point, faceVertices[2].point);
  const supportA = blendSupportVertex(faceVertices, weights, 'supportA');
  const supportB = blendSupportVertex(faceVertices, weights, 'supportB');

  return {
    normal: face.normal,
    penetration: Math.max(face.distance, 0),
    supportA,
    supportB,
    faceVertices,
    contactPosition: createVec3(
      (supportA.x + supportB.x) / 2,
      (supportA.y + supportB.y) / 2,
      (supportA.z + supportB.z) / 2
    )
  };
}

function createManifoldCandidate(featureA, featureB, normal, fallbackFeatureId = 'feature') {
  const featureIdA = String(featureA?.featureId ?? 'shape-a');
  const featureIdB = String(featureB?.featureId ?? 'shape-b');
  const separation = dotVec3(subtractVec3(featureB.worldPoint, featureA.worldPoint), normal);
  return {
    featureId: `${featureIdA}|${featureIdB}|${fallbackFeatureId}`,
    position: createVec3(
      (featureA.worldPoint.x + featureB.worldPoint.x) / 2,
      (featureA.worldPoint.y + featureB.worldPoint.y) / 2,
      (featureA.worldPoint.z + featureB.worldPoint.z) / 2
    ),
    penetration: Math.max(0, -separation),
    separation
  };
}

function dedupeCandidates(candidates) {
  const uniqueCandidates = [];
  const seenFeatureIds = new Set();
  const seenPoints = new Set();

  for (const candidate of candidates) {
    const featureId = String(candidate.featureId ?? '').trim();
    const pointKey = `${candidate.position.x.toFixed(5)}|${candidate.position.y.toFixed(5)}|${candidate.position.z.toFixed(5)}`;
    if (seenFeatureIds.has(featureId) || seenPoints.has(pointKey)) {
      continue;
    }

    seenFeatureIds.add(featureId);
    seenPoints.add(pointKey);
    uniqueCandidates.push(candidate);
  }

  return uniqueCandidates;
}

function collectFaceCandidates(faceVertices, normal) {
  return faceVertices.map((vertex, index) => createManifoldCandidate(
    vertex.supportFeatureA,
    vertex.supportFeatureB,
    normal,
    `face:${index}`
  ));
}

function collectSampledCandidates(shapeA, poseA, shapeB, poseB, normal) {
  const basis = createTangentBasis(normal);
  const sampleDirections = [
    normal,
    normalizeVec3(addScaledVec3(normal, basis.tangentA, 0.35), normal),
    normalizeVec3(addScaledVec3(normal, basis.tangentA, -0.35), normal),
    normalizeVec3(addScaledVec3(normal, basis.tangentB, 0.35), normal),
    normalizeVec3(addScaledVec3(normal, basis.tangentB, -0.35), normal),
    normalizeVec3(addScaledVec3(addScaledVec3(normal, basis.tangentA, 0.25), basis.tangentB, 0.25), normal),
    normalizeVec3(addScaledVec3(addScaledVec3(normal, basis.tangentA, 0.25), basis.tangentB, -0.25), normal),
    normalizeVec3(addScaledVec3(addScaledVec3(normal, basis.tangentA, -0.25), basis.tangentB, 0.25), normal),
    normalizeVec3(addScaledVec3(addScaledVec3(normal, basis.tangentA, -0.25), basis.tangentB, -0.25), normal)
  ];

  return sampleDirections.map((direction, index) => {
    const featureA = getShapeSupportFeature(shapeA, poseA, direction);
    const featureB = getShapeSupportFeature(shapeB, poseB, negateVec3(direction));
    return createManifoldCandidate(featureA, featureB, normal, `sample:${index}`);
  });
}

function selectManifoldCandidates(candidates, normal, maxContacts = 4) {
  const uniqueCandidates = dedupeCandidates(candidates).sort((left, right) => right.penetration - left.penetration);
  if (uniqueCandidates.length <= maxContacts) {
    return uniqueCandidates;
  }

  const basis = createTangentBasis(normal);
  const selected = [];
  const selectedKeys = new Set();

  function addCandidate(candidate) {
    if (!candidate || selectedKeys.has(candidate.featureId)) {
      return;
    }

    selected.push(candidate);
    selectedKeys.add(candidate.featureId);
  }

  addCandidate(uniqueCandidates[0]);

  const extremalDirections = [
    basis.tangentA,
    negateVec3(basis.tangentA),
    basis.tangentB,
    negateVec3(basis.tangentB)
  ];

  for (const direction of extremalDirections) {
    const candidate = uniqueCandidates.reduce((best, current) => {
      if (selectedKeys.has(current.featureId)) {
        return best;
      }

      if (!best) {
        return current;
      }

      return dotVec3(current.position, direction) > dotVec3(best.position, direction) ? current : best;
    }, null);
    addCandidate(candidate);
    if (selected.length >= maxContacts) {
      return selected.slice(0, maxContacts);
    }
  }

  while (selected.length < maxContacts) {
    const candidate = uniqueCandidates.find((current) => !selectedKeys.has(current.featureId));
    if (!candidate) {
      break;
    }

    addCandidate(candidate);
  }

  return selected.slice(0, maxContacts);
}

export function buildConvexContactManifold(shapeA, poseA, shapeB, poseB, epaResult) {
  if (!epaResult || epaResult.penetration <= 0) {
    return [];
  }

  const faceCandidates = collectFaceCandidates(epaResult.faceVertices ?? [], epaResult.normal);
  const sampledCandidates = collectSampledCandidates(shapeA, poseA, shapeB, poseB, epaResult.normal);
  const deepestPenetration = Math.max(epaResult.penetration, 1e-6);
  const minimumPenetration = Math.max(1e-5, deepestPenetration * 0.2);
  const filteredCandidates = [...faceCandidates, ...sampledCandidates]
    .filter((candidate) => candidate.penetration >= minimumPenetration);

  return selectManifoldCandidates(filteredCandidates, epaResult.normal, 4);
}

export function runGjk(shapeA, poseA, shapeB, poseB, options = {}) {
  const maxIterations = Math.max(4, Number(options.maxIterations ?? GJK_MAX_ITERATIONS));
  let direction = subtractVec3(getShapeWorldCenter(shapeB, poseB), getShapeWorldCenter(shapeA, poseA));
  if (lengthSquaredVec3(direction) <= 1e-12) {
    direction = createVec3(1, 0, 0);
  }

  const simplex = [createSupportVertex(shapeA, poseA, shapeB, poseB, direction)];
  direction = negateVec3(simplex[0].point);

  for (let iteration = 0; iteration < maxIterations; iteration += 1) {
    if (lengthSquaredVec3(direction) <= 1e-12) {
      direction = createVec3(0, 1, 0);
    }

    const support = createSupportVertex(shapeA, poseA, shapeB, poseB, direction);
    if (dotVec3(support.point, direction) <= 1e-8) {
      return {
        hit: false,
        simplex,
        iterations: iteration + 1
      };
    }

    simplex.unshift(support);
    const updated = updateSimplex(simplex);
    direction = updated.direction;

    if (updated.hit) {
      return {
        hit: true,
        simplex,
        iterations: iteration + 1
      };
    }
  }

  return {
    hit: false,
    simplex,
    iterations: maxIterations
  };
}

export function runEpa(shapeA, poseA, shapeB, poseB, simplex, options = {}) {
  if (!Array.isArray(simplex) || simplex.length < 4) {
    return null;
  }

  const tolerance = Number(options.tolerance ?? EPA_TOLERANCE);
  const maxIterations = Math.max(8, Number(options.maxIterations ?? EPA_MAX_ITERATIONS));
  const vertices = simplex.slice(0, 4).map((vertex) => ({
    supportA: vertex.supportA,
    supportB: vertex.supportB,
    supportFeatureA: vertex.supportFeatureA,
    supportFeatureB: vertex.supportFeatureB,
    point: vertex.point
  }));
  let faces = [
    createFace(0, 1, 2, vertices),
    createFace(0, 3, 1, vertices),
    createFace(0, 2, 3, vertices),
    createFace(1, 3, 2, vertices)
  ].filter(Boolean);

  for (let iteration = 0; iteration < maxIterations; iteration += 1) {
    faces.sort((left, right) => left.distance - right.distance);
    const closestFace = faces[0];
    if (!closestFace) {
      return null;
    }

    const support = createSupportVertex(shapeA, poseA, shapeB, poseB, closestFace.normal);
    const supportDistance = dotVec3(support.point, closestFace.normal);

    if (supportDistance - closestFace.distance <= tolerance) {
      return {
        ...buildContactFromFace(closestFace, vertices),
        closestFace,
        vertices,
        iterations: iteration + 1
      };
    }

    const newVertexIndex = vertices.length;
    vertices.push(support);

    const horizonEdges = new Map();
    const remainingFaces = [];

    for (const face of faces) {
      const visible = dotVec3(face.normal, subtractVec3(support.point, vertices[face.a].point)) > tolerance;
      if (!visible) {
        remainingFaces.push(face);
        continue;
      }

      addHorizonEdge(horizonEdges, face.a, face.b);
      addHorizonEdge(horizonEdges, face.b, face.c);
      addHorizonEdge(horizonEdges, face.c, face.a);
    }

    faces = remainingFaces;
    for (const edge of horizonEdges.values()) {
      const face = createFace(edge.start, edge.end, newVertexIndex, vertices);
      if (face) {
        faces.push(face);
      }
    }
  }

  faces.sort((left, right) => left.distance - right.distance);
  if (!faces[0]) {
    return null;
  }

  return {
    ...buildContactFromFace(faces[0], vertices),
    closestFace: faces[0],
    vertices,
    iterations: maxIterations
  };
}

export function collideConvexPairWithGjkEpa(shapeA, poseA, shapeB, poseB, options = {}) {
  const gjk = runGjk(shapeA, poseA, shapeB, poseB, options);
  if (!gjk.hit) {
    return null;
  }

  return runEpa(shapeA, poseA, shapeB, poseB, gjk.simplex, options);
}
