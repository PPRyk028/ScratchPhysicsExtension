import { rotateVec3ByQuat } from '../math/quat.js';
import { addScaledVec3, createVec3, crossVec3, dotVec3, lengthSquaredVec3, negateVec3, normalizeVec3, scaleVec3, subtractVec3 } from '../math/vec3.js';
import { createTangentBasis, getShapeSupportFeature, getShapeSupportPolygon, getShapeWorldCenter, getShapeWorldPose, transformPointByPose } from './support.js';

const GJK_MAX_ITERATIONS = 24;
const EPA_MAX_ITERATIONS = 32;
const EPA_TOLERANCE = 1e-4;
const MANIFOLD_CLIP_EPSILON = 1e-5;
const FACE_CLIP_EPSILON = 1e-4;
const REFERENCE_FACE_SWITCH_EPSILON = 1e-3;
const FACE_SUPPORT_PLANE_EPSILON = 0.25;

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

function reduceOrderedPolygonCandidates(candidates, normal, maxContacts = 4) {
  const uniqueCandidates = dedupeCandidates(candidates);
  if (uniqueCandidates.length <= maxContacts) {
    return uniqueCandidates;
  }

  const basis = createTangentBasis(normal);
  const projected = uniqueCandidates.map((candidate, index) => ({
    index,
    candidate,
    u: dotVec3(candidate.position, basis.tangentA),
    v: dotVec3(candidate.position, basis.tangentB)
  }));
  const centroid = projected.reduce((sum, point) => ({
    u: sum.u + point.u,
    v: sum.v + point.v
  }), { u: 0, v: 0 });
  centroid.u /= projected.length;
  centroid.v /= projected.length;

  const ordered = projected
    .map((point) => ({
      ...point,
      angle: Math.atan2(point.v - centroid.v, point.u - centroid.u),
      radiusSquared: (point.u - centroid.u) * (point.u - centroid.u) + (point.v - centroid.v) * (point.v - centroid.v)
    }))
    .sort((left, right) => left.angle - right.angle);

  let startIndex = 0;
  for (let index = 1; index < ordered.length; index += 1) {
    if (ordered[index].radiusSquared > ordered[startIndex].radiusSquared + 1e-8) {
      startIndex = index;
      continue;
    }

    if (
      Math.abs(ordered[index].radiusSquared - ordered[startIndex].radiusSquared) <= 1e-8 &&
      ordered[index].candidate.penetration > ordered[startIndex].candidate.penetration
    ) {
      startIndex = index;
    }
  }

  const selected = [];
  const selectedIndices = new Set();
  const step = ordered.length / maxContacts;

  for (let slot = 0; slot < maxContacts; slot += 1) {
    const orderedIndex = Math.floor((startIndex + slot * step) % ordered.length);
    if (selectedIndices.has(orderedIndex)) {
      continue;
    }

    selectedIndices.add(orderedIndex);
    selected.push(ordered[orderedIndex].candidate);
  }

  if (selected.length < maxContacts) {
    const remaining = ordered
      .filter((point, index) => !selectedIndices.has(index))
      .sort((left, right) => {
        if (right.radiusSquared !== left.radiusSquared) {
          return right.radiusSquared - left.radiusSquared;
        }

        return right.candidate.penetration - left.candidate.penetration;
      });

    for (const point of remaining) {
      if (selected.length >= maxContacts) {
        break;
      }

      selected.push(point.candidate);
    }
  }

  return selected.slice(0, maxContacts);
}

function projectSupportPolygon(points, basis) {
  return points.map((point, index) => ({
    index,
    u: dotVec3(point.worldPoint, basis.tangentA),
    v: dotVec3(point.worldPoint, basis.tangentB),
    featureId: point.featureId ?? `feature:${index}`
  }));
}

function polygonSignedArea(polygon) {
  if (!Array.isArray(polygon) || polygon.length < 3) {
    return 0;
  }

  let area = 0;
  for (let index = 0; index < polygon.length; index += 1) {
    const current = polygon[index];
    const next = polygon[(index + 1) % polygon.length];
    area += current.u * next.v - next.u * current.v;
  }

  return area / 2;
}

function orderProjectedPolygon(polygon) {
  if (!Array.isArray(polygon) || polygon.length <= 2) {
    return Array.isArray(polygon) ? polygon.slice() : [];
  }

  const centroid = polygon.reduce((sum, point) => ({
    u: sum.u + point.u,
    v: sum.v + point.v
  }), { u: 0, v: 0 });
  centroid.u /= polygon.length;
  centroid.v /= polygon.length;

  const ordered = polygon
    .slice()
    .sort((left, right) => {
      const leftAngle = Math.atan2(left.v - centroid.v, left.u - centroid.u);
      const rightAngle = Math.atan2(right.v - centroid.v, right.u - centroid.u);
      return leftAngle - rightAngle;
    });

  if (polygonSignedArea(ordered) < 0) {
    ordered.reverse();
  }

  return ordered;
}

function isPointInsideConvexPolygon(point, polygon) {
  if (!Array.isArray(polygon) || polygon.length === 0) {
    return false;
  }

  if (polygon.length === 1) {
    return Math.abs(point.u - polygon[0].u) <= MANIFOLD_CLIP_EPSILON &&
      Math.abs(point.v - polygon[0].v) <= MANIFOLD_CLIP_EPSILON;
  }

  if (polygon.length === 2) {
    const edge = { u: polygon[1].u - polygon[0].u, v: polygon[1].v - polygon[0].v };
    const fromStart = { u: point.u - polygon[0].u, v: point.v - polygon[0].v };
    const cross = edge.u * fromStart.v - edge.v * fromStart.u;
    if (Math.abs(cross) > MANIFOLD_CLIP_EPSILON) {
      return false;
    }

    const dot = fromStart.u * edge.u + fromStart.v * edge.v;
    const edgeLengthSquared = edge.u * edge.u + edge.v * edge.v;
    return dot >= -MANIFOLD_CLIP_EPSILON && dot <= edgeLengthSquared + MANIFOLD_CLIP_EPSILON;
  }

  for (let index = 0; index < polygon.length; index += 1) {
    const start = polygon[index];
    const end = polygon[(index + 1) % polygon.length];
    const edgeU = end.u - start.u;
    const edgeV = end.v - start.v;
    const pointU = point.u - start.u;
    const pointV = point.v - start.v;
    const cross = edgeU * pointV - edgeV * pointU;
    if (cross < -MANIFOLD_CLIP_EPSILON) {
      return false;
    }
  }

  return true;
}

function intersectProjectedSegments(startA, endA, startB, endB) {
  const r = { u: endA.u - startA.u, v: endA.v - startA.v };
  const s = { u: endB.u - startB.u, v: endB.v - startB.v };
  const denominator = r.u * s.v - r.v * s.u;
  const delta = { u: startB.u - startA.u, v: startB.v - startA.v };

  if (Math.abs(denominator) <= MANIFOLD_CLIP_EPSILON) {
    return null;
  }

  const t = (delta.u * s.v - delta.v * s.u) / denominator;
  const u = (delta.u * r.v - delta.v * r.u) / denominator;
  if (t < -MANIFOLD_CLIP_EPSILON || t > 1 + MANIFOLD_CLIP_EPSILON || u < -MANIFOLD_CLIP_EPSILON || u > 1 + MANIFOLD_CLIP_EPSILON) {
    return null;
  }

  return {
    u: startA.u + r.u * t,
    v: startA.v + r.v * t
  };
}

function dedupeProjectedPoints(points) {
  const uniquePoints = [];
  const seen = new Set();

  for (const point of points) {
    const key = `${point.u.toFixed(5)}|${point.v.toFixed(5)}`;
    if (seen.has(key)) {
      continue;
    }

    seen.add(key);
    uniquePoints.push(point);
  }

  return uniquePoints;
}

function isPointOnProjectedSegment(point, start, end) {
  const edgeU = end.u - start.u;
  const edgeV = end.v - start.v;
  const pointU = point.u - start.u;
  const pointV = point.v - start.v;
  const cross = edgeU * pointV - edgeV * pointU;
  if (Math.abs(cross) > MANIFOLD_CLIP_EPSILON) {
    return false;
  }

  const dot = pointU * edgeU + pointV * edgeV;
  const edgeLengthSquared = edgeU * edgeU + edgeV * edgeV;
  return dot >= -MANIFOLD_CLIP_EPSILON && dot <= edgeLengthSquared + MANIFOLD_CLIP_EPSILON;
}

function sortFeatureIds(featureIds) {
  return featureIds
    .map((featureId) => String(featureId ?? '').trim())
    .filter(Boolean)
    .sort();
}

function describeProjectedFeatureAtPoint(point, polygon) {
  if (!Array.isArray(polygon) || polygon.length === 0) {
    return 'feature:empty';
  }

  for (const vertex of polygon) {
    if (Math.abs(point.u - vertex.u) <= MANIFOLD_CLIP_EPSILON && Math.abs(point.v - vertex.v) <= MANIFOLD_CLIP_EPSILON) {
      return `vertex:${vertex.featureId ?? 'unknown'}`;
    }
  }

  const matchedEdges = [];
  for (let index = 0; index < polygon.length; index += 1) {
    const start = polygon[index];
    const end = polygon[(index + 1) % polygon.length];
    if (!isPointOnProjectedSegment(point, start, end)) {
      continue;
    }

    const edgeFeatureId = sortFeatureIds([start.featureId, end.featureId]).join('&');
    if (edgeFeatureId) {
      matchedEdges.push(`edge:${edgeFeatureId}`);
    }
  }

  if (matchedEdges.length > 0) {
    return matchedEdges.sort().join('+');
  }

  const faceFeatureId = sortFeatureIds(polygon.map((vertex) => vertex.featureId)).join('&');
  return faceFeatureId ? `face:${faceFeatureId}` : 'feature:face';
}

function createFeaturePoint(position, featureId) {
  return {
    position,
    featureId: String(featureId ?? '').trim() || 'feature:unknown'
  };
}

function getBoxWorldAxes(pose) {
  const rotation = pose?.rotation;
  return [
    normalizeVec3(rotateVec3ByQuat(rotation, createVec3(1, 0, 0)), createVec3(1, 0, 0)),
    normalizeVec3(rotateVec3ByQuat(rotation, createVec3(0, 1, 0)), createVec3(0, 1, 0)),
    normalizeVec3(rotateVec3ByQuat(rotation, createVec3(0, 0, 1)), createVec3(0, 0, 1))
  ];
}

function computeProjectedBoxRadius(halfExtents, axes, direction) {
  return (
    Math.abs(dotVec3(direction, axes[0])) * Number(halfExtents?.x ?? 0) +
    Math.abs(dotVec3(direction, axes[1])) * Number(halfExtents?.y ?? 0) +
    Math.abs(dotVec3(direction, axes[2])) * Number(halfExtents?.z ?? 0)
  );
}

function computeBoxFaceAxisNormal(shapeA, poseA, shapeB, poseB) {
  if (shapeA?.type !== 'box' || shapeB?.type !== 'box') {
    return null;
  }

  const worldPoseA = getShapeWorldPose(shapeA, poseA);
  const worldPoseB = getShapeWorldPose(shapeB, poseB);
  const axesA = getBoxWorldAxes(worldPoseA);
  const axesB = getBoxWorldAxes(worldPoseB);
  const centerDelta = subtractVec3(worldPoseB.position, worldPoseA.position);
  const candidateAxes = [...axesA, ...axesB];

  let bestAxis = null;
  let bestPenetration = Infinity;

  for (const axis of candidateAxes) {
    if (lengthSquaredVec3(axis) <= 1e-12) {
      continue;
    }

    const unitAxis = normalizeVec3(axis, createVec3(0, 1, 0));
    const distance = dotVec3(centerDelta, unitAxis);
    const radiusA = computeProjectedBoxRadius(shapeA.geometry?.halfExtents, axesA, unitAxis);
    const radiusB = computeProjectedBoxRadius(shapeB.geometry?.halfExtents, axesB, unitAxis);
    const overlap = radiusA + radiusB - Math.abs(distance);
    if (overlap <= 0) {
      return null;
    }

    if (overlap < bestPenetration) {
      bestPenetration = overlap;
      bestAxis = distance >= 0 ? unitAxis : negateVec3(unitAxis);
    }
  }

  if (!bestAxis) {
    return null;
  }

  return {
    normal: bestAxis,
    penetration: bestPenetration
  };
}

function getBoxLocalCorners(halfExtents) {
  return [
    createVec3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
    createVec3(halfExtents.x, -halfExtents.y, -halfExtents.z),
    createVec3(halfExtents.x, halfExtents.y, -halfExtents.z),
    createVec3(-halfExtents.x, halfExtents.y, -halfExtents.z),
    createVec3(-halfExtents.x, -halfExtents.y, halfExtents.z),
    createVec3(halfExtents.x, -halfExtents.y, halfExtents.z),
    createVec3(halfExtents.x, halfExtents.y, halfExtents.z),
    createVec3(-halfExtents.x, halfExtents.y, halfExtents.z)
  ];
}

function projectWorldFacePoints(points, normal) {
  const basis = createTangentBasis(normal);
  return points.map((point, index) => ({
    index,
    u: dotVec3(point.position, basis.tangentA),
    v: dotVec3(point.position, basis.tangentB)
  }));
}

function orderWorldFacePoints(points, normal) {
  if (!Array.isArray(points) || points.length <= 2) {
    return Array.isArray(points) ? points.slice() : [];
  }

  const projected = projectWorldFacePoints(points, normal);
  const orderedProjected = orderProjectedPolygon(projected);
  return orderedProjected.map((projectedPoint) => points[projectedPoint.index]);
}

function createWorldFace(id, normal, points) {
  const orderedPoints = orderWorldFacePoints(points, normal);
  if (orderedPoints.length === 0) {
    return null;
  }

  const unitNormal = normalizeVec3(normal, createVec3(0, 1, 0));
  return {
    id: String(id ?? '').trim() || 'face:unknown',
    normal: unitNormal,
    planeOffset: dotVec3(unitNormal, orderedPoints[0].position),
    points: orderedPoints
  };
}

function flipWorldFace(face) {
  if (!face) {
    return null;
  }

  return createWorldFace(face.id, negateVec3(face.normal), face.points.slice().reverse());
}

function getBoxWorldFaces(shape, pose) {
  const worldPose = getShapeWorldPose(shape, pose);
  const localCorners = getBoxLocalCorners(shape.geometry.halfExtents);
  const faceDefinitions = [
    { id: 'face:x+', normal: createVec3(1, 0, 0), indices: [1, 5, 6, 2] },
    { id: 'face:x-', normal: createVec3(-1, 0, 0), indices: [4, 0, 3, 7] },
    { id: 'face:y+', normal: createVec3(0, 1, 0), indices: [3, 2, 6, 7] },
    { id: 'face:y-', normal: createVec3(0, -1, 0), indices: [0, 4, 5, 1] },
    { id: 'face:z+', normal: createVec3(0, 0, 1), indices: [4, 7, 6, 5] },
    { id: 'face:z-', normal: createVec3(0, 0, -1), indices: [0, 1, 2, 3] }
  ];

  return faceDefinitions
    .map((definition) => createWorldFace(
      definition.id,
      rotateVec3ByQuat(worldPose.rotation, definition.normal),
      definition.indices.map((index) => createFeaturePoint(
        transformPointByPose(worldPose, localCorners[index]),
        `vertex:${index}`
      ))
    ))
    .filter(Boolean);
}

function getConvexHullWorldFaces(shape, pose) {
  const worldPose = getShapeWorldPose(shape, pose);
  const localVertices = Array.isArray(shape?.geometry?.vertices) ? shape.geometry.vertices : [];
  const faces = Array.isArray(shape?.geometry?.faces) ? shape.geometry.faces : [];

  return faces
    .map((face, faceIndex) => {
      const boundaryIndices = Array.isArray(face.boundaryIndices) ? face.boundaryIndices : [];
      const points = boundaryIndices
        .map((vertexIndex) => localVertices[vertexIndex])
        .filter(Boolean)
        .map((localPoint, boundaryIndex) => createFeaturePoint(
          transformPointByPose(worldPose, localPoint),
          `vertex:${boundaryIndices[boundaryIndex]}`
        ));
      return createWorldFace(
        `face:${faceIndex}`,
        rotateVec3ByQuat(worldPose.rotation, face.normal ?? createVec3(0, 1, 0)),
        points
      );
    })
    .filter((face) => face && face.points.length >= 2);
}

function getPolygonalWorldFaces(shape, pose) {
  if (shape?.type === 'box') {
    return getBoxWorldFaces(shape, pose);
  }

  if (shape?.type === 'convex-hull') {
    return getConvexHullWorldFaces(shape, pose);
  }

  return [];
}

function findBestFacingFace(faces, direction) {
  let bestFace = null;
  let bestAlignment = -Infinity;

  for (const face of Array.isArray(faces) ? faces : []) {
    const alignment = dotVec3(face.normal, direction);
    if (alignment > bestAlignment) {
      bestAlignment = alignment;
      bestFace = face;
    }
  }

  return bestFace ? { face: bestFace, alignment: bestAlignment } : null;
}

function isWorldFaceNearSupportPlane(face, direction, supportPlaneOffset, epsilon = FACE_SUPPORT_PLANE_EPSILON) {
  if (!face || !Array.isArray(face.points) || face.points.length < 3) {
    return false;
  }

  return face.points.every((point) => dotVec3(point.position, direction) >= supportPlaneOffset - epsilon);
}

function getFaceClipConfiguration(shapeA, poseA, shapeB, poseB, normal) {
  const facesA = getPolygonalWorldFaces(shapeA, poseA);
  const facesB = getPolygonalWorldFaces(shapeB, poseB);
  if (facesB.length === 0) {
    return null;
  }

  const supportA = getShapeSupportPolygon(shapeA, poseA, normal);
  const supportB = getShapeSupportPolygon(shapeB, poseB, negateVec3(normal));
  const bestA = findBestFacingFace(facesA, normal);
  if (bestA?.face && isWorldFaceNearSupportPlane(bestA.face, normal, supportA.planeOffset)) {
    const bestB = findBestFacingFace(facesB, negateVec3(bestA.face.normal));
    if (bestB?.face && isWorldFaceNearSupportPlane(bestB.face, negateVec3(bestA.face.normal), supportB.planeOffset)) {
      return {
        referenceFace: bestA.face,
        incidentFace: bestB.face
      };
    }
  }

  const bestB = findBestFacingFace(facesB, negateVec3(normal));
  if (!bestB?.face || facesA.length === 0) {
    return null;
  }

  const flippedReference = flipWorldFace(bestB.face);
  const incident = findBestFacingFace(facesA, negateVec3(flippedReference.normal));
  if (!isWorldFaceNearSupportPlane(bestB.face, negateVec3(normal), supportB.planeOffset)) {
    return null;
  }

  if (!incident?.face || !isWorldFaceNearSupportPlane(incident.face, negateVec3(flippedReference.normal), supportA.planeOffset)) {
    return null;
  }

  return {
    referenceFace: flippedReference,
    incidentFace: incident.face
  };
}

function intersectSegmentWithPlane(startPoint, endPoint, startDistance, endDistance, planeTag) {
  const denominator = startDistance - endDistance;
  const t = Math.abs(denominator) <= 1e-12 ? 0 : startDistance / denominator;
  return createFeaturePoint(
    createVec3(
      startPoint.position.x + (endPoint.position.x - startPoint.position.x) * t,
      startPoint.position.y + (endPoint.position.y - startPoint.position.y) * t,
      startPoint.position.z + (endPoint.position.z - startPoint.position.z) * t
    ),
    `${startPoint.featureId}|${endPoint.featureId}|${planeTag}`
  );
}

function clipPolygonAgainstPlane(points, planePoint, planeNormal, planeTag) {
  if (!Array.isArray(points) || points.length === 0) {
    return [];
  }

  const output = [];
  let previousPoint = points[points.length - 1];
  let previousDistance = dotVec3(subtractVec3(previousPoint.position, planePoint), planeNormal);
  let previousInside = previousDistance >= -FACE_CLIP_EPSILON;

  for (const currentPoint of points) {
    const currentDistance = dotVec3(subtractVec3(currentPoint.position, planePoint), planeNormal);
    const currentInside = currentDistance >= -FACE_CLIP_EPSILON;

    if (currentInside !== previousInside) {
      output.push(intersectSegmentWithPlane(previousPoint, currentPoint, previousDistance, currentDistance, planeTag));
    }

    if (currentInside) {
      output.push(currentPoint);
    }

    previousPoint = currentPoint;
    previousDistance = currentDistance;
    previousInside = currentInside;
  }

  return output;
}

function dedupeFaceClipPoints(points) {
  const uniquePoints = [];
  const seen = new Set();

  for (const point of points) {
    const key = `${point.position.x.toFixed(5)}|${point.position.y.toFixed(5)}|${point.position.z.toFixed(5)}`;
    if (seen.has(key)) {
      continue;
    }

    seen.add(key);
    uniquePoints.push(point);
  }

  return uniquePoints;
}

function computeFeaturePointCentroid(points) {
  if (!Array.isArray(points) || points.length === 0) {
    return createVec3();
  }

  const centroid = points.reduce((sum, point) => createVec3(
    sum.x + point.position.x,
    sum.y + point.position.y,
    sum.z + point.position.z
  ), createVec3());

  return scaleVec3(centroid, 1 / points.length);
}

function buildFaceClippedManifoldCandidates(shapeA, poseA, shapeB, poseB, normal) {
  const configuration = getFaceClipConfiguration(shapeA, poseA, shapeB, poseB, normal);
  if (!configuration?.referenceFace || !configuration?.incidentFace) {
    return null;
  }

  return buildProjectedFaceManifoldCandidates(configuration.referenceFace, configuration.incidentFace);
}

function buildProjectedFaceManifoldCandidates(referenceFace, incidentFace) {
  const referencePoints = referenceFace?.points ?? [];
  const incidentPoints = incidentFace?.points ?? [];
  if (referencePoints.length < 3 || incidentPoints.length < 3) {
    return null;
  }

  const basis = createTangentBasis(referenceFace.normal);
  const referencePolygon = projectSupportPolygon(referencePoints.map((point) => ({
    worldPoint: point.position,
    featureId: point.featureId
  })), basis);
  const incidentPolygon = projectSupportPolygon(incidentPoints.map((point) => ({
    worldPoint: point.position,
    featureId: point.featureId
  })), basis);
  const clippedPolygon = intersectSupportPolygons(referencePolygon, incidentPolygon);
  if (clippedPolygon.length === 0) {
    return null;
  }

  const referenceOffset = dotVec3(referenceFace.normal, referencePoints[0].position);
  const incidentOffset = incidentPoints.reduce(
    (maxOffset, point) => Math.max(maxOffset, dotVec3(referenceFace.normal, point.position)),
    -Infinity
  );
  const separation = incidentOffset - referenceOffset;
  if (separation > FACE_CLIP_EPSILON) {
    return null;
  }

  const penetration = Math.max(0, -separation);
  const candidates = clippedPolygon.map((point) => {
    const referencePoint = addScaledVec3(
      addScaledVec3(createVec3(), basis.tangentA, point.u),
      basis.tangentB,
      point.v
    );
    referencePoint.x += referenceFace.normal.x * referenceOffset;
    referencePoint.y += referenceFace.normal.y * referenceOffset;
    referencePoint.z += referenceFace.normal.z * referenceOffset;

    const incidentPoint = addScaledVec3(referencePoint, referenceFace.normal, separation);
    const referenceFeature = describeProjectedFeatureAtPoint(point, referencePolygon);
    const incidentFeature = describeProjectedFeatureAtPoint(point, incidentPolygon);

    return {
      featureId: `${referenceFeature}|${incidentFeature}`,
      position: createVec3(
        (referencePoint.x + incidentPoint.x) / 2,
        (referencePoint.y + incidentPoint.y) / 2,
        (referencePoint.z + incidentPoint.z) / 2
      ),
      penetration,
      separation
    };
  });

  if (candidates.length === 0) {
    return null;
  }

  return {
    normal: referenceFace.normal,
    candidates
  };
}

function buildBoxBoxFaceManifoldCandidates(shapeA, poseA, shapeB, poseB, normal) {
  const facesA = getBoxWorldFaces(shapeA, poseA);
  const facesB = getBoxWorldFaces(shapeB, poseB);
  const reference = findBestFacingFace(facesA, normal)?.face ?? null;
  const incident = findBestFacingFace(facesB, negateVec3(normal))?.face ?? null;
  if (!reference || !incident) {
    return null;
  }

  return buildProjectedFaceManifoldCandidates(reference, incident);
}

function intersectSupportPolygons(referencePolygon, incidentPolygon) {
  const reference = orderProjectedPolygon(referencePolygon);
  const incident = orderProjectedPolygon(incidentPolygon);
  if (reference.length === 0 || incident.length === 0) {
    return [];
  }

  const points = [];

  for (const point of reference) {
    if (isPointInsideConvexPolygon(point, incident)) {
      points.push({ u: point.u, v: point.v });
    }
  }

  for (const point of incident) {
    if (isPointInsideConvexPolygon(point, reference)) {
      points.push({ u: point.u, v: point.v });
    }
  }

  if (reference.length >= 2 && incident.length >= 2) {
    for (let refIndex = 0; refIndex < reference.length; refIndex += 1) {
      const refStart = reference[refIndex];
      const refEnd = reference[(refIndex + 1) % reference.length];

      for (let incidentIndex = 0; incidentIndex < incident.length; incidentIndex += 1) {
        const incidentStart = incident[incidentIndex];
        const incidentEnd = incident[(incidentIndex + 1) % incident.length];
        const intersection = intersectProjectedSegments(refStart, refEnd, incidentStart, incidentEnd);
        if (intersection) {
          points.push(intersection);
        }
      }
    }
  }

  return orderProjectedPolygon(dedupeProjectedPoints(points));
}

function buildSupportPolygonClippedCandidates(shapeA, poseA, shapeB, poseB, normal) {
  const basis = createTangentBasis(normal);
  const supportA = getShapeSupportPolygon(shapeA, poseA, normal);
  const supportB = getShapeSupportPolygon(shapeB, poseB, negateVec3(normal));
  if ((supportA.points?.length ?? 0) === 0 || (supportB.points?.length ?? 0) === 0) {
    return [];
  }

  const referencePolygon = projectSupportPolygon(supportA.points, basis);
  const incidentPolygon = projectSupportPolygon(supportB.points, basis);
  const clippedPolygon = intersectSupportPolygons(referencePolygon, incidentPolygon);
  if (clippedPolygon.length === 0) {
    return [];
  }

  const referenceOffset = Number(supportA.planeOffset ?? 0);
  const incidentOffset = -Number(supportB.planeOffset ?? 0);
  const separation = incidentOffset - referenceOffset;
  const penetration = Math.max(0, referenceOffset - incidentOffset);

  return clippedPolygon.map((point, index) => {
    const referencePoint = addScaledVec3(
      addScaledVec3(createVec3(), basis.tangentA, point.u),
      basis.tangentB,
      point.v
    );
    referencePoint.x += normal.x * referenceOffset;
    referencePoint.y += normal.y * referenceOffset;
    referencePoint.z += normal.z * referenceOffset;

    const incidentPoint = addScaledVec3(
      addScaledVec3(createVec3(), basis.tangentA, point.u),
      basis.tangentB,
      point.v
    );
    incidentPoint.x += normal.x * incidentOffset;
    incidentPoint.y += normal.y * incidentOffset;
    incidentPoint.z += normal.z * incidentOffset;

    const referenceFeature = describeProjectedFeatureAtPoint(point, referencePolygon);
    const incidentFeature = describeProjectedFeatureAtPoint(point, incidentPolygon);

    return {
      featureId: `${referenceFeature}|${incidentFeature}`,
      position: createVec3(
        (referencePoint.x + incidentPoint.x) / 2,
        (referencePoint.y + incidentPoint.y) / 2,
        (referencePoint.z + incidentPoint.z) / 2
      ),
      penetration,
      separation
    };
  });
}

export function buildConvexContactManifold(shapeA, poseA, shapeB, poseB, epaResult) {
  if (!epaResult || epaResult.penetration <= 0) {
    return {
      normal: epaResult?.normal ?? createVec3(0, 1, 0),
      contacts: []
    };
  }

  const boxFaceAxis = computeBoxFaceAxisNormal(shapeA, poseA, shapeB, poseB);
  const manifoldNormal = boxFaceAxis?.normal ?? epaResult.normal;
  const polygonalResult = shapeA?.type === 'box' && shapeB?.type === 'box' && boxFaceAxis?.normal
    ? (buildBoxBoxFaceManifoldCandidates(shapeA, poseA, shapeB, poseB, boxFaceAxis.normal)
      ?? buildFaceClippedManifoldCandidates(shapeA, poseA, shapeB, poseB, manifoldNormal))
    : buildFaceClippedManifoldCandidates(shapeA, poseA, shapeB, poseB, manifoldNormal);
  const polygonalCandidates = polygonalResult?.candidates?.filter((candidate) => candidate.penetration > 0) ?? [];
  if (polygonalCandidates.length > 0) {
    return {
      normal: polygonalResult.normal,
      contacts: reduceOrderedPolygonCandidates(polygonalCandidates, polygonalResult.normal, 4)
    };
  }

  const clippedCandidates = buildSupportPolygonClippedCandidates(shapeA, poseA, shapeB, poseB, manifoldNormal)
    .filter((candidate) => candidate.penetration > 0);
  if (clippedCandidates.length > 0) {
    return {
      normal: manifoldNormal,
      contacts: selectManifoldCandidates(clippedCandidates, manifoldNormal, 4)
    };
  }

  const faceCandidates = collectFaceCandidates(epaResult.faceVertices ?? [], epaResult.normal);
  const sampledCandidates = collectSampledCandidates(shapeA, poseA, shapeB, poseB, epaResult.normal);
  const deepestPenetration = Math.max(epaResult.penetration, 1e-6);
  const minimumPenetration = Math.max(1e-5, deepestPenetration * 0.2);
  const filteredCandidates = [...faceCandidates, ...sampledCandidates]
    .filter((candidate) => candidate.penetration >= minimumPenetration);

  return {
    normal: manifoldNormal,
    contacts: selectManifoldCandidates(filteredCandidates, manifoldNormal, 4)
  };
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
