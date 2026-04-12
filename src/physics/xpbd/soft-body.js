import { cloneVec3, createVec3, lengthVec3, subtractVec3 } from '../math/vec3.js';
import { createDistanceConstraint, createPinConstraint, createVolumeConstraint } from './constraints.js';

function toCount(value, fallback, minimum = 2) {
  const parsed = Math.floor(Number(value));
  return Number.isFinite(parsed) && parsed >= minimum ? parsed : fallback;
}

function toPositiveNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
}

function toNonNegativeNumber(value, fallback) {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 ? parsed : fallback;
}

function normalizePinMode(value) {
  const normalized = String(value ?? 'none').trim().toLowerCase();
  if (!normalized) {
    return 'none';
  }

  if (normalized === 'top' || normalized === 'top-face') {
    return 'top-layer';
  }

  if (normalized === 'top-layer' || normalized === 'top-corners' || normalized === 'corners' || normalized === 'none') {
    return normalized;
  }

  return 'none';
}

function shouldPinParticle(pinMode, layerIndex, rowIndex, columnIndex, layerCount, rowCount, columnCount) {
  const lastLayer = layerCount - 1;
  const lastRow = rowCount - 1;
  const lastColumn = columnCount - 1;

  switch (pinMode) {
    case 'top-layer':
      return layerIndex === 0;
    case 'top-corners':
      return layerIndex === 0 && (rowIndex === 0 || rowIndex === lastRow) && (columnIndex === 0 || columnIndex === lastColumn);
    case 'corners':
      return (layerIndex === 0 || layerIndex === lastLayer) &&
        (rowIndex === 0 || rowIndex === lastRow) &&
        (columnIndex === 0 || columnIndex === lastColumn);
    case 'none':
    default:
      return false;
  }
}

function addEdgeConstraint(records, particleIdGrid, particlePositions, softBodyId, kind, pointA, pointB, compliance, edgeStore) {
  const [layerA, rowA, columnA] = pointA;
  const [layerB, rowB, columnB] = pointB;
  const particleAId = particleIdGrid[layerA]?.[rowA]?.[columnA];
  const particleBId = particleIdGrid[layerB]?.[rowB]?.[columnB];
  if (!particleAId || !particleBId) {
    return;
  }

  const positionA = particlePositions.get(particleAId) ?? createVec3();
  const positionB = particlePositions.get(particleBId) ?? createVec3();
  const restLength = lengthVec3(subtractVec3(positionB, positionA));
  records.push(createDistanceConstraint({
    id: `${softBodyId}:${kind}:${layerA}:${rowA}:${columnA}:${layerB}:${rowB}:${columnB}`,
    kind,
    particleAId,
    particleBId,
    restLength,
    compliance
  }));
  edgeStore.push({
    id: `${softBodyId}:${kind}-edge:${layerA}:${rowA}:${columnA}:${layerB}:${rowB}:${columnB}`,
    particleAId,
    particleBId
  });
}

function addTetraVolumeConstraint(records, particleIdGrid, particlePositions, softBodyId, tetraIndex, pointA, pointB, pointC, pointD, compliance) {
  const [layerA, rowA, columnA] = pointA;
  const [layerB, rowB, columnB] = pointB;
  const [layerC, rowC, columnC] = pointC;
  const [layerD, rowD, columnD] = pointD;
  const particleAId = particleIdGrid[layerA]?.[rowA]?.[columnA];
  const particleBId = particleIdGrid[layerB]?.[rowB]?.[columnB];
  const particleCId = particleIdGrid[layerC]?.[rowC]?.[columnC];
  const particleDId = particleIdGrid[layerD]?.[rowD]?.[columnD];
  if (!particleAId || !particleBId || !particleCId || !particleDId) {
    return;
  }

  const positionA = particlePositions.get(particleAId) ?? createVec3();
  const positionB = particlePositions.get(particleBId) ?? createVec3();
  const positionC = particlePositions.get(particleCId) ?? createVec3();
  const positionD = particlePositions.get(particleDId) ?? createVec3();
  const edgeAB = subtractVec3(positionB, positionA);
  const edgeAC = subtractVec3(positionC, positionA);
  const edgeAD = subtractVec3(positionD, positionA);
  const restVolume = (
    (edgeAB.y * edgeAC.z - edgeAB.z * edgeAC.y) * edgeAD.x +
    (edgeAB.z * edgeAC.x - edgeAB.x * edgeAC.z) * edgeAD.y +
    (edgeAB.x * edgeAC.y - edgeAB.y * edgeAC.x) * edgeAD.z
  ) / 6;

  records.push(createVolumeConstraint({
    id: `${softBodyId}:volume:${tetraIndex}:${layerA}:${rowA}:${columnA}:${layerD}:${rowD}:${columnD}`,
    kind: 'volume',
    particleAId,
    particleBId,
    particleCId,
    particleDId,
    restVolume,
    compliance
  }));
}

export function createSoftBodyCubeDefinition(options = {}) {
  const softBodyId = String(options.id ?? '').trim() || 'soft-body-1';
  const rows = toCount(options.rows, 4);
  const columns = toCount(options.columns ?? options.cols, 4);
  const layers = toCount(options.layers ?? options.depth, 4);
  const spacing = toPositiveNumber(options.spacing, 20);
  const origin = cloneVec3(options.position ?? createVec3());
  const pinMode = normalizePinMode(options.pinMode);
  const particleMass = toPositiveNumber(options.particleMass ?? options.mass, 0.2);
  const inverseMass = 1 / particleMass;
  const damping = toNonNegativeNumber(options.damping, 0.04);
  const collisionMargin = toPositiveNumber(options.collisionMargin, Math.max(0.5, spacing * 0.18));
  const stretchCompliance = toNonNegativeNumber(options.stretchCompliance, 0);
  const shearCompliance = toNonNegativeNumber(options.shearCompliance, 0.0002);
  const bendCompliance = toNonNegativeNumber(options.bendCompliance, 0.0008);
  const volumeCompliance = toNonNegativeNumber(options.volumeCompliance, 0.00008);

  const particles = [];
  const distanceConstraints = [];
  const volumeConstraints = [];
  const pinConstraints = [];
  const structuralEdges = [];
  const shearEdges = [];
  const bendEdges = [];
  const particleIds = [];
  const particleIdGrid = Array.from({ length: layers }, () => (
    Array.from({ length: rows }, () => Array(columns).fill(null))
  ));
  const particlePositions = new Map();

  for (let layerIndex = 0; layerIndex < layers; layerIndex += 1) {
    for (let rowIndex = 0; rowIndex < rows; rowIndex += 1) {
      for (let columnIndex = 0; columnIndex < columns; columnIndex += 1) {
        const particleId = `${softBodyId}:particle:${layerIndex}:${rowIndex}:${columnIndex}`;
        const pinned = shouldPinParticle(pinMode, layerIndex, rowIndex, columnIndex, layers, rows, columns);
        const position = createVec3(
          origin.x + columnIndex * spacing,
          origin.y - layerIndex * spacing,
          origin.z + rowIndex * spacing
        );

        particles.push({
          id: particleId,
          softBodyId,
          layerIndex,
          rowIndex,
          columnIndex,
          position,
          previousPosition: cloneVec3(position),
          predictedPosition: cloneVec3(position),
          velocity: createVec3(),
          inverseMass: pinned ? 0 : inverseMass,
          pinned,
          pinTarget: pinned ? cloneVec3(position) : null
        });
        particleIds.push(particleId);
        particleIdGrid[layerIndex][rowIndex][columnIndex] = particleId;
        particlePositions.set(particleId, position);

        if (pinned) {
          pinConstraints.push(createPinConstraint({
            id: `${softBodyId}:pin:${layerIndex}:${rowIndex}:${columnIndex}`,
            particleId,
            targetPosition: position,
            compliance: 0
          }));
        }
      }
    }
  }

  for (let layerIndex = 0; layerIndex < layers; layerIndex += 1) {
    for (let rowIndex = 0; rowIndex < rows; rowIndex += 1) {
      for (let columnIndex = 0; columnIndex < columns; columnIndex += 1) {
        if (columnIndex + 1 < columns) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'stretch', [layerIndex, rowIndex, columnIndex], [layerIndex, rowIndex, columnIndex + 1], stretchCompliance, structuralEdges);
        }
        if (rowIndex + 1 < rows) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'stretch', [layerIndex, rowIndex, columnIndex], [layerIndex, rowIndex + 1, columnIndex], stretchCompliance, structuralEdges);
        }
        if (layerIndex + 1 < layers) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'stretch', [layerIndex, rowIndex, columnIndex], [layerIndex + 1, rowIndex, columnIndex], stretchCompliance, structuralEdges);
        }

        if (columnIndex + 1 < columns && rowIndex + 1 < rows) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex, rowIndex, columnIndex], [layerIndex, rowIndex + 1, columnIndex + 1], shearCompliance, shearEdges);
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex, rowIndex + 1, columnIndex], [layerIndex, rowIndex, columnIndex + 1], shearCompliance, shearEdges);
        }
        if (columnIndex + 1 < columns && layerIndex + 1 < layers) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex, rowIndex, columnIndex], [layerIndex + 1, rowIndex, columnIndex + 1], shearCompliance, shearEdges);
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex + 1, rowIndex, columnIndex], [layerIndex, rowIndex, columnIndex + 1], shearCompliance, shearEdges);
        }
        if (rowIndex + 1 < rows && layerIndex + 1 < layers) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex, rowIndex, columnIndex], [layerIndex + 1, rowIndex + 1, columnIndex], shearCompliance, shearEdges);
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex + 1, rowIndex, columnIndex], [layerIndex, rowIndex + 1, columnIndex], shearCompliance, shearEdges);
        }
        if (columnIndex + 1 < columns && rowIndex + 1 < rows && layerIndex + 1 < layers) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex, rowIndex, columnIndex], [layerIndex + 1, rowIndex + 1, columnIndex + 1], shearCompliance, shearEdges);
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex, rowIndex + 1, columnIndex], [layerIndex + 1, rowIndex, columnIndex + 1], shearCompliance, shearEdges);
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex + 1, rowIndex, columnIndex], [layerIndex, rowIndex + 1, columnIndex + 1], shearCompliance, shearEdges);
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'shear', [layerIndex + 1, rowIndex + 1, columnIndex], [layerIndex, rowIndex, columnIndex + 1], shearCompliance, shearEdges);
        }

        if (columnIndex + 2 < columns) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'bend', [layerIndex, rowIndex, columnIndex], [layerIndex, rowIndex, columnIndex + 2], bendCompliance, bendEdges);
        }
        if (rowIndex + 2 < rows) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'bend', [layerIndex, rowIndex, columnIndex], [layerIndex, rowIndex + 2, columnIndex], bendCompliance, bendEdges);
        }
        if (layerIndex + 2 < layers) {
          addEdgeConstraint(distanceConstraints, particleIdGrid, particlePositions, softBodyId, 'bend', [layerIndex, rowIndex, columnIndex], [layerIndex + 2, rowIndex, columnIndex], bendCompliance, bendEdges);
        }
      }
    }
  }

  let tetraIndex = 0;
  for (let layerIndex = 0; layerIndex < layers - 1; layerIndex += 1) {
    for (let rowIndex = 0; rowIndex < rows - 1; rowIndex += 1) {
      for (let columnIndex = 0; columnIndex < columns - 1; columnIndex += 1) {
        const a = [layerIndex, rowIndex, columnIndex];
        const b = [layerIndex, rowIndex, columnIndex + 1];
        const c = [layerIndex, rowIndex + 1, columnIndex];
        const d = [layerIndex, rowIndex + 1, columnIndex + 1];
        const e = [layerIndex + 1, rowIndex, columnIndex];
        const f = [layerIndex + 1, rowIndex, columnIndex + 1];
        const g = [layerIndex + 1, rowIndex + 1, columnIndex];
        const h = [layerIndex + 1, rowIndex + 1, columnIndex + 1];

        addTetraVolumeConstraint(volumeConstraints, particleIdGrid, particlePositions, softBodyId, tetraIndex++, a, b, d, h, volumeCompliance);
        addTetraVolumeConstraint(volumeConstraints, particleIdGrid, particlePositions, softBodyId, tetraIndex++, a, d, c, h, volumeCompliance);
        addTetraVolumeConstraint(volumeConstraints, particleIdGrid, particlePositions, softBodyId, tetraIndex++, a, c, g, h, volumeCompliance);
        addTetraVolumeConstraint(volumeConstraints, particleIdGrid, particlePositions, softBodyId, tetraIndex++, a, g, e, h, volumeCompliance);
        addTetraVolumeConstraint(volumeConstraints, particleIdGrid, particlePositions, softBodyId, tetraIndex++, a, e, f, h, volumeCompliance);
        addTetraVolumeConstraint(volumeConstraints, particleIdGrid, particlePositions, softBodyId, tetraIndex++, a, f, b, h, volumeCompliance);
      }
    }
  }

  return {
    id: softBodyId,
    type: 'soft-body-cube',
    rows,
    columns,
    layers,
    spacing,
    pinMode,
    particleMass,
    damping,
    collisionMargin,
    stretchCompliance,
    shearCompliance,
    bendCompliance,
    volumeCompliance,
    position: origin,
    particleIds,
    particleIdGrid,
    particles,
    distanceConstraints,
    volumeConstraints,
    pinConstraints,
    structuralEdges,
    shearEdges,
    bendEdges
  };
}
