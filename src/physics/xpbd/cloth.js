import { cloneVec3, createVec3, lengthVec3, subtractVec3 } from '../math/vec3.js';
import { createDistanceConstraint, createPinConstraint } from './constraints.js';

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
  const normalized = String(value ?? 'top-row').trim().toLowerCase();
  if (!normalized) {
    return 'top-row';
  }

  if (normalized === 'top' || normalized === 'top-edge') {
    return 'top-row';
  }

  if (normalized === 'top-corners' || normalized === 'corners' || normalized === 'none' || normalized === 'left-edge' || normalized === 'right-edge') {
    return normalized;
  }

  return 'top-row';
}

function shouldPinParticle(pinMode, rowIndex, columnIndex, rowCount, columnCount) {
  const lastColumn = columnCount - 1;
  switch (pinMode) {
    case 'none':
      return false;
    case 'corners':
      return (rowIndex === 0 || rowIndex === rowCount - 1) && (columnIndex === 0 || columnIndex === lastColumn);
    case 'top-corners':
      return rowIndex === 0 && (columnIndex === 0 || columnIndex === lastColumn);
    case 'left-edge':
      return columnIndex === 0;
    case 'right-edge':
      return columnIndex === lastColumn;
    case 'top-row':
    default:
      return rowIndex === 0;
  }
}

export function createClothSheetDefinition(options = {}) {
  const clothId = String(options.id ?? '').trim() || 'cloth-1';
  const rows = toCount(options.rows, 6);
  const columns = toCount(options.columns ?? options.cols, 8);
  const spacing = toPositiveNumber(options.spacing, 20);
  const origin = cloneVec3(options.position ?? createVec3());
  const pinMode = normalizePinMode(options.pinMode);
  const particleMass = toPositiveNumber(options.particleMass ?? options.mass, 0.2);
  const inverseMass = 1 / particleMass;
  const damping = toNonNegativeNumber(options.damping, 0.03);
  const collisionMargin = toPositiveNumber(options.collisionMargin, Math.max(0.5, spacing * 0.15));
  const selfCollisionEnabled = options.selfCollisionEnabled === true;
  const selfCollisionDistance = toPositiveNumber(options.selfCollisionDistance, Math.max(collisionMargin * 2, spacing * 0.35));
  const stretchCompliance = toNonNegativeNumber(options.stretchCompliance, 0);
  const shearCompliance = toNonNegativeNumber(options.shearCompliance, 0.00015);
  const bendCompliance = toNonNegativeNumber(options.bendCompliance, 0.0005);

  const particles = [];
  const distanceConstraints = [];
  const pinConstraints = [];
  const structuralEdges = [];
  const shearEdges = [];
  const bendEdges = [];
  const particleIds = [];
  const particleIdGrid = Array.from({ length: rows }, () => Array(columns).fill(null));

  for (let rowIndex = 0; rowIndex < rows; rowIndex += 1) {
    for (let columnIndex = 0; columnIndex < columns; columnIndex += 1) {
      const particleId = `${clothId}:particle:${rowIndex}:${columnIndex}`;
      const pinned = shouldPinParticle(pinMode, rowIndex, columnIndex, rows, columns);
      const position = createVec3(
        origin.x + columnIndex * spacing,
        origin.y,
        origin.z + rowIndex * spacing
      );
      particles.push({
        id: particleId,
        clothId,
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
      particleIdGrid[rowIndex][columnIndex] = particleId;

      if (pinned) {
        pinConstraints.push(createPinConstraint({
          id: `${clothId}:pin:${rowIndex}:${columnIndex}`,
          particleId,
          targetPosition: position,
          compliance: 0
        }));
      }
    }
  }

  const getParticlePosition = (rowIndex, columnIndex) => {
    const particle = particles[rowIndex * columns + columnIndex];
    return particle?.position ?? createVec3();
  };

  const addDistance = (kind, rowA, columnA, rowB, columnB, compliance, edgeStore) => {
    const particleAId = particleIdGrid[rowA]?.[columnA];
    const particleBId = particleIdGrid[rowB]?.[columnB];
    if (!particleAId || !particleBId) {
      return;
    }

    const restLength = lengthVec3(subtractVec3(getParticlePosition(rowB, columnB), getParticlePosition(rowA, columnA)));
    const constraint = createDistanceConstraint({
      id: `${clothId}:${kind}:${rowA}:${columnA}:${rowB}:${columnB}`,
      kind,
      particleAId,
      particleBId,
      restLength,
      compliance
    });
    distanceConstraints.push(constraint);
    edgeStore.push({
      id: `${clothId}:${kind}-edge:${rowA}:${columnA}:${rowB}:${columnB}`,
      particleAId,
      particleBId
    });
  };

  for (let rowIndex = 0; rowIndex < rows; rowIndex += 1) {
    for (let columnIndex = 0; columnIndex < columns; columnIndex += 1) {
      if (columnIndex + 1 < columns) {
        addDistance('stretch', rowIndex, columnIndex, rowIndex, columnIndex + 1, stretchCompliance, structuralEdges);
      }
      if (rowIndex + 1 < rows) {
        addDistance('stretch', rowIndex, columnIndex, rowIndex + 1, columnIndex, stretchCompliance, structuralEdges);
      }
      if (rowIndex + 1 < rows && columnIndex + 1 < columns) {
        addDistance('shear', rowIndex, columnIndex, rowIndex + 1, columnIndex + 1, shearCompliance, shearEdges);
      }
      if (rowIndex + 1 < rows && columnIndex - 1 >= 0) {
        addDistance('shear', rowIndex, columnIndex, rowIndex + 1, columnIndex - 1, shearCompliance, shearEdges);
      }
      if (columnIndex + 2 < columns) {
        addDistance('bend', rowIndex, columnIndex, rowIndex, columnIndex + 2, bendCompliance, bendEdges);
      }
      if (rowIndex + 2 < rows) {
        addDistance('bend', rowIndex, columnIndex, rowIndex + 2, columnIndex, bendCompliance, bendEdges);
      }
    }
  }

  return {
    id: clothId,
    type: 'cloth-sheet',
    rows,
    columns,
    spacing,
    pinMode,
    particleMass,
    damping,
    collisionMargin,
    selfCollisionEnabled,
    selfCollisionDistance,
    stretchCompliance,
    shearCompliance,
    bendCompliance,
    position: origin,
    particleIds,
    particleIdGrid,
    particles,
    distanceConstraints,
    pinConstraints,
    structuralEdges,
    shearEdges,
    bendEdges
  };
}
