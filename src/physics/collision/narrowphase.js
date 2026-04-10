import { cloneVec3, createVec3 } from '../math/vec3.js';
import { collideBoxPair } from './box-box.js';

function cloneContact(contact) {
  return {
    id: contact.id,
    position: cloneVec3(contact.position),
    normal: cloneVec3(contact.normal),
    penetration: contact.penetration,
    separation: contact.separation,
    featureId: contact.featureId
  };
}

export function createContactPair(options = {}) {
  return {
    id: String(options.id ?? options.pairKey ?? '').trim() || String(options.pairKey ?? '').trim(),
    pairKey: String(options.pairKey ?? '').trim() || null,
    algorithm: String(options.algorithm ?? '').trim() || 'unknown',
    status: String(options.status ?? '').trim() || 'unknown',
    pairKind: String(options.pairKind ?? '').trim() || 'dynamic-dynamic',
    colliderAId: String(options.colliderAId ?? '').trim() || null,
    colliderBId: String(options.colliderBId ?? '').trim() || null,
    bodyAId: String(options.bodyAId ?? '').trim() || null,
    bodyBId: String(options.bodyBId ?? '').trim() || null,
    shapeAType: String(options.shapeAType ?? '').trim() || 'unknown',
    shapeBType: String(options.shapeBType ?? '').trim() || 'unknown',
    normal: cloneVec3(options.normal ?? createVec3()),
    penetration: Number(options.penetration ?? 0),
    contactCount: Number(options.contactCount ?? (Array.isArray(options.contacts) ? options.contacts.length : 0)),
    contacts: Array.isArray(options.contacts) ? options.contacts.map((contact) => cloneContact(contact)) : []
  };
}

export function cloneContactPair(contactPair) {
  return createContactPair(contactPair);
}

export function runNarrowphase(pairs) {
  const contactPairs = [];
  const unsupportedPairs = [];

  for (const pair of Array.isArray(pairs) ? pairs : []) {
    let contactPair = null;

    if (pair.shapeAType === 'box' && pair.shapeBType === 'box') {
      contactPair = collideBoxPair(pair);
    }

    if (contactPair) {
      contactPairs.push(createContactPair(contactPair));
      continue;
    }

    unsupportedPairs.push(pair.pairKey);
  }

  return {
    contactPairs,
    summary: {
      pairCount: Array.isArray(pairs) ? pairs.length : 0,
      contactCount: contactPairs.length,
      unsupportedPairCount: unsupportedPairs.length,
      unsupportedPairs,
      algorithms: contactPairs.reduce((counts, contactPair) => {
        counts[contactPair.algorithm] = (counts[contactPair.algorithm] ?? 0) + 1;
        return counts;
      }, {})
    }
  };
}
