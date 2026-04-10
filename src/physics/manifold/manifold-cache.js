import { cloneVec3, createVec3 } from '../math/vec3.js';

export function cloneManifoldContact(contact) {
  return {
    id: contact.id,
    featureId: contact.featureId,
    position: cloneVec3(contact.position),
    normal: cloneVec3(contact.normal),
    penetration: contact.penetration,
    separation: contact.separation,
    accumulatedNormalImpulse: contact.accumulatedNormalImpulse,
    accumulatedTangentImpulseA: contact.accumulatedTangentImpulseA,
    accumulatedTangentImpulseB: contact.accumulatedTangentImpulseB,
    lifetime: contact.lifetime,
    persisted: contact.persisted
  };
}

export function cloneManifold(manifold) {
  return {
    id: manifold.id,
    pairKey: manifold.pairKey,
    pairKind: manifold.pairKind,
    colliderAId: manifold.colliderAId,
    colliderBId: manifold.colliderBId,
    bodyAId: manifold.bodyAId,
    bodyBId: manifold.bodyBId,
    shapeAType: manifold.shapeAType,
    shapeBType: manifold.shapeBType,
    algorithm: manifold.algorithm,
    status: manifold.status,
    friction: manifold.friction,
    restitution: manifold.restitution,
    restitutionThreshold: manifold.restitutionThreshold,
    normal: cloneVec3(manifold.normal),
    contactCount: manifold.contactCount,
    contacts: manifold.contacts.map((contact) => cloneManifoldContact(contact)),
    lifetime: manifold.lifetime,
    lastUpdatedTick: manifold.lastUpdatedTick
  };
}

function createPersistentContact(contact, previousContact, simulationTick) {
  const matchedPreviousContact = previousContact ?? null;
  return {
    id: contact.id,
    featureId: contact.featureId ?? contact.id,
    position: cloneVec3(contact.position),
    normal: cloneVec3(contact.normal),
    penetration: Number(contact.penetration ?? 0),
    separation: Number(contact.separation ?? 0),
    accumulatedNormalImpulse: matchedPreviousContact?.accumulatedNormalImpulse ?? 0,
    accumulatedTangentImpulseA: matchedPreviousContact?.accumulatedTangentImpulseA ?? 0,
    accumulatedTangentImpulseB: matchedPreviousContact?.accumulatedTangentImpulseB ?? 0,
    lifetime: matchedPreviousContact?.lastUpdatedTick === simulationTick
      ? matchedPreviousContact.lifetime
      : (matchedPreviousContact ? matchedPreviousContact.lifetime + 1 : 1),
    lastUpdatedTick: simulationTick,
    persisted: Boolean(matchedPreviousContact)
  };
}

function findMatchingPreviousContact(previousContacts, contact) {
  if (!Array.isArray(previousContacts) || previousContacts.length === 0) {
    return null;
  }

  const featureId = contact.featureId ?? contact.id;
  for (const previousContact of previousContacts) {
    if ((previousContact.featureId ?? previousContact.id) === featureId) {
      return previousContact;
    }
  }

  return null;
}

function createPersistentManifold(contactPair, previousManifold, simulationTick) {
  const previousContacts = previousManifold?.contacts ?? [];
  const contacts = Array.isArray(contactPair.contacts)
    ? contactPair.contacts.map((contact) => createPersistentContact(
      contact,
      findMatchingPreviousContact(previousContacts, contact),
      simulationTick
    ))
    : [];

  return {
    id: `${contactPair.pairKey}:manifold`,
    pairKey: contactPair.pairKey,
    pairKind: contactPair.pairKind,
    colliderAId: contactPair.colliderAId,
    colliderBId: contactPair.colliderBId,
    bodyAId: contactPair.bodyAId,
    bodyBId: contactPair.bodyBId,
    shapeAType: contactPair.shapeAType,
    shapeBType: contactPair.shapeBType,
    algorithm: contactPair.algorithm,
    status: contactPair.status,
    friction: Number(contactPair.friction ?? previousManifold?.friction ?? 0.5),
    restitution: Number(contactPair.restitution ?? previousManifold?.restitution ?? 0),
    restitutionThreshold: Number(contactPair.restitutionThreshold ?? previousManifold?.restitutionThreshold ?? 1),
    normal: cloneVec3(contactPair.normal ?? createVec3()),
    contactCount: contacts.length,
    contacts,
    lifetime: previousManifold?.lastUpdatedTick === simulationTick
      ? previousManifold.lifetime
      : (previousManifold ? previousManifold.lifetime + 1 : 1),
    lastUpdatedTick: simulationTick
  };
}

export class ManifoldCache {
  constructor() {
    this.records = new Map();
  }

  clear() {
    this.records.clear();
  }

  get(pairKey) {
    const manifold = this.records.get(pairKey);
    return manifold ? cloneManifold(manifold) : null;
  }

  getMutable(pairKey) {
    return this.records.get(pairKey) ?? null;
  }

  list() {
    return Array.from(this.records.values(), (manifold) => cloneManifold(manifold));
  }

  syncFromContactPairs(contactPairs, simulationTick = 0) {
    const nextRecords = new Map();

    for (const contactPair of Array.isArray(contactPairs) ? contactPairs : []) {
      const previousManifold = this.records.get(contactPair.pairKey) ?? null;
      const manifold = createPersistentManifold(contactPair, previousManifold, simulationTick);
      nextRecords.set(manifold.pairKey, manifold);
    }

    this.records = nextRecords;
    return Array.from(this.records.values());
  }
}
