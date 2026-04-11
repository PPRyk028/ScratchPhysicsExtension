function shouldIncludeDynamicBody(body) {
  return Boolean(body && body.enabled && body.motionType === 'dynamic');
}

function connectBodies(adjacency, bodyAId, bodyBId) {
  if (!bodyAId || !bodyBId || bodyAId === bodyBId) {
    return;
  }

  adjacency.get(bodyAId)?.add(bodyBId);
  adjacency.get(bodyBId)?.add(bodyAId);
}

export function cloneRigidBodyIsland(island) {
  return {
    id: island.id,
    bodyIds: [...island.bodyIds],
    manifoldPairKeys: [...island.manifoldPairKeys],
    jointIds: [...(island.jointIds ?? [])],
    manifoldCount: island.manifoldCount,
    jointCount: island.jointCount ?? 0,
    constraintCount: island.constraintCount ?? island.manifoldCount,
    contactCount: island.contactCount,
    sleepingBodyCount: island.sleepingBodyCount,
    awakeBodyCount: island.awakeBodyCount,
    touchesStatic: island.touchesStatic,
    touchesConstraint: island.touchesConstraint ?? false,
    canSleep: island.canSleep
  };
}

export function cloneRigidBodyIslandGraph(graph) {
  const source = graph ?? {
    islands: [],
    islandCount: 0,
    bodyCount: 0,
    sleepingBodyCount: 0,
    awakeBodyCount: 0
  };

  return {
    islands: Array.isArray(source.islands) ? source.islands.map((island) => cloneRigidBodyIsland(island)) : [],
    islandCount: Number(source.islandCount ?? 0),
    bodyCount: Number(source.bodyCount ?? 0),
    sleepingBodyCount: Number(source.sleepingBodyCount ?? 0),
    awakeBodyCount: Number(source.awakeBodyCount ?? 0)
  };
}

export function buildRigidBodyIslandGraph(options = {}) {
  const bodyRegistry = options.bodyRegistry;
  const manifolds = Array.isArray(options.manifolds) ? options.manifolds : [];
  const joints = Array.isArray(options.joints) ? options.joints : [];
  const bodyIds = [];
  const bodyStateById = new Map();
  const adjacency = new Map();
  const manifoldKeysByBodyId = new Map();
  const manifoldByPairKey = new Map();
  const jointIdsByBodyId = new Map();
  const jointById = new Map();

  bodyRegistry?.forEachMutable((body) => {
    if (!shouldIncludeDynamicBody(body)) {
      return;
    }

    bodyIds.push(body.id);
    bodyStateById.set(body.id, {
      sleeping: Boolean(body.sleeping),
      canSleep: body.canSleep !== false
    });
    adjacency.set(body.id, new Set());
    manifoldKeysByBodyId.set(body.id, new Set());
    jointIdsByBodyId.set(body.id, new Set());
  });

  for (const manifold of manifolds) {
    if (!manifold?.pairKey) {
      continue;
    }

    manifoldByPairKey.set(manifold.pairKey, manifold);

    const dynamicBodyIds = [];
    if (manifold.bodyAId && adjacency.has(manifold.bodyAId)) {
      dynamicBodyIds.push(manifold.bodyAId);
      manifoldKeysByBodyId.get(manifold.bodyAId)?.add(manifold.pairKey);
    }

    if (manifold.bodyBId && adjacency.has(manifold.bodyBId) && manifold.bodyBId !== manifold.bodyAId) {
      dynamicBodyIds.push(manifold.bodyBId);
      manifoldKeysByBodyId.get(manifold.bodyBId)?.add(manifold.pairKey);
    }

    if (dynamicBodyIds.length >= 2) {
      connectBodies(adjacency, dynamicBodyIds[0], dynamicBodyIds[1]);
    }
  }

  for (const joint of joints) {
    if (!joint?.id || joint.enabled === false) {
      continue;
    }

    jointById.set(joint.id, joint);
    const dynamicBodyIds = [];
    if (joint.bodyAId && adjacency.has(joint.bodyAId)) {
      dynamicBodyIds.push(joint.bodyAId);
      jointIdsByBodyId.get(joint.bodyAId)?.add(joint.id);
    }

    if (joint.bodyBId && adjacency.has(joint.bodyBId) && joint.bodyBId !== joint.bodyAId) {
      dynamicBodyIds.push(joint.bodyBId);
      jointIdsByBodyId.get(joint.bodyBId)?.add(joint.id);
    }

    if (dynamicBodyIds.length >= 2) {
      connectBodies(adjacency, dynamicBodyIds[0], dynamicBodyIds[1]);
    }
  }

  const islands = [];
  const visited = new Set();
  let nextIslandId = 1;

  for (const rootBodyId of bodyIds) {
    if (visited.has(rootBodyId)) {
      continue;
    }

    const queue = [rootBodyId];
    const islandBodyIds = [];
    const manifoldPairKeys = new Set();
    const jointIds = new Set();

    while (queue.length > 0) {
      const bodyId = queue.shift();
      if (!bodyId || visited.has(bodyId)) {
        continue;
      }

      visited.add(bodyId);
      islandBodyIds.push(bodyId);

      for (const pairKey of manifoldKeysByBodyId.get(bodyId) ?? []) {
        manifoldPairKeys.add(pairKey);
      }

      for (const jointId of jointIdsByBodyId.get(bodyId) ?? []) {
        jointIds.add(jointId);
      }

      for (const neighborId of adjacency.get(bodyId) ?? []) {
        if (!visited.has(neighborId)) {
          queue.push(neighborId);
        }
      }
    }

    const pairKeyList = Array.from(manifoldPairKeys);
    const jointIdList = Array.from(jointIds);
    let contactCount = 0;
    let touchesStatic = false;
    let sleepingBodyCount = 0;
    let awakeBodyCount = 0;
    let canSleep = true;
    let touchesConstraint = jointIdList.length > 0;

    for (const bodyId of islandBodyIds) {
      const state = bodyStateById.get(bodyId);
      if (!state) {
        continue;
      }

      if (state.sleeping) {
        sleepingBodyCount += 1;
      } else {
        awakeBodyCount += 1;
      }

      canSleep = canSleep && state.canSleep;
    }

    for (const pairKey of pairKeyList) {
      const manifold = manifoldByPairKey.get(pairKey);
      if (!manifold) {
        continue;
      }

      contactCount += Number(manifold.contactCount ?? manifold.contacts?.length ?? 0);
      const dynamicParticipants = [manifold.bodyAId, manifold.bodyBId].filter((bodyId) => bodyId && bodyStateById.has(bodyId));
      if (dynamicParticipants.length < 2) {
        touchesStatic = true;
      }
    }

    for (const jointId of jointIdList) {
      const joint = jointById.get(jointId);
      if (!joint) {
        continue;
      }

      const dynamicParticipants = [joint.bodyAId, joint.bodyBId].filter((bodyId) => bodyId && bodyStateById.has(bodyId));
      if (dynamicParticipants.length < 2) {
        touchesStatic = true;
      }
    }

    islands.push({
      id: `island-${nextIslandId}`,
      bodyIds: islandBodyIds,
      manifoldPairKeys: pairKeyList,
      jointIds: jointIdList,
      manifoldCount: pairKeyList.length,
      jointCount: jointIdList.length,
      constraintCount: pairKeyList.length + jointIdList.length,
      contactCount,
      sleepingBodyCount,
      awakeBodyCount,
      touchesStatic,
      touchesConstraint,
      canSleep
    });
    nextIslandId += 1;
  }

  return {
    islands,
    islandCount: islands.length,
    bodyCount: bodyIds.length,
    sleepingBodyCount: islands.reduce((total, island) => total + island.sleepingBodyCount, 0),
    awakeBodyCount: islands.reduce((total, island) => total + island.awakeBodyCount, 0)
  };
}
