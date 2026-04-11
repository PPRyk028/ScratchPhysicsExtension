export class BaseRegistry {
  constructor(prefix) {
    this.prefix = prefix;
    this.records = new Map();
    this.nextId = 1;
  }

  allocateId(requestedId) {
    const explicitId = String(requestedId ?? '').trim();
    if (explicitId) {
      return explicitId;
    }

    const allocatedId = `${this.prefix}-${this.nextId}`;
    this.nextId += 1;
    return allocatedId;
  }

  clear() {
    this.records.clear();
    this.nextId = 1;
  }

  count() {
    return this.records.size;
  }

  has(id) {
    return this.records.has(id);
  }

  get(id) {
    const record = this.records.get(id);
    return record ? this.cloneRecord(record) : null;
  }

  getMutable(id) {
    return this.records.get(id) ?? null;
  }

  list() {
    return Array.from(this.records.values(), (record) => this.cloneRecord(record));
  }

  listMutable() {
    return Array.from(this.records.values());
  }

  remove(id) {
    return this.records.delete(id);
  }

  forEachMutable(visitor) {
    for (const record of this.records.values()) {
      visitor(record);
    }
  }

  store(record) {
    this.records.set(record.id, record);
    return this.cloneRecord(record);
  }

  cloneRecord(record) {
    return record;
  }
}
