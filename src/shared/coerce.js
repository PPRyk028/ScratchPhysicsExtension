export function toNumber(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

export function toString(value, fallback = '') {
  const parsed = String(value ?? '').trim();
  return parsed || fallback;
}

