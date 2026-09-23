/**
 * Structural comparison of two decoded JSON values.
 *
 * The dashboard polls and is pushed to several times a second, and every
 * response decodes into fresh objects. Comparing them by value is what lets a
 * store keep the reference it already published when the content is the same,
 * which is what keeps the renderers from rebuilding and the effects from
 * re-running for a message that said nothing new.
 */
export function sameFieldValue(a: unknown, b: unknown): boolean {
  if (a === b) return true;
  if (typeof a !== 'object' || typeof b !== 'object' || a === null || b === null) return false;
  const arrayA = Array.isArray(a);
  if (arrayA !== Array.isArray(b)) return false;
  if (arrayA) {
    const left = a as unknown[];
    const right = b as unknown[];
    if (left.length !== right.length) return false;
    for (let i = 0; i < left.length; i++) if (!sameFieldValue(left[i], right[i])) return false;
    return true;
  }
  const left = a as Record<string, unknown>;
  const right = b as Record<string, unknown>;
  const keys = Object.keys(left);
  if (keys.length !== Object.keys(right).length) return false;
  for (const key of keys) {
    if (!Object.hasOwn(right, key)) return false;
    if (!sameFieldValue(left[key], right[key])) return false;
  }
  return true;
}

/** The previous value when the new one says the same thing, else the new one. */
export function keepIfUnchanged<T>(previous: T, next: T): T {
  return sameFieldValue(previous, next) ? previous : next;
}
