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
  return sameStructure(a, b, sameExactly);
}

/**
 * Numbers closer than this are drawn identically: a micrometre, or a
 * microradian. The server recomputes poses through their frame transforms and
 * resends a parked robot with different last digits in every keep-alive; it
 * compares at the same resolution (`STATE_SIGNATURE_FLOAT_DIGITS` in
 * `server/swarmdeck_server/api/app.py`) when it decides whether to broadcast.
 */
export const DRAWN_VALUE_DIGITS = 6;
const DRAWN_VALUE_SCALE = 10 ** DRAWN_VALUE_DIGITS;

/**
 * Structural comparison with numbers rounded to DRAWN_VALUE_DIGITS, for
 * deciding whether a map has to redraw. Rounding rather than a tolerance means
 * a slow crawl still crosses a rounding step within one micrometre, so an
 * undrawn change never accumulates.
 */
export function sameDrawnValue(a: unknown, b: unknown): boolean {
  return sameStructure(a, b, sameRounded);
}

function sameExactly(a: unknown, b: unknown): boolean {
  return a === b;
}

function sameRounded(a: unknown, b: unknown): boolean {
  if (typeof a === 'number' && typeof b === 'number') {
    return Math.round(a * DRAWN_VALUE_SCALE) === Math.round(b * DRAWN_VALUE_SCALE);
  }
  return a === b;
}

function sameStructure(a: unknown, b: unknown, sameLeaf: (a: unknown, b: unknown) => boolean): boolean {
  if (sameLeaf(a, b)) return true;
  if (typeof a !== 'object' || typeof b !== 'object' || a === null || b === null) return false;
  const arrayA = Array.isArray(a);
  if (arrayA !== Array.isArray(b)) return false;
  if (arrayA) {
    const left = a as unknown[];
    const right = b as unknown[];
    if (left.length !== right.length) return false;
    for (let i = 0; i < left.length; i++) if (!sameStructure(left[i], right[i], sameLeaf)) return false;
    return true;
  }
  const left = a as Record<string, unknown>;
  const right = b as Record<string, unknown>;
  const keys = Object.keys(left);
  if (keys.length !== Object.keys(right).length) return false;
  for (const key of keys) {
    if (!Object.hasOwn(right, key)) return false;
    if (!sameStructure(left[key], right[key], sameLeaf)) return false;
  }
  return true;
}

/** The previous value when the new one says the same thing, else the new one. */
export function keepIfUnchanged<T>(previous: T, next: T): T {
  return sameFieldValue(previous, next) ? previous : next;
}
