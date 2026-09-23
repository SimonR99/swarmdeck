import type { RobotState } from '../types/protocol.ts';

/**
 * Structural comparison of one `robot_state` field.
 *
 * Telemetry arrives as freshly parsed JSON, so every field of every message is
 * a new object even when the robot has not moved. Comparing by value is what
 * lets the store keep the previous reference, which in turn is what lets the
 * renderers detect change by identity instead of stringifying the fleet.
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

/**
 * Fields that advance on their own clock in every broadcast, whether or not
 * the robot did anything: the three record timestamps and the unattended
 * timer. They are displayed, so they are stored, but a map that redrew for
 * them would redraw for ever.
 */
export const SELF_ADVANCING_FIELDS: ReadonlySet<string> = new Set([
  't_wall',
  't_mono',
  't_sess',
  'unattended_s'
]);

export interface RobotStateUpdate {
  /** `previous` itself when the message carried nothing new. */
  value: RobotState;
  /** Any field changed, so the store must hold the new value. */
  changed: boolean;
  /** A field the maps draw changed, so they must redraw. */
  drawable: boolean;
}

/**
 * Fold a `robot_state` message into the state already held for that robot.
 *
 * Reports `previous` unchanged when the message carries nothing new, so an
 * unchanged robot — including the server's keep-alive for a robot that has not
 * moved — performs no store write at all. Fields that did change are taken
 * from the message; fields that did not keep their previous reference.
 */
export function mergeRobotState(
  previous: RobotState | undefined,
  next: RobotState
): RobotStateUpdate {
  if (!previous) return { value: next, changed: true, drawable: true };
  const merged: Record<string, unknown> = { ...previous };
  let changed = false;
  let drawable = false;
  for (const [key, value] of Object.entries(next)) {
    if (sameFieldValue((previous as unknown as Record<string, unknown>)[key], value)) continue;
    merged[key] = value;
    changed = true;
    if (!SELF_ADVANCING_FIELDS.has(key)) drawable = true;
  }
  return changed
    ? { value: merged as unknown as RobotState, changed, drawable }
    : { value: previous, changed: false, drawable: false };
}
