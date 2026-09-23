import type { RobotState } from '../types/protocol.ts';
import { sameFieldValue } from './sameFieldValue.ts';

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
