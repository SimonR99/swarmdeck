import type { RobotState } from '../types/protocol.ts';
import { sameDrawnValue, sameFieldValue } from './sameFieldValue.ts';

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

/**
 * The same kind of clock inside `live_mapping`: how long ago the mapping
 * authority last reported, refreshed by the server in every keep-alive.
 */
const SELF_ADVANCING_LIVE_MAPPING_FIELDS: ReadonlySet<string> = new Set(['authority_age_s']);

/**
 * Whether a field changed in a way a map could show. Mirrors the server's
 * broadcast signature (`_robot_state_signature`): the clocks are left out and
 * numbers are compared to a micrometre, so the keep-alive of a parked robot,
 * which differs from the last message only there, does not redraw the maps.
 */
function drawnFieldChanged(key: string, previous: unknown, next: unknown): boolean {
  if (SELF_ADVANCING_FIELDS.has(key)) return false;
  if (key === 'live_mapping') return !sameDrawnValue(withoutClocks(previous), withoutClocks(next));
  return !sameDrawnValue(previous, next);
}

function withoutClocks(liveMapping: unknown): unknown {
  if (!liveMapping || typeof liveMapping !== 'object' || Array.isArray(liveMapping)) return liveMapping;
  return Object.fromEntries(
    Object.entries(liveMapping).filter(([key]) => !SELF_ADVANCING_LIVE_MAPPING_FIELDS.has(key))
  );
}

export interface RobotStateUpdate {
  /** `previous` itself when the message carried nothing new. */
  value: RobotState;
  /** Any field changed, so the store must hold the new value. */
  changed: boolean;
  /**
   * A field the maps draw changed, so they must redraw. Clocks and
   * sub-micrometre recomputation noise are stored but do not count.
   */
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
    const held = (previous as unknown as Record<string, unknown>)[key];
    if (sameFieldValue(held, value)) continue;
    merged[key] = value;
    changed = true;
    if (!drawable && drawnFieldChanged(key, held, value)) drawable = true;
  }
  return changed
    ? { value: merged as unknown as RobotState, changed, drawable }
    : { value: previous, changed: false, drawable: false };
}
