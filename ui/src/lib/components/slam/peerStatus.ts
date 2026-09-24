import type { RobotState } from '../../types/protocol';
import type { ReplicaCatalogueEntry } from '../replicas/replicaCatalogue.ts';
import type { ReplicaCatalogueSnapshot } from '../replicas/replicaCataloguePoll.ts';

/**
 * The largest verified multi-robot component of the current mission, as the
 * Peer SLAM panel reports it. The deployment composite places every robot by
 * surveyed start pose; it is not a verified merge and must not count. While
 * the catalogue's latest refresh failed or timed out, nothing is reported as
 * merged: the panel never shows a merge it could not confirm.
 */
export function peerMergeComponent(
  snapshot: Pick<ReplicaCatalogueSnapshot, 'catalogue' | 'refreshFailed'>
): ReplicaCatalogueEntry | null {
  const catalogue = snapshot.catalogue;
  if (!catalogue || snapshot.refreshFailed) return null;
  return catalogue.components
    .filter((entry) => entry.available && entry.status === 'ready' &&
      entry.session_id === catalogue.active_session_id && !entry.composite &&
      entry.robot_ids.length >= 2)
    .sort((a, b) => b.robot_ids.length - a.robot_ids.length)[0] ?? null;
}

/** Pair-max symmetric peer reports; these are verification messages, not unique graph edges. */
export function summarizePeerSlam(robots: Pick<RobotState, 'online' | 'robot_id' | 'peer_slam'>[]) {
  const pairs = new Map<string, number>();
  let keyframes = 0, reporters = 0;
  for (const robot of robots) {
    const status = robot.peer_slam;
    if (!robot.online || !status || status.robot_id !== robot.robot_id) continue;
    reporters++;
    keyframes += status.keyframes;
    for (const [other, count] of Object.entries(status.by_peer)) {
      const key = JSON.stringify([status.mission_id, ...[robot.robot_id, other].sort()]);
      pairs.set(key, Math.max(pairs.get(key) ?? 0, count));
    }
  }
  return { reporters, keyframes, closures: [...pairs.values()].reduce((a, b) => a + b, 0) };
}
