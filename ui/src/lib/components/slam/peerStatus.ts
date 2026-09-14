import type { RobotState } from '../../types/protocol';

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
