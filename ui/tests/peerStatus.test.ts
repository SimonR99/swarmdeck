import assert from 'node:assert/strict';
import test from 'node:test';
import { summarizePeerSlam } from '../src/lib/components/slam/peerStatus.ts';

const report = (id: string, other: string, count: number, online = true) => ({
  robot_id: id, online,
  peer_slam: { robot_id: id, mission_id: 'mission', keyframes: 5,
    verified: count, rejected: 0, by_peer: { [other]: count } }
});

test('both peers reporting one closure does not double its fleet count', () => {
  assert.deepEqual(summarizePeerSlam([report('r0', 'r1', 3), report('r1', 'r0', 2)]),
    { reporters: 2, keyframes: 10, closures: 3 });
});

test('offline and other-robot reports do not masquerade as current SLAM', () => {
  assert.deepEqual(summarizePeerSlam([
    report('r0', 'r1', 3, false),
    { ...report('r1', 'r0', 2), robot_id: 'r2' }
  ]), { reporters: 0, keyframes: 0, closures: 0 });
});
