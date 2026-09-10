import assert from 'node:assert/strict';
import { test } from 'node:test';
import {
  CameraStreamGate,
  cameraRetryDelayMs,
  decodedFrameIsStalled,
  hlsCameraUrl
} from '../src/lib/video/cameraStream.ts';

test('switching robot or transport fences stale decoded-frame callbacks', () => {
  const gate = new CameraStreamGate();
  const oldWebrtc = gate.begin('robot_0', 'webrtc');
  const currentHls = gate.begin('robot_0', 'hls');

  assert.equal(gate.isCurrent(oldWebrtc, 'robot_0'), false);
  assert.equal(gate.isCurrent(currentHls, 'robot_0'), true);
  assert.equal(gate.isCurrent(currentHls, 'robot_1'), false);

  gate.invalidate();
  assert.equal(gate.isCurrent(currentHls, 'robot_0'), false);
});

test('a continuation resolving after fallback cannot reclaim the stream', async () => {
  const gate = new CameraStreamGate();
  const whep = gate.begin('robot_0', 'webrtc');
  let release!: () => void;
  const pending = new Promise<void>((resolve) => {
    release = resolve;
  });
  const continuation = pending.then(() => gate.isCurrent(whep, 'robot_0'));

  const hls = gate.begin('robot_0', 'hls');
  release();

  assert.equal(await continuation, false);
  assert.equal(gate.isCurrent(hls, 'robot_0'), true);
});

test('HLS camera paths encode the robot id as one path segment', () => {
  assert.equal(hlsCameraUrl('robot_1'), '/hls/robot_1/index.m3u8');
  assert.equal(hlsCameraUrl('robot/odd'), '/hls/robot%2Fodd/index.m3u8');
});

test('camera retry delay is bounded exponential backoff', () => {
  assert.deepEqual(
    [0, 1, 2, 3, 4, 5, 9].map(cameraRetryDelayMs),
    [2_000, 2_000, 4_000, 8_000, 16_000, 30_000, 30_000]
  );
});

test('decoded-frame health cannot treat an absent or stale frame as live', () => {
  assert.equal(decodedFrameIsStalled(0, 10_000, 3_000), false);
  assert.equal(decodedFrameIsStalled(7_001, 10_000, 3_000), false);
  assert.equal(decodedFrameIsStalled(6_999, 10_000, 3_000), true);
});

test('a stale frame is hidden before the transport is rebuilt', () => {
  const lastFrameAt = 1_000;
  const now = 4_500;
  assert.equal(decodedFrameIsStalled(lastFrameAt, now, 3_000), true);
  assert.equal(decodedFrameIsStalled(lastFrameAt, now, 10_000), false);
  assert.equal(decodedFrameIsStalled(lastFrameAt, 11_001, 10_000), true);
});
