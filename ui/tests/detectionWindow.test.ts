import { test } from 'node:test';
import assert from 'node:assert/strict';
import { DETECTION_LIMIT, capDetections } from '../src/lib/stores/detectionWindow.ts';

function tracks(count: number, from = 0) {
  return Array.from({ length: count }, (_, i) => ({ id: `d${from + i}`, received_at: from + i }));
}

test('a session under the limit is left exactly as it is', () => {
  const detections = tracks(DETECTION_LIMIT);
  assert.equal(capDetections(detections), detections);
});

test('a long mission keeps the most recent tracks and drops the oldest', () => {
  const capped = capDetections(tracks(DETECTION_LIMIT + 40));
  assert.equal(capped.length, DETECTION_LIMIT);
  assert.equal(capped[0].id, 'd40');
  assert.equal(capped[capped.length - 1].id, `d${DETECTION_LIMIT + 39}`);
});

test('a track that keeps being re-seen is kept, wherever it sits in the list', () => {
  const detections = tracks(10, 0);
  // The oldest entry by position was just re-reported, so it is the freshest.
  detections[0] = { id: 'd0', received_at: 999 };
  const capped = capDetections(detections, 3);
  assert.deepEqual(
    capped.map((detection) => detection.id),
    ['d0', 'd8', 'd9']
  );
});
