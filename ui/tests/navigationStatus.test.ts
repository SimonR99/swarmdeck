import assert from 'node:assert/strict';
import test from 'node:test';

import { explorationLabel, navigationFailureTooltip } from '../src/lib/components/fleet/navigationStatus.ts';

test('native navigation failure reason is exposed as a Fleet tooltip', () => {
  assert.equal(
    navigationFailureTooltip(
      'failed',
      '  no mapped ground support at (0.01, 0.00, -0.00)  '
    ),
    'Navigation failed: no mapped ground support at (0.01, 0.00, -0.00)'
  );
});

test('missing navigation failure reason keeps the generic status badge', () => {
  assert.equal(navigationFailureTooltip('failed', null), null);
  assert.equal(navigationFailureTooltip('failed', '   '), null);
});

test('stale reasons stay hidden outside the failed navigation state', () => {
  assert.equal(navigationFailureTooltip('active', 'old planner rejection'), null);
  assert.equal(navigationFailureTooltip('idle', 'old planner rejection'), null);
  assert.equal(navigationFailureTooltip('cancelled', 'old planner rejection'), null);
  assert.equal(
    navigationFailureTooltip('failed', 'current planner rejection'),
    'Navigation failed: current planner rejection'
  );
});

test('exploration without an executable route is visibly waiting', () => {
  assert.equal(explorationLabel('starting'), 'EXPLORE STARTING');
  assert.equal(explorationLabel('waiting'), 'EXPLORE WAITING');
  assert.equal(explorationLabel('exploring'), 'EXPLORING');
});
