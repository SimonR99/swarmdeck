import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  ALERT_TOAST_TONES,
  contrastRatio,
  legibleOn,
  toastBackground
} from '../src/lib/components/alerts/alertToast.ts';

const AA = 4.5;

test('contrast follows the WCAG definition', () => {
  assert.equal(contrastRatio('#000000', '#ffffff').toFixed(2), '21.00');
  assert.equal(contrastRatio('#ffffff', '#ffffff'), 1);
});

test('every toast level reads at WCAG AA, message and secondary text alike', () => {
  for (const [level, tone] of Object.entries(ALERT_TOAST_TONES)) {
    const background = toastBackground(tone);
    for (const role of ['text', 'secondary'] as const) {
      const ratio = contrastRatio(tone[role], background);
      assert.ok(ratio >= AA, `${level} ${role} ${tone[role]} on ${background}: ${ratio.toFixed(2)}`);
    }
  }
});

/** Every colour on a 0x33 grid: the web-safe cube, dark and saturated ones included. */
const anyRobotColor = [0, 0x33, 0x66, 0x99, 0xcc, 0xff].flatMap((r) =>
  [0, 0x33, 0x66, 0x99, 0xcc, 0xff].flatMap((g) =>
    [0, 0x33, 0x66, 0x99, 0xcc, 0xff].map(
      (b) => `#${[r, g, b].map((c) => c.toString(16).padStart(2, '0')).join('')}`
    )
  )
);

test('a robot name keeps its hue but is lightened until it reads on every toast', () => {
  for (const tone of Object.values(ALERT_TOAST_TONES)) {
    const background = toastBackground(tone);
    for (const color of anyRobotColor) {
      const shown = legibleOn(color, background);
      assert.ok(contrastRatio(shown, background) >= AA, `${color} -> ${shown} on ${background}`);
    }
  }
  // A colour that already reads is left alone.
  assert.equal(legibleOn('#ffffff', '#1e242d'), '#ffffff');
});

test('a robot colour that is not #rrggbb falls back to the secondary text', () => {
  const tone = ALERT_TOAST_TONES.warn;
  assert.equal(legibleOn('rebeccapurple', toastBackground(tone), tone.secondary), tone.secondary);
});
