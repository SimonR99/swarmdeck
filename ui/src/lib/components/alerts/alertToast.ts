/**
 * Colours of the alert toasts that float over the map.
 *
 * The toasts are dark cards: the map's slate with a faint tint of the level's
 * hue. The theme's accent, warn and danger colours are made for light
 * surfaces and read at under 3:1 here, so the text uses light tints of the
 * same hues, chosen to clear WCAG AA (4.5:1) on the card; a test holds them to
 * it.
 */

/** The map's background slate, which the toasts sit on. */
export const ALERT_TOAST_BASE = '#1e242d';

export interface AlertToastTone {
  /** The level's theme hue, used for the tint and the border. */
  hue: string;
  /** How much of the hue is layered over the base. */
  tint: number;
  /** The message, and the icon through currentColor. */
  text: string;
  /** The detail line. */
  secondary: string;
}

export const ALERT_TOAST_TONES = {
  info: { hue: '#2f63c7', tint: 0.1, text: '#b3cdfa', secondary: '#93b4ee' },
  warn: { hue: '#b45309', tint: 0.1, text: '#f8c99a', secondary: '#e5ac78' },
  critical: { hue: '#dc2626', tint: 0.12, text: '#fbb4b4', secondary: '#f09494' }
} as const satisfies Record<string, AlertToastTone>;

type Rgb = [number, number, number];

function parseHex(color: string): Rgb | null {
  const match = /^#([0-9a-f]{6})$/i.exec(color.trim());
  if (!match) return null;
  const value = parseInt(match[1], 16);
  return [(value >> 16) & 255, (value >> 8) & 255, value & 255];
}

function toHex(rgb: Rgb): string {
  return `#${rgb.map((c) => Math.round(c).toString(16).padStart(2, '0')).join('')}`;
}

function mix(from: Rgb, to: Rgb, amount: number): Rgb {
  return from.map((c, i) => c + (to[i] - c) * amount) as Rgb;
}

function luminance([r, g, b]: Rgb): number {
  const channel = (c: number) => {
    const s = c / 255;
    return s <= 0.04045 ? s / 12.92 : ((s + 0.055) / 1.055) ** 2.4;
  };
  return 0.2126 * channel(r) + 0.7152 * channel(g) + 0.0722 * channel(b);
}

/** WCAG contrast ratio of two #rrggbb colours, from 1 to 21. */
export function contrastRatio(a: string, b: string): number {
  const la = luminance(parseHex(a) ?? [0, 0, 0]);
  const lb = luminance(parseHex(b) ?? [0, 0, 0]);
  return (Math.max(la, lb) + 0.05) / (Math.min(la, lb) + 0.05);
}

/** The opaque card colour of a tone: its tint over the base. */
export function toastBackground(tone: AlertToastTone): string {
  return toHex(mix(parseHex(ALERT_TOAST_BASE)!, parseHex(tone.hue)!, tone.tint));
}

/**
 * `color` lightened toward white just enough to reach 4.5:1 on `background`,
 * so a robot's name keeps its identity hue and stays readable. A colour that
 * is not #rrggbb is replaced by `fallback`.
 */
export function legibleOn(color: string, background: string, fallback = '#ffffff'): string {
  const rgb = parseHex(color);
  if (!rgb) return fallback;
  for (let step = 0; step <= 20; step++) {
    const candidate = toHex(mix(rgb, [255, 255, 255], step / 20));
    if (contrastRatio(candidate, background) >= 4.5) return candidate;
  }
  return '#ffffff';
}
