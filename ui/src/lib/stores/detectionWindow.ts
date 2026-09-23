/**
 * Live camera tracks kept at once.
 *
 * These are what a robot can see this instant — every one of them is retracted
 * by the server the moment it leaves frame, and `bboxesFor` shows only the
 * last two seconds. What the fleet has agreed is actually there lives in the
 * review store instead. Without a cap, a long mission accumulates every track
 * it ever saw, and each one is then scanned on every camera frame.
 */
export const DETECTION_LIMIT = 200;

/**
 * The most recently received detections, in the order the store holds them.
 * Returns the same array when nothing has to be dropped.
 */
export function capDetections<T extends { received_at: number }>(
  detections: T[],
  limit = DETECTION_LIMIT
): T[] {
  if (detections.length <= limit) return detections;
  const kept = new Set(
    [...detections].sort((a, b) => b.received_at - a.received_at).slice(0, limit)
  );
  return detections.filter((detection) => kept.has(detection));
}
