/** Keep world coordinates stationary when a raster grows or changes resolution. */
export function rebaseViewport(
  view: { scale: number; tx: number; ty: number; rotation: number },
  previous: { resolution: number; height: number; origin: { x: number; y: number } },
  next: { resolution: number; height: number; origin: { x: number; y: number } }
) {
  const pixelsPerMetre = view.scale / previous.resolution;
  const dx = (previous.origin.x - next.origin.x) * pixelsPerMetre;
  const dy = (next.height * next.resolution - previous.height * previous.resolution
    + next.origin.y - previous.origin.y) * pixelsPerMetre;
  const c = Math.cos(view.rotation), s = Math.sin(view.rotation);
  return {
    scale: pixelsPerMetre * next.resolution,
    tx: view.tx - (dx * c - dy * s),
    ty: view.ty - (dx * s + dy * c)
  };
}
