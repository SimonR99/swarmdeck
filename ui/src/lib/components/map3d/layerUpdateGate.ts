/** Layer rebuild budget; freshness invalidations must survive until a draw. */
export class LayerUpdateGate {
  private lastUpdate = 0;

  private freshnessPending = false;

  invalidateFreshness() {
    this.freshnessPending = true;
  }

  take(now: number): boolean {
    if (!this.freshnessPending && now - this.lastUpdate < 200) return false;
    this.freshnessPending = false;
    this.lastUpdate = now;
    return true;
  }
}
