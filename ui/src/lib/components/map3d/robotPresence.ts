export class RobotPresenceTracker {
  private sourceIdentity: string | null = null;
  private lastSeen = new Map<string, number>();
  private readonly graceSeconds: number;

  constructor(graceSeconds = 3) {
    this.graceSeconds = graceSeconds;
  }

  update(ids: Iterable<string>, time: number, sourceIdentity: string) {
    const reset = this.sourceIdentity !== null && this.sourceIdentity !== sourceIdentity;
    if (reset) this.lastSeen.clear();
    this.sourceIdentity = sourceIdentity;
    for (const id of ids) this.lastSeen.set(id, time);
    for (const [id, seen] of this.lastSeen) {
      if (time - seen > this.graceSeconds) this.lastSeen.delete(id);
    }
    return { reset, visible: new Set(this.lastSeen.keys()) };
  }

  clear() {
    this.sourceIdentity = null;
    this.lastSeen.clear();
  }
}
