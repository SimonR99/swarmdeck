/** How often the merged poll wakes up to see what is due. */
export const MAP_POLL_TICK_MS = 500;
/** The replica raster catalogue: which scopes exist and how far each has got. */
export const MAP_SCOPES_INTERVAL_MS = 2000;
/** Merge status, registrations and SLAM graphs. */
export const MAP_STATUS_INTERVAL_MS = 3000;
/** The raster image itself, reloaded only when its scope or sequence moved. */
export const MAP_RASTER_INTERVAL_MS = 2000;

export interface MapPollInputs {
  now: number;
  /** The tab is not on screen. */
  hidden: boolean;
  /** The 2D canvas, the only thing that draws the raster image, is showing. */
  rasterVisible: boolean;
  /** No raster has been displayed yet, so one is worth fetching regardless. */
  rasterReady: boolean;
}

export interface MapPollWork {
  scopes: boolean;
  status: boolean;
  raster: boolean;
}

/**
 * One timer for the map's HTTP polling.
 *
 * The dashboard used to run two overlapping loops — a two-second raster
 * refresh in the connection and a three-second status refresh in the map view
 * — each of which also fetched the scope catalogue, so the index was fetched
 * about twice as often as either loop asked for and the raster was decoded
 * while the 3D view was covering the canvas. They now share one tick, each
 * part keeps its own cadence, and nothing is fetched while the tab is hidden.
 */
export class MapPollScheduler {
  private lastScopes = Number.NEGATIVE_INFINITY;
  private lastStatus = Number.NEGATIVE_INFINITY;
  private lastRaster = Number.NEGATIVE_INFINITY;

  due(inputs: MapPollInputs): MapPollWork {
    const idle: MapPollWork = { scopes: false, status: false, raster: false };
    // A hidden tab keeps its deadlines: whatever was due while it was away is
    // due at once when it comes back.
    if (inputs.hidden) return idle;

    const work: MapPollWork = {
      scopes: inputs.now - this.lastScopes >= MAP_SCOPES_INTERVAL_MS,
      status: inputs.now - this.lastStatus >= MAP_STATUS_INTERVAL_MS,
      raster:
        (inputs.rasterVisible || !inputs.rasterReady) &&
        inputs.now - this.lastRaster >= MAP_RASTER_INTERVAL_MS
    };
    if (work.scopes) this.lastScopes = inputs.now;
    if (work.status) this.lastStatus = inputs.now;
    if (work.raster) this.lastRaster = inputs.now;
    return work;
  }

  /** Make every part due again, after a reset or a view the raster must follow. */
  invalidate() {
    this.lastScopes = Number.NEGATIVE_INFINITY;
    this.lastStatus = Number.NEGATIVE_INFINITY;
    this.lastRaster = Number.NEGATIVE_INFINITY;
  }
}
