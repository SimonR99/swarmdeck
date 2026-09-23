import { untrack } from 'svelte';
import { fetchReplicaCatalogue } from '$lib/components/replicas/replicaCatalogue';
import {
  ReplicaCataloguePoller,
  type ReplicaCatalogueSnapshot
} from '$lib/components/replicas/replicaCataloguePoll';
import { keepIfUnchanged } from './sameFieldValue';

const state = $state<ReplicaCatalogueSnapshot>({ catalogue: null, error: '', loading: false });

const poller = new ReplicaCataloguePoller(
  (signal) => fetchReplicaCatalogue(undefined, signal),
  (snapshot) => {
    // A poll that says what the last one said keeps the published catalogue,
    // so the views reading it are not re-run for nothing.
    state.catalogue = keepIfUnchanged(state.catalogue, snapshot.catalogue);
    state.error = snapshot.error;
    state.loading = snapshot.loading;
  }
);

/** The replica component catalogue, polled once for every view that reads it. */
export const replicaCatalogue = {
  get catalogue() {
    return state.catalogue;
  },
  get error() {
    return state.error;
  },
  get loading() {
    return state.loading;
  },
  /**
   * Keep the catalogue polled while the caller needs it; call the result to
   * stop. Safe inside an effect: the refresh it starts is not a dependency.
   */
  subscribe(intervalMs?: number): () => void {
    return untrack(() => poller.subscribe(intervalMs));
  },
  refresh(): Promise<void> {
    return untrack(() => poller.refresh());
  }
};
