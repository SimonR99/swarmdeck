import type { Point } from '$lib/types/protocol';

const state = $state({
  goalMode: false,
  // Explore toward a goal the map does not reach yet, instead of failing it.
  exploreIfUnknown: false,
  lastTarget: null as Point | null
});

export const navigation = {
  get goalMode() {
    return state.goalMode;
  },
  get lastTarget() {
    return state.lastTarget;
  },
  get exploreIfUnknown() {
    return state.exploreIfUnknown;
  },
  setExploreIfUnknown(value: boolean) {
    state.exploreIfUnknown = value;
  },
  toggleGoalMode() {
    state.goalMode = !state.goalMode;
  },
  cancelGoalMode() {
    state.goalMode = false;
  },
  finishGoal(target: Point) {
    state.lastTarget = target;
    state.goalMode = false;
  }
};
