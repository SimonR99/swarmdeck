import type { Point } from '$lib/types/protocol';

const state = $state({
  goalMode: false,
  lastTarget: null as Point | null
});

export const navigation = {
  get goalMode() {
    return state.goalMode;
  },
  get lastTarget() {
    return state.lastTarget;
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
