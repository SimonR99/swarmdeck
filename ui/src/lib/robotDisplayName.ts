const ROBOT_ALIASES: Readonly<Record<string, string>> = {
  tars_0: 'scout'
};

/** Human-friendly UI label; the protocol-facing robot ID remains unchanged. */
export function robotDisplayName(robotId: string): string {
  return ROBOT_ALIASES[robotId] ?? robotId.replace(/^robot_/, 'R');
}
