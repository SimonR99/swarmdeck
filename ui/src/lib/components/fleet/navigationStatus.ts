export function navigationFailureTooltip(
  navStatus: string,
  reason: string | null | undefined
): string | null {
  if (navStatus !== 'failed') return null;
  const value = typeof reason === 'string' ? reason.trim() : '';
  return value ? `Navigation failed: ${value}` : null;
}

export function explorationLabel(status: string | undefined): string {
  if (status === 'starting') return 'EXPLORE STARTING';
  if (status === 'waiting') return 'EXPLORE WAITING';
  return 'EXPLORING';
}
