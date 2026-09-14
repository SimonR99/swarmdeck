"""Decode navigation results without depending on a particular ROS release."""


def navigation_failure_reason(result):
    parts = []
    for field in ("error_msg", "message"):
        value = str(getattr(result, field, "") or "")
        if value and value not in parts:
            parts.append(value)
    code = getattr(result, "error_code", None)
    progress_code = getattr(result, "FAILED_TO_MAKE_PROGRESS", None)
    # Some Nav2 controllers return the typed error but leave error_msg empty.
    # Use the installed action's constant; numeric codes vary by action/release.
    if (
        type(code) is int
        and type(progress_code) is int
        and code == progress_code != 0
        and not any("failed to make progress" in part.lower() for part in parts)
    ):
        parts.insert(0, "Failed to make progress")
    if code not in (None, "", 0, "0"):
        parts.append(f"error_code={code}")
    return "; ".join(parts) or None
