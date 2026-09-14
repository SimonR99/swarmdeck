"""Pure capture-selection seam used before optional measured color projection."""


def message_stamp_ns(message):
    stamp = message.header.stamp
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("invalid ROS timestamp")
    return sec * 1_000_000_000 + nanosec


def validated_image_bytes(message, kind, max_bytes):
    """Return a ROS Image payload size, or ``None`` when unsafe to retain."""
    try:
        width, height, step = (
            int(message.width), int(message.height), int(message.step)
        )
        encoding = str(message.encoding).lower()
        size = memoryview(message.data).nbytes
    except (AttributeError, TypeError, ValueError):
        return None
    bytes_per_pixel = (
        {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
        if kind == "color"
        else {"16uc1": 2, "mono16": 2, "32fc1": 4}.get(encoding)
        if kind == "depth"
        else None
    )
    if (
        bytes_per_pixel is None
        or width < 2
        or height < 2
        or step < width * bytes_per_pixel
        or size != height * step
        or size > max_bytes
    ):
        return None
    return size


def retain_bounded_image(
    cache, message, kind, max_records, max_bytes, max_message_bytes=None,
):
    """Validate and retain a frame in an insertion-ordered bounded mapping."""
    try:
        stamp = message_stamp_ns(message)
    except (AttributeError, TypeError, ValueError):
        return False
    message_limit = max_bytes if max_message_bytes is None else min(
        max_bytes, max_message_bytes,
    )
    size = validated_image_bytes(message, kind, message_limit)
    if size is None:
        return False
    # Refresh retransmissions so a useful repeated stamp is not immediately
    # evicted as if it were still the oldest callback in the stream.
    cache.pop(stamp, None)
    cache[stamp] = message
    retained_bytes = sum(memoryview(item.data).nbytes for item in cache.values())
    while len(cache) > max_records or retained_bytes > max_bytes:
        oldest = cache.pop(next(iter(cache)))
        retained_bytes -= memoryview(oldest.data).nbytes
    return True


def select_geometry_and_color(frontend_xyz, calibration, raw, colorize):
    mount, sensor_frame = calibration
    provenance = None
    xyz = frontend_xyz
    if raw is not None:
        xyz, mount, sensor_frame, provenance = raw
    # The projector must see the exact ordered array passed to storage. A raw
    # qualified capture commonly differs from the frontend's sampled cloud.
    colors = colorize(xyz)
    return xyz, mount, sensor_frame, provenance, colors


def qualified_rgbd_frame(
    cloud_ns, image_ns, depth_ns, image_frame, depth_frame, info_frame,
    configured_frame="",
):
    if abs(image_ns - depth_ns) > 50_000_000 or abs(image_ns - cloud_ns) > 250_000_000:
        return None
    frames = {value for value in (image_frame, depth_frame, info_frame) if value}
    if len(frames) > 1:
        return None
    measured = next(iter(frames), "")
    if configured_frame and measured and configured_frame != measured:
        return None
    return configured_frame or measured or None


def select_rgbd_observation(
    cloud_ns, images, depths, info, configured_frame="",
):
    """Select a timestamp-qualified pair despite cross-topic callback ordering.

    ROS preserves order within one topic, not between the image and depth data
    writers. Callers therefore retain a small bounded history of each stream.
    Prefer the most tightly synchronized pair, then the image nearest the scan.
    """
    if info is None:
        return None
    try:
        info_frame = info.header.frame_id
    except AttributeError:
        return None
    best = None
    for image in images:
        try:
            image_ns = message_stamp_ns(image)
            image_frame = image.header.frame_id
        except (AttributeError, TypeError, ValueError):
            continue
        for depth in depths:
            try:
                depth_ns = message_stamp_ns(depth)
                depth_frame = depth.header.frame_id
            except (AttributeError, TypeError, ValueError):
                continue
            frame = qualified_rgbd_frame(
                cloud_ns,
                image_ns,
                depth_ns,
                image_frame,
                depth_frame,
                info_frame,
                configured_frame,
            )
            if frame is None:
                continue
            score = (abs(image_ns - depth_ns), abs(image_ns - cloud_ns), -image_ns)
            if best is None or score < best[0]:
                best = (score, image, depth, frame)
    return None if best is None else best[1:]
