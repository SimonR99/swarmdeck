"""Run from the repository root with PYTHONPATH=. (no ROS required)."""

from statistics import median
from time import perf_counter_ns
from types import SimpleNamespace as NS

from autonomy.capture_color import retain_bounded_image, select_rgbd_observation


def image(stamp_ns, encoding, channels):
    return NS(
        header=NS(stamp=NS(sec=1, nanosec=stamp_ns), frame_id="camera"),
        width=1920,
        height=1080,
        step=1920 * channels,
        encoding=encoding,
        data=bytes(1920 * 1080 * channels),
    )


cache = {}
frames = [image(i * 10_000_000, "rgb8", 3) for i in range(9)]
samples = []
for i in range(10000):
    frame = frames[i % len(frames)]
    start = perf_counter_ns()
    assert retain_bounded_image(cache, frame, "color", 8, 64 << 20, 16 << 20)
    samples.append(perf_counter_ns() - start)
assert len(cache) == 8
assert all(any(retained is frame for frame in frames) for retained in cache.values())
print(f"1080p retain_bounded_image median={median(samples) / 1000:.3f}us")

# A legal pre-window pair the old cache selects, but post-cloud gating loses.
color = image(0, "rgb8", 3)
depth = image(0, "32FC1", 4)
info = NS(header=NS(frame_id="camera"))
assert select_rgbd_observation(1_100_000_000, [color], [depth], info) is not None
assert select_rgbd_observation(1_100_000_000, [], [], info) is None
print("100ms pre-cloud pair selected; window-only cache loses it (synthetic example)")
